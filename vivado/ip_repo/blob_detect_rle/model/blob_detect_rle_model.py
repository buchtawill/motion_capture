"""
blob_detect_rle_model.py — Functional model for the blob_detect_rle IP.

Produces bit-exact integer results matching the RTL computation:
  1. Parse hex file → pixel array (LSB-first packing, 4 px per 32-bit word).
  2. For each row, extract contiguous foreground runs (xs, xe).
  3. Merge runs across rows using union-find (8-connected overlap).
  4. Accumulate per-blob statistics (count, sum_x, sum_y, bbox).
  5. Flatten and sort blobs by (ymin, xmin).

Usage (standalone):
  python blob_detect_rle_model.py \\
      --input   frame_0000.hex \\
      --width   1280 \\
      --height  800 \\
      --threshold 128 \\
      --output  blobs_rle_model.json
"""

import argparse
import json
import sys

MAX_BLOBS = 64  # matches the deployed mocap wrapper's blob_table capacity


# ---------------------------------------------------------------------------
# Hex file I/O  (identical packing to blob_detect_grid_model)
# ---------------------------------------------------------------------------

def load_hex_frame(hex_path: str, width: int, height: int) -> list:
    """
    Parse a $readmemh-style hex file (one 32-bit word per line).
    Return flat list of pixel values, length == width * height.
    LSB-first packing: bits [7:0] → px0, [15:8] → px1, [23:16] → px2, [31:24] → px3.
    """
    pixels = []
    with open(hex_path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('//'):
                continue
            word = int(line, 16)
            pixels.append( word        & 0xFF)
            pixels.append((word >>  8) & 0xFF)
            pixels.append((word >> 16) & 0xFF)
            pixels.append((word >> 24) & 0xFF)
    return pixels[:width * height]


# ---------------------------------------------------------------------------
# Union-Find with path compression and union-by-rank
# ---------------------------------------------------------------------------

class UnionFind:
    def __init__(self, max_size: int):
        self._parent = list(range(max_size))
        self._rank   = [0] * max_size

    def find(self, x: int) -> int:
        """Path-compressing find."""
        root = x
        while self._parent[root] != root:
            root = self._parent[root]
        # Path compression
        while self._parent[x] != root:
            self._parent[x], x = root, self._parent[x]
        return root

    def union(self, a: int, b: int) -> None:
        """Union-by-rank."""
        ra = self.find(a)
        rb = self.find(b)
        if ra == rb:
            return
        if self._rank[ra] < self._rank[rb]:
            ra, rb = rb, ra
        self._parent[rb] = ra
        if self._rank[ra] == self._rank[rb]:
            self._rank[ra] += 1


# ---------------------------------------------------------------------------
# Stage 1 — run extraction per row
# ---------------------------------------------------------------------------

def extract_runs(pixels: list, row: int, width: int, threshold: int) -> list:
    """
    Scan one row; return list of (xs, xe) tuples for contiguous foreground
    runs.  Foreground condition: pixel >= threshold  (matches grid model).
    """
    runs = []
    start = -1
    base = row * width
    for x in range(width):
        fg = pixels[base + x] >= threshold
        if fg and start < 0:
            start = x
        elif not fg and start >= 0:
            runs.append((start, x - 1))
            start = -1
    if start >= 0:
        runs.append((start, width - 1))
    return runs


# ---------------------------------------------------------------------------
# Stage 2 — row-by-row merge with union-find + accumulation
# ---------------------------------------------------------------------------

def _overlap_8conn(xs_c: int, xe_c: int, xs_p: int, xe_p: int) -> bool:
    """True when current run and previous run are 8-connected (touch or overlap)."""
    return xs_c <= xe_p + 1 and xs_p <= xe_c + 1


def process_frame(pixels: list, width: int, height: int, threshold: int) -> list:
    """
    Run RLE-based streaming CCL.

    Returns a list of MAX_BLOBS descriptor dicts (sparse — many will have
    count==0).  Each descriptor is keyed by its allocated blob_id.

    Descriptor fields:
      count, sum_x, sum_y, xmin, xmax, ymin, ymax
    """
    uf = UnionFind(MAX_BLOBS)
    next_id = 0

    # Descriptor table — one slot per allocated blob_id
    desc = [None] * MAX_BLOBS   # None = unallocated

    def alloc_blob():
        nonlocal next_id
        if next_id >= MAX_BLOBS:
            return -1            # overflow — drop
        bid = next_id
        next_id += 1
        desc[bid] = {
            'count': 0, 'sum_x': 0, 'sum_y': 0,
            'xmin': width,  'xmax': -1,
            'ymin': height, 'ymax': -1,
        }
        return bid

    def accumulate(bid: int, xs: int, xe: int, row: int) -> None:
        """Add a run's contribution into the descriptor of blob_id bid."""
        root = uf.find(bid)
        if desc[root] is None:
            # Root might have been merged into another slot; allocate if needed
            desc[root] = {
                'count': 0, 'sum_x': 0, 'sum_y': 0,
                'xmin': width,  'xmax': -1,
                'ymin': height, 'ymax': -1,
            }
        d = desc[root]
        run_len    = xe - xs + 1
        d['count'] += run_len
        # Arithmetic series sum: xs + (xs+1) + ... + xe  = (xs + xe) * run_len // 2
        d['sum_x'] += (xs + xe) * run_len // 2
        d['sum_y'] += row * run_len
        if xs < d['xmin']: d['xmin'] = xs
        if xe > d['xmax']: d['xmax'] = xe
        if row < d['ymin']: d['ymin'] = row
        if row > d['ymax']: d['ymax'] = row

    prev_runs = []   # list of (xs, xe, blob_id) for previous row

    for row in range(height):
        curr_runs_raw = extract_runs(pixels, row, width, threshold)

        # Assign blob_ids to current-row runs and handle merges
        curr_runs = []  # (xs, xe, blob_id)

        for (xs_c, xe_c) in curr_runs_raw:
            assigned_id = -1   # blob_id assigned to this current run

            for (xs_p, xe_p, bid_p) in prev_runs:
                if not _overlap_8conn(xs_c, xe_c, xs_p, xe_p):
                    continue

                root_p = uf.find(bid_p)

                if assigned_id < 0:
                    # First overlapping prev run — inherit its blob_id
                    assigned_id = root_p
                else:
                    # Additional overlapping prev run — union if different root
                    root_a = uf.find(assigned_id)
                    if root_a != root_p:
                        uf.union(root_a, root_p)
                        # After union, use the new root as our assigned id
                        assigned_id = uf.find(root_a)

            if assigned_id < 0:
                # No overlap — new blob
                assigned_id = alloc_blob()

            if assigned_id >= 0:
                accumulate(assigned_id, xs_c, xe_c, row)
                curr_runs.append((xs_c, xe_c, assigned_id))

        prev_runs = curr_runs

    return desc


# ---------------------------------------------------------------------------
# Stage 3 — flatten: merge all descriptors to their final root
# ---------------------------------------------------------------------------

def flatten_blobs(desc: list, uf: 'UnionFind') -> list:
    """
    Walk all allocated descriptors; merge each into its final root's descriptor.
    Return list of blob dicts (count > 0 only), sorted by (ymin, xmin).
    """
    # We need the final uf state, so re-derive it from the desc list passed in.
    # Actually we receive the uf object alongside desc from process_frame.
    # Build merged descriptors indexed by final root.
    merged = {}   # root_id -> merged dict

    for bid in range(len(desc)):
        if desc[bid] is None:
            continue
        if desc[bid]['count'] == 0:
            continue
        root = uf.find(bid)
        if root not in merged:
            merged[root] = {
                'count': 0, 'sum_x': 0, 'sum_y': 0,
                'xmin': desc[bid]['xmin'],  # will be overwritten properly below
                'xmax': desc[bid]['xmax'],
                'ymin': desc[bid]['ymin'],
                'ymax': desc[bid]['ymax'],
            }
            # Reset extremes to safe initial values before merging
            merged[root]['xmin'] = 2**31
            merged[root]['xmax'] = -1
            merged[root]['ymin'] = 2**31
            merged[root]['ymax'] = -1

        m = merged[root]
        d = desc[bid]
        m['count'] += d['count']
        m['sum_x'] += d['sum_x']
        m['sum_y'] += d['sum_y']
        if d['xmin'] < m['xmin']: m['xmin'] = d['xmin']
        if d['xmax'] > m['xmax']: m['xmax'] = d['xmax']
        if d['ymin'] < m['ymin']: m['ymin'] = d['ymin']
        if d['ymax'] > m['ymax']: m['ymax'] = d['ymax']

    blobs = [m for m in merged.values() if m['count'] > 0]
    blobs.sort(key=lambda b: (b['ymin'], b['xmin']))
    return blobs


# ---------------------------------------------------------------------------
# Top-level entry point
# ---------------------------------------------------------------------------

def run_model(hex_path: str, width: int, height: int, threshold: int) -> list:
    """
    Read frame from hex file, run RLE-based streaming CCL, return blob descriptors.

    Returns list of dicts sorted by (ymin, xmin), each with:
      count: int   (foreground pixel count)
      sum_x: int   (sum of x coordinates of foreground pixels)
      sum_y: int   (sum of y coordinates of foreground pixels)
      xmin, xmax, ymin, ymax: int  (pixel-exact bounding box)

    MAX_BLOBS = 64; blobs beyond that limit are silently dropped.
    All arithmetic is integer — no floats.
    """
    pixels = load_hex_frame(hex_path, width, height)

    # Build a fresh union-find so flatten_blobs can use it
    uf = UnionFind(MAX_BLOBS)
    next_id = 0

    # --- inline process_frame so we can hand uf to flatten_blobs ---
    desc = [None] * MAX_BLOBS

    def alloc_blob():
        nonlocal next_id
        if next_id >= MAX_BLOBS:
            return -1
        bid = next_id
        next_id += 1
        desc[bid] = {
            'count': 0, 'sum_x': 0, 'sum_y': 0,
            'xmin': width,  'xmax': -1,
            'ymin': height, 'ymax': -1,
        }
        return bid

    def accumulate(bid: int, xs: int, xe: int, row: int) -> None:
        root = uf.find(bid)
        if desc[root] is None:
            desc[root] = {
                'count': 0, 'sum_x': 0, 'sum_y': 0,
                'xmin': width,  'xmax': -1,
                'ymin': height, 'ymax': -1,
            }
        d = desc[root]
        run_len    = xe - xs + 1
        d['count'] += run_len
        d['sum_x'] += (xs + xe) * run_len // 2
        d['sum_y'] += row * run_len
        if xs < d['xmin']: d['xmin'] = xs
        if xe > d['xmax']: d['xmax'] = xe
        if row < d['ymin']: d['ymin'] = row
        if row > d['ymax']: d['ymax'] = row

    prev_runs = []

    for row in range(height):
        curr_runs_raw = extract_runs(pixels, row, width, threshold)
        curr_runs = []

        for (xs_c, xe_c) in curr_runs_raw:
            assigned_id = -1

            for (xs_p, xe_p, bid_p) in prev_runs:
                if not _overlap_8conn(xs_c, xe_c, xs_p, xe_p):
                    continue
                root_p = uf.find(bid_p)
                if assigned_id < 0:
                    assigned_id = root_p
                else:
                    root_a = uf.find(assigned_id)
                    if root_a != root_p:
                        uf.union(root_a, root_p)
                        assigned_id = uf.find(root_a)

            if assigned_id < 0:
                assigned_id = alloc_blob()

            if assigned_id >= 0:
                accumulate(assigned_id, xs_c, xe_c, row)
                curr_runs.append((xs_c, xe_c, assigned_id))

        prev_runs = curr_runs

    return flatten_blobs(desc, uf)


# ---------------------------------------------------------------------------
# Standalone CLI
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description='Blob detect RLE functional model')
    parser.add_argument('--input',     required=True, help='Input frame hex file')
    parser.add_argument('--width',     type=int, required=True)
    parser.add_argument('--height',    type=int, required=True)
    parser.add_argument('--threshold', type=int, default=128)
    parser.add_argument('--output',    default='blobs_rle_model.json',
                        help='Output JSON file (default: blobs_rle_model.json)')
    args = parser.parse_args()

    blobs = run_model(args.input, args.width, args.height, args.threshold)

    result = {'blob_count': len(blobs), 'blobs': blobs}
    with open(args.output, 'w') as f:
        json.dump(result, f, indent=2)

    print(f'Detected {len(blobs)} blob(s). Written to {args.output}')
    for i, b in enumerate(blobs):
        cx = b['sum_x'] / b['count'] if b['count'] else 0
        cy = b['sum_y'] / b['count'] if b['count'] else 0
        print(f'  blob[{i}]: count={b["count"]:6d}  centroid=({cx:.1f},{cy:.1f})'
              f'  bbox=({b["xmin"]},{b["ymin"]})-({b["xmax"]},{b["ymax"]})')


if __name__ == '__main__':
    main()
