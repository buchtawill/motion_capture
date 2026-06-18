"""
blob_detect_grid_model.py — Functional model for the blob_detect_grid IP.

Produces bit-exact integer results matching the RTL computation:
  1. Parse hex file → pixel array (LSB-first packing, 4 px per 32-bit word).
  2. Scan pixels; accumulate per-cell (count, sum_x, sum_y) for foreground pixels.
  3. 4-connected flood-fill on non-empty cell grid → blob groups.
  4. Aggregate per-blob statistics; sort by (ymin, xmin).

Usage (standalone):
  python blob_detect_grid_model.py \\
      --input   frame_0000.hex \\
      --width   1280 \\
      --height  800 \\
      --threshold 128 \\
      --cell-w  32 \\
      --cell-h  32 \\
      --output  blobs_model.json
"""

import argparse
import json
import math
import sys


# ---------------------------------------------------------------------------
# Hex file I/O
# ---------------------------------------------------------------------------

def load_hex_frame(hex_path: str, width: int, height: int) -> list[int]:
    """
    Parse a $readmemh-style hex file (one 32-bit word per line).
    Return flat list of pixel values, length == width * height.
    Extra padding pixels (if total pixels wasn't a multiple of 4) are discarded.
    """
    pixels: list[int] = []
    with open(hex_path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('//'):
                continue
            word = int(line, 16)
            # Unpack LSB-first: bits [7:0] → px0, [15:8] → px1, ...
            pixels.append( word        & 0xFF)
            pixels.append((word >>  8) & 0xFF)
            pixels.append((word >> 16) & 0xFF)
            pixels.append((word >> 24) & 0xFF)

    # Trim to exact frame size
    return pixels[:width * height]


# ---------------------------------------------------------------------------
# Stage 1 — per-cell accumulation
# ---------------------------------------------------------------------------

def accumulate_cells(pixels: list[int], width: int, height: int,
                     threshold: int, cell_w: int, cell_h: int
                     ) -> dict[tuple[int, int], dict]:
    """
    Walk pixels in raster order.  For every foreground pixel (value >= threshold)
    update its cell's (count, sum_x, sum_y).

    Returns dict keyed by (cell_row, cell_col) with values
      {'count': int, 'sum_x': int, 'sum_y': int}
    Only cells with at least one foreground pixel appear in the dict.
    """
    cells: dict[tuple[int, int], dict] = {}

    for idx in range(len(pixels)):
        pval = pixels[idx]
        if pval < threshold:
            continue
        x = idx % width
        y = idx // width
        cell_col = x // cell_w
        cell_row = y // cell_h
        key = (cell_row, cell_col)
        if key not in cells:
            cells[key] = {'count': 0, 'sum_x': 0, 'sum_y': 0}
        cells[key]['count'] += 1
        cells[key]['sum_x'] += x
        cells[key]['sum_y'] += y

    return cells


# ---------------------------------------------------------------------------
# Stage 2 — 4-connected flood-fill on cell grid
# ---------------------------------------------------------------------------

def flood_fill_cells(cells: dict[tuple[int, int], dict],
                     num_cell_rows: int, num_cell_cols: int
                     ) -> list[list[tuple[int, int]]]:
    """
    Group non-empty cells into 4-connected blobs.
    Returns list of groups; each group is a list of (cell_row, cell_col) tuples.
    """
    visited: set[tuple[int, int]] = set()
    groups: list[list[tuple[int, int]]] = []

    # Use a list-based stack (LIFO) — matches iterative DFS the RTL will do.
    for start in cells:
        if start in visited:
            continue
        # New blob
        group: list[tuple[int, int]] = []
        stack: list[tuple[int, int]] = [start]
        while stack:
            cr, cc = stack.pop()
            if (cr, cc) in visited:
                continue
            visited.add((cr, cc))
            group.append((cr, cc))
            # 4-connected neighbours
            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                nr, nc = cr + dr, cc + dc
                if (nr, nc) in cells and (nr, nc) not in visited:
                    stack.append((nr, nc))
        groups.append(group)

    return groups


# ---------------------------------------------------------------------------
# Stage 3 — aggregate blob statistics
# ---------------------------------------------------------------------------

def aggregate_blobs(groups: list[list[tuple[int, int]]],
                    cells: dict[tuple[int, int], dict],
                    width: int, height: int,
                    cell_w: int, cell_h: int
                    ) -> list[dict]:
    """
    For each group, sum count/sum_x/sum_y across member cells and compute
    cell-granularity bounding box.

    Bounding box definition (matches hardware):
      xmin = cell_col * cell_w
      xmax = min((cell_col + 1) * cell_w - 1, width  - 1)
      ymin = cell_row * cell_h
      ymax = min((cell_row + 1) * cell_h - 1, height - 1)
    The blob bbox is the union of its member cell bboxes.
    """
    blobs: list[dict] = []

    for group in groups:
        count = 0
        sum_x = 0
        sum_y = 0
        xmin = width  - 1
        xmax = 0
        ymin = height - 1
        ymax = 0

        for (cr, cc) in group:
            c = cells[(cr, cc)]
            count += c['count']
            sum_x += c['sum_x']
            sum_y += c['sum_y']

            cell_xmin = cc * cell_w
            cell_xmax = min((cc + 1) * cell_w - 1, width  - 1)
            cell_ymin = cr * cell_h
            cell_ymax = min((cr + 1) * cell_h - 1, height - 1)

            if cell_xmin < xmin: xmin = cell_xmin
            if cell_xmax > xmax: xmax = cell_xmax
            if cell_ymin < ymin: ymin = cell_ymin
            if cell_ymax > ymax: ymax = cell_ymax

        blobs.append({
            'count': count,
            'sum_x': sum_x,
            'sum_y': sum_y,
            'xmin':  xmin,
            'xmax':  xmax,
            'ymin':  ymin,
            'ymax':  ymax,
        })

    # Sort by (ymin, xmin) — deterministic ordering matching RTL scan order
    blobs.sort(key=lambda b: (b['ymin'], b['xmin']))
    return blobs


# ---------------------------------------------------------------------------
# Top-level entry point
# ---------------------------------------------------------------------------

def run_model(frame_hex_path: str, width: int, height: int, threshold: int,
              cell_w: int = 32, cell_h: int = 32) -> list[dict]:
    """
    Read frame from hex file, run grid-based blob detection, return blob descriptors.

    Returns list of dicts, each with:
      count: int   (foreground pixel count)
      sum_x: int   (sum of x coordinates)
      sum_y: int   (sum of y coordinates)
      xmin, xmax, ymin, ymax: int  (bounding box, cell-granularity)

    Blobs sorted by (ymin, xmin).
    """
    num_cell_rows = math.ceil(height / cell_h)
    num_cell_cols = math.ceil(width  / cell_w)

    pixels = load_hex_frame(frame_hex_path, width, height)
    cells  = accumulate_cells(pixels, width, height, threshold, cell_w, cell_h)
    groups = flood_fill_cells(cells, num_cell_rows, num_cell_cols)
    blobs  = aggregate_blobs(groups, cells, width, height, cell_w, cell_h)
    return blobs


# ---------------------------------------------------------------------------
# Standalone CLI
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description='Blob detect grid functional model')
    parser.add_argument('--input',     required=True, help='Input frame hex file')
    parser.add_argument('--width',     type=int, required=True)
    parser.add_argument('--height',    type=int, required=True)
    parser.add_argument('--threshold', type=int, default=128)
    parser.add_argument('--cell-w',    type=int, default=32)
    parser.add_argument('--cell-h',    type=int, default=32)
    parser.add_argument('--output',    default='blobs_model.json',
                        help='Output JSON file (default: blobs_model.json)')
    args = parser.parse_args()

    blobs = run_model(args.input, args.width, args.height, args.threshold,
                      args.cell_w, args.cell_h)

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
