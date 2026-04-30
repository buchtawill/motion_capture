#!/home/will/Desktop/motion_capture/mocap_env/bin/python

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


def get_frame_with_blobs(width: int, height: int, n_blobs: int = 8, blob_size: int = 5) -> np.ndarray:
    """
    Generates a frame with random blobs for testing purposes.

    Parameters:
    - width: The width of the frame.
    - height: The height of the frame.
    - n_blobs: The number of blobs to generate.
    - blob_size: The size of each blob (radius).

    Returns:
    - A 2D numpy array representing the frame with blobs.
    """
    frame = np.zeros((height, width), dtype=np.uint8)

    for _ in range(n_blobs):
        center_x = np.random.randint(blob_size, width - blob_size)
        center_y = np.random.randint(blob_size, height - blob_size)

        for x in range(center_x - blob_size, center_x + blob_size + 1):
            for y in range(center_y - blob_size, center_y + blob_size + 1):
                if (x - center_x) ** 2 + (y - center_y) ** 2 <= blob_size ** 2:
                    frame[y, x] = 255

    return frame


# ---------------------------------------------------------------------------
# Streaming blob detector: RLE + run-merge architecture
# ---------------------------------------------------------------------------
#
# Pipeline (each stage maps to one future SV module, communicating over AXIS):
#
#   pixel stream ──► [run_extractor] ──► run FIFO ──► [row_merger] ──► blob FIFO
#                       (Stage 1)         (xs,xe)      (Stage 2)        (Stage 3)
#                                                          ▲
#                                              prev-row run FIFO
#                                              (xs,xe,blob_id)  (loops back from
#                                                                row_merger output)
#
# Hardware-friendly constraints honoured below:
#   - The frame is read in raster order ONLY (one pixel at a time, no 2-D
#     indexing). This models the AXIS pixel stream input.
#   - All inter-stage queues are plain FIFOs (push at tail, pop at head).
#   - The "blob table" holding accumulator state for currently-live blobs is a
#     small register file / BRAM in HW. It is *not* a frame buffer; its size is
#     bounded by the maximum number of simultaneously-live blobs (~ width / 2
#     in the worst case for a comb pattern).
#   - Every per-run computation is integer add / min / max / shift.
#     `sum_x_in_run = (xs+xe)*(xe-xs+1) >> 1`  — exact, since (xs+xe) and
#     (xe-xs+1) always have opposite parity.
# ---------------------------------------------------------------------------


def _run_extractor(pixel_iter, width, threshold):
    """Stage 1: stream pixels for one row, push (xs, xe) runs to a FIFO."""
    run_fifo = []
    in_run = False
    run_start = 0
    for x in range(width):
        pix = next(pixel_iter)                      # one pixel per cycle
        high = pix >= threshold                     # 1-bit threshold compare
        if high and not in_run:
            run_start = x
            in_run = True
        elif (not high) and in_run:
            run_fifo.append((run_start, x - 1))     # FIFO push
            in_run = False
    if in_run:
        run_fifo.append((run_start, width - 1))     # row-end flush
    return run_fifo


def _new_blob(xs, xe, y):
    n = xe - xs + 1
    return {
        "count": n,
        "sx":    (xs + xe) * n // 2,                # Σ x  for x in [xs..xe]
        "sy":    n * y,
        "xmin":  xs, "xmax": xe,
        "ymin":  y,  "ymax": y,
    }


def _accumulate_run(blob, xs, xe, y):
    n = xe - xs + 1
    blob["count"] += n
    blob["sx"]    += (xs + xe) * n // 2
    blob["sy"]    += n * y
    if xs < blob["xmin"]: blob["xmin"] = xs
    if xe > blob["xmax"]: blob["xmax"] = xe
    blob["ymax"] = y                                # y monotonic; ymin already set


def _merge_into(dst, src):
    dst["count"] += src["count"]
    dst["sx"]    += src["sx"]
    dst["sy"]    += src["sy"]
    if src["xmin"] < dst["xmin"]: dst["xmin"] = src["xmin"]
    if src["xmax"] > dst["xmax"]: dst["xmax"] = src["xmax"]
    if src["ymin"] < dst["ymin"]: dst["ymin"] = src["ymin"]
    if src["ymax"] > dst["ymax"]: dst["ymax"] = src["ymax"]


def find_blobs_streaming(frame: np.ndarray, threshold: int = 128, connectivity: int = 8):
    """
    Detect blobs in a streaming raster-scan fashion.

    Parameters
    ----------
    frame        : 2-D uint8 array. Accessed in raster order only (models AXIS).
    threshold    : pixel >= threshold is foreground.
    connectivity : 4 or 8 (8 allows diagonal touches between rows).

    Returns
    -------
    List of blob descriptor dicts:
        {count, sx, sy, xmin, xmax, ymin, ymax, cx, cy}
    """
    height, width = frame.shape
    margin = 1 if connectivity == 8 else 0          # x-overlap slack between rows

    # AXIS pixel stream: a generator yielding pixels in raster order
    pixel_stream = (int(frame[y, x]) for y in range(height) for x in range(width))

    # Blob descriptor table (HW: small register file holding active blobs)
    blobs        = {}        # blob_id -> accumulator dict
    next_id      = 0
    completed    = []        # output FIFO

    # Inter-row state: previous-row run FIFO (each entry carries its blob id)
    prev_row_runs     = []   # list of (xs, xe, blob_id), x-sorted
    prev_row_blob_ids = set()

    for y in range(height):
        # ---- Stage 1 ---------------------------------------------------------
        run_fifo = _run_extractor(pixel_stream, width, threshold)

        # ---- Stage 2 ---------------------------------------------------------
        new_prev_runs     = []
        curr_row_blob_ids = set()
        prev_idx          = 0    # head pointer into prev_row_runs FIFO

        while run_fifo:
            xs, xe = run_fifo.pop(0)                # FIFO pop
            merged_id = None

            # Drop prev-row runs entirely to the left of the current run
            # (they cannot touch this run or any later run in this row).
            while (prev_idx < len(prev_row_runs)
                   and prev_row_runs[prev_idx][1] < xs - margin):
                prev_idx += 1

            # Peek through prev-row runs that overlap the current run.
            # A prev-row run extending past xe may also overlap the NEXT
            # current-row run, so it stays in the FIFO until it's fully passed.
            scan = prev_idx
            while (scan < len(prev_row_runs)
                   and prev_row_runs[scan][0] <= xe + margin):
                p_id = prev_row_runs[scan][2]
                if merged_id is None:
                    merged_id = p_id
                elif p_id != merged_id and p_id in blobs:
                    # Union: fold p_id's accumulator into merged_id and
                    # rewrite p_id's references in BOTH FIFOs to merged_id.
                    _merge_into(blobs[merged_id], blobs.pop(p_id))
                    for k in range(len(prev_row_runs)):
                        if prev_row_runs[k][2] == p_id:
                            prev_row_runs[k] = (prev_row_runs[k][0],
                                                prev_row_runs[k][1], merged_id)
                    for k in range(len(new_prev_runs)):
                        if new_prev_runs[k][2] == p_id:
                            new_prev_runs[k] = (new_prev_runs[k][0],
                                                new_prev_runs[k][1], merged_id)
                    if p_id in curr_row_blob_ids:
                        curr_row_blob_ids.discard(p_id)
                        curr_row_blob_ids.add(merged_id)
                scan += 1

            if merged_id is None:
                merged_id = next_id
                next_id  += 1
                blobs[merged_id] = _new_blob(xs, xe, y)
            else:
                _accumulate_run(blobs[merged_id], xs, xe, y)

            new_prev_runs.append((xs, xe, merged_id))
            curr_row_blob_ids.add(merged_id)

        # ---- Stage 3: emit blobs not touched this row -----------------------
        for done_id in (prev_row_blob_ids - curr_row_blob_ids):
            if done_id in blobs:                    # may have been merged away
                completed.append(blobs.pop(done_id))

        prev_row_runs     = new_prev_runs
        prev_row_blob_ids = curr_row_blob_ids

    # End-of-frame flush: anything still live is complete.
    for bid in prev_row_blob_ids:
        if bid in blobs:
            completed.append(blobs.pop(bid))

    for b in completed:
        b["cx"] = b["sx"] / b["count"]
        b["cy"] = b["sy"] / b["count"]
        bw = b["xmax"] - b["xmin"] + 1
        bh = b["ymax"] - b["ymin"] + 1
        b["bbox_w"] = bw
        b["bbox_h"] = bh
        # Derived metrics, computable from existing accumulators (no extra state):
        #   fill   ≈ π/4 ≈ 0.785  for an isolated solid disc
        #          drops sharply when two discs touch (bbox grows but pixel count doesn't)
        #   aspect = 1.0          for an isolated disc; >>1 for a chain of touching discs
        b["fill"]   = b["count"] / (bw * bh)
        b["aspect"] = bw / bh
    return completed


def _draw_disc(frame, cx, cy, r, val=255):
    h, w = frame.shape
    for y in range(max(0, cy - r), min(h, cy + r + 1)):
        for x in range(max(0, cx - r), min(w, cx + r + 1)):
            if (x - cx) ** 2 + (y - cy) ** 2 <= r * r:
                frame[y, x] = val


def test_overlapping_blobs():
    """
    Demonstrate detector behaviour on touching / overlapping discs.

    The threshold step erases brightness, so any 8-connected foreground region
    becomes a single component by definition — the detector cannot recover
    "this was actually two discs". What it CAN do, with no extra state, is
    flag a component as "probably merged" using metrics that fall straight
    out of the existing accumulators:

      fill   = count / (bbox_w * bbox_h)
               isolated disc → π/4 ≈ 0.785
               two touching  → ~0.50
               triangle of 3 → ~0.40
      aspect = bbox_w / bbox_h
               isolated disc → 1.0
               horiz pair    → ~2.0

    Both fill and aspect come from accumulators we already track (count, xmin,
    xmax, ymin, ymax). No extra HW cost beyond a divider per blob at emit time.
    """
    h, w = 220, 760
    r    = 15
    frame = np.zeros((h, w), dtype=np.uint8)

    groups = [
        ("single",         [(  60, 60)]),
        ("heavy overlap",  [( 160, 60), ( 185, 60)]),                      # dx = 25 < 2r
        ("tangent",        [( 280, 60), ( 310, 60)]),                      # dx = 30 = 2r
        ("narrow gap",     [( 400, 60), ( 435, 60)]),                      # dx = 35 > 2r
        ("clear gap",      [( 540, 60), ( 600, 60)]),                      # dx = 60
        ("triangle of 3",  [( 120,150), ( 155,150), ( 137,178)]),
        ("vertical pair",  [( 280,150), ( 280,180)]),
        ("L-shape (3)",    [( 420,150), ( 450,150), ( 450,180)]),
    ]
    for label, centers in groups:
        for (cx, cy) in centers:
            _draw_disc(frame, cx, cy, r)

    blobs = find_blobs_streaming(frame, threshold=128, connectivity=8)
    blobs.sort(key=lambda b: (b["ymin"], b["xmin"]))

    n_sources = sum(len(g[1]) for g in groups)
    print(f"\n--- overlap test: {n_sources} source discs ---")
    print(f"detected {len(blobs)} connected components "
          f"({n_sources - len(blobs)} merged away):\n")
    print(f"{'idx':>3} {'count':>5} {'bbox':>11} {'fill':>5} {'aspect':>6}  flag")
    for i, b in enumerate(blobs):
        # Heuristic: a component is "likely merged" if either metric is far
        # from a single-disc baseline. Tunable per application.
        merged = (b["fill"] < 0.65) or (b["aspect"] > 1.4) or (b["aspect"] < 1/1.4)
        flag   = "MERGED" if merged else "single"
        bbox   = f"{b['bbox_w']}x{b['bbox_h']}"
        print(f"{i:3d} {b['count']:5d} {bbox:>11} {b['fill']:5.2f} "
              f"{b['aspect']:6.2f}  {flag}")

    fig, ax = plt.subplots(figsize=(11, 4))
    ax.imshow(frame, cmap="gray")
    for b in blobs:
        merged = (b["fill"] < 0.65) or (b["aspect"] > 1.4) or (b["aspect"] < 1/1.4)
        color  = "red" if merged else "lime"
        ax.add_patch(Rectangle((b["xmin"] - 0.5, b["ymin"] - 0.5),
                               b["bbox_w"], b["bbox_h"],
                               fill=False, edgecolor=color, linewidth=1.2))
        ax.plot(b["cx"], b["cy"], "+", color=color, markersize=8)
        ax.text(b["xmin"], b["ymin"] - 4,
                f"f={b['fill']:.2f} a={b['aspect']:.2f}",
                color=color, fontsize=7)
    ax.set_title("overlap test — red = MERGED flag, green = single-disc-like")
    plt.tight_layout()
    plt.show()


# ---------------------------------------------------------------------------
# Demo / visual check
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    np.random.seed(0)
    width, height = 1280, 800
    frame = get_frame_with_blobs(width, height, n_blobs=12, blob_size=8)

    blobs = find_blobs_streaming(frame, threshold=128, connectivity=8)
    print(f"detected {len(blobs)} blobs")
    for i, b in enumerate(blobs):
        print(f"  [{i:2d}] count={b['count']:5d}  "
              f"centroid=({b['cx']:7.2f},{b['cy']:7.2f})  "
              f"bbox=[{b['xmin']:4d}..{b['xmax']:4d}, {b['ymin']:4d}..{b['ymax']:4d}]")

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.imshow(frame, cmap="gray")
    for b in blobs:
        w = b["xmax"] - b["xmin"] + 1
        h = b["ymax"] - b["ymin"] + 1
        ax.add_patch(Rectangle((b["xmin"] - 0.5, b["ymin"] - 0.5),
                               w, h, fill=False, edgecolor="lime", linewidth=1))
        ax.plot(b["cx"], b["cy"], "r+", markersize=8)
    ax.set_title(f"streaming RLE blob detector — {len(blobs)} blobs")
    plt.show()

    test_overlapping_blobs()
