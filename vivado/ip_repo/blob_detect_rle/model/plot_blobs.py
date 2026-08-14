#!/usr/bin/env python3
"""plot_blobs.py -- run the functional RLE blob-detect model on a raw 8-bit
grayscale frame and show an interactive 2x2 matplotlib figure:

    top-left     original frame
    bottom-left  original + blob bounding boxes
    top-right    thresholded  (pixel >= T -> 255, else 0)
    bottom-right thresholded + blob bounding boxes

The blob boxes come from blob_detect_rle_model.run_model(), fed via a temporary
hex file packed in the model's exact LSB-first 4-px/word format, so the result
is bit-identical to the RTL cosim path. Foreground is `pixel >= threshold`
(same test the model/RTL use), so the thresholded panels show exactly what the
detector sees.

Usage:
    python plot_blobs.py FRAME.gray [--threshold 200] [--width W] [--height H]
                         [--save out.png] [--no-show]

W/H default to the WxH parsed from a `screen_<n>_<W>x<H>.gray` style filename.
Example:
    python plot_blobs.py ../../screen_0000_1280x720.gray --threshold 200
"""
import argparse
import os
import re
import sys
import tempfile

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# Import the functional model that lives next to this script.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import blob_detect_rle_model as model  # noqa: E402


def parse_dims_from_name(path):
    """Pull WxH out of a filename like screen_0000_1280x720.gray."""
    m = re.search(r"(\d+)x(\d+)", os.path.basename(path))
    return (int(m.group(1)), int(m.group(2))) if m else (None, None)


def pack_hex(flat_pixels, hex_path):
    """Write pixels as one 32-bit word per line, LSB-first 4-px packing --
    matches blob_detect_rle_model.load_hex_frame exactly."""
    with open(hex_path, "w") as f:
        for i in range(0, len(flat_pixels), 4):
            w = (int(flat_pixels[i])
                 | (int(flat_pixels[i + 1]) << 8)
                 | (int(flat_pixels[i + 2]) << 16)
                 | (int(flat_pixels[i + 3]) << 24))
            f.write(f"{w:08x}\n")


def run_functional_model(img, threshold):
    """img: HxW uint8. Returns the model's blob list (via a temp hex file)."""
    H, W = img.shape
    fd, tmp = tempfile.mkstemp(suffix=".hex")
    os.close(fd)
    try:
        pack_hex(img.reshape(-1), tmp)
        return model.run_model(tmp, W, H, threshold)
    finally:
        os.unlink(tmp)


def draw_boxes(ax, blobs):
    for b in blobs:
        # Boxes are pixel-inclusive [xmin..xmax]; offset -0.5 so the rectangle
        # hugs the outer edge of those pixels (imshow centers pixels on ints).
        ax.add_patch(patches.Rectangle(
            (b["xmin"] - 0.5, b["ymin"] - 0.5),
            b["xmax"] - b["xmin"] + 1,
            b["ymax"] - b["ymin"] + 1,
            fill=False, edgecolor="red", linewidth=1.2))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("frame", help="raw 8-bit grayscale frame (.gray)")
    ap.add_argument("--threshold", "-t", type=int, default=200,
                    help="foreground threshold (pixel >= T); default 200")
    ap.add_argument("--width", "-W", type=int, default=None,
                    help="frame width (default: parsed from filename)")
    ap.add_argument("--height", "-H", type=int, default=None,
                    help="frame height (default: parsed from filename)")
    ap.add_argument("--save", default=None, metavar="PNG",
                    help="also save the figure to this file")
    ap.add_argument("--no-show", action="store_true",
                    help="do not open the interactive window (use with --save)")
    args = ap.parse_args()

    dw, dh = parse_dims_from_name(args.frame)
    W = args.width or dw
    H = args.height or dh
    if not W or not H:
        sys.exit("error: width/height not given and not parseable from filename")
    if W % 4 != 0:
        sys.exit("error: width must be a multiple of 4 (model packs 4 px/word)")

    raw = np.fromfile(args.frame, dtype=np.uint8)
    if raw.size < W * H:
        sys.exit(f"error: {args.frame} has {raw.size} bytes, need {W*H} for {W}x{H}")
    img = raw[:W * H].reshape(H, W)

    blobs = run_functional_model(img, args.threshold)

    print(f"{os.path.basename(args.frame)}  {W}x{H}  threshold={args.threshold}: "
          f"{len(blobs)} blob(s)")
    for i, b in enumerate(blobs):
        cx = b["sum_x"] / b["count"] if b["count"] else 0
        cy = b["sum_y"] / b["count"] if b["count"] else 0
        print(f"  #{i:<3d} bbox x[{b['xmin']:4d}..{b['xmax']:4d}] "
              f"y[{b['ymin']:4d}..{b['ymax']:4d}]  count={b['count']:6d}  "
              f"centroid=({cx:.1f},{cy:.1f})")

    extreme = np.where(img >= args.threshold, 255, 0).astype(np.uint8)

    fig, axs = plt.subplots(2, 2, figsize=(13, 8), constrained_layout=True)
    fig.suptitle(f"{os.path.basename(args.frame)}   {W}x{H}   "
                 f"threshold={args.threshold}   blobs={len(blobs)}")

    axs[0, 0].imshow(img, cmap="gray", vmin=0, vmax=255)
    axs[0, 0].set_title("original")

    axs[1, 0].imshow(img, cmap="gray", vmin=0, vmax=255)
    axs[1, 0].set_title("original + bounding boxes")
    draw_boxes(axs[1, 0], blobs)

    axs[0, 1].imshow(extreme, cmap="gray", vmin=0, vmax=255)
    axs[0, 1].set_title(f"thresholded  (>= {args.threshold} -> 255)")

    axs[1, 1].imshow(extreme, cmap="gray", vmin=0, vmax=255)
    axs[1, 1].set_title("thresholded + bounding boxes")
    draw_boxes(axs[1, 1], blobs)

    for ax in axs.flat:
        ax.set_xticks([])
        ax.set_yticks([])

    if args.save:
        fig.savefig(args.save, dpi=110)
        print(f"saved figure -> {args.save}")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
