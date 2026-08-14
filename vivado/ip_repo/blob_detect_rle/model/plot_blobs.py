#!/usr/bin/env python3
"""plot_blobs.py -- run the functional RLE blob-detect model on a raw 8-bit
grayscale frame and show an interactive matplotlib figure of what the detector
sees, optionally after a spatial pre-filter.

Columns (top row = plain, bottom row = with blob bounding boxes):
  * no --filter :  original | thresholded(>= T)
  * with --filter:  original | filtered (<- MODEL INPUT) | thresholded(>= T)

The blob boxes come from blob_detect_rle_model.run_model() run on the MODEL
INPUT -- the filtered image when a filter is active, else the raw frame -- fed
via a temp hex file in the model's exact LSB-first 4-px/word format, so the
result is bit-identical to the RTL cosim path. Foreground is `pixel >= T`; the
thresholded panel is the threshold applied to the MODEL INPUT, i.e. the exact
foreground mask the detector labels.

The spatial pre-filter prototypes a hardware denoise stage (a streaming 3x3
line-buffer window before run_extractor, in the blob snoop path only) that
rejects single-pixel threshold noise. 'median' preserves >=2 px markers best;
'box'/'gaussian' are linear low-pass and attenuate small bright features too.

Usage:
    python plot_blobs.py FRAME.gray [--threshold 200]
        [--filter none|box|median|gaussian] [--ksize 3] [--sigma 1.0]
        [--width W] [--height H] [--save out.png] [--no-show]

W/H default to the WxH parsed from a screen_<n>_<W>x<H>.gray style filename.
Example:
    python plot_blobs.py ../../screen_0000_1280x720.gray -t 200 --filter box -k 3
"""
import argparse
import os
import re
import sys
import tempfile

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from scipy.ndimage import uniform_filter, median_filter, gaussian_filter

# Import the functional model that lives next to this script.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import blob_detect_rle_model as model  # noqa: E402


def parse_dims_from_name(path):
    """Pull WxH out of a filename like screen_0000_1280x720.gray."""
    m = re.search(r"(\d+)x(\d+)", os.path.basename(path))
    return (int(m.group(1)), int(m.group(2))) if m else (None, None)


def pack_hex(flat_pixels, hex_path):
    """Write pixels one 32-bit word per line, LSB-first 4-px packing -- matches
    blob_detect_rle_model.load_hex_frame exactly."""
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
        pack_hex(np.ascontiguousarray(img).reshape(-1), tmp)
        return model.run_model(tmp, W, H, threshold)
    finally:
        os.unlink(tmp)


def apply_filter(img, kind, ksize, sigma):
    """Spatial pre-filter; returns a uint8 image in the same 0..255 range.
    'reflect' borders mirror what a HW line-buffer edge-replicate would do.
    box/median use a ksize x ksize window; gaussian uses sigma.

    NOTE: box here is a rounded mean (avg >= T). The RTL equivalent compares the
    window SUM against ksize*ksize*T (no divide); the two differ only by <=1 LSB
    rounding at the threshold boundary."""
    if kind == "none":
        return img
    if kind == "box":
        return uniform_filter(img, size=ksize, mode="reflect")
    if kind == "median":
        return median_filter(img, size=ksize, mode="reflect")
    if kind == "gaussian":
        return gaussian_filter(img, sigma=sigma, mode="reflect")
    raise ValueError(f"unknown filter '{kind}'")


def draw_boxes(ax, blobs):
    for b in blobs:
        # Boxes are pixel-inclusive [xmin..xmax]; -0.5 hugs the outer pixel edge.
        ax.add_patch(patches.Rectangle(
            (b["xmin"] - 0.5, b["ymin"] - 0.5),
            b["xmax"] - b["xmin"] + 1,
            b["ymax"] - b["ymin"] + 1,
            fill=False, edgecolor="red", linewidth=1.2))


def print_blobs(tag, blobs):
    print(f"{tag}: {len(blobs)} blob(s)")
    for i, b in enumerate(blobs):
        cx = b["sum_x"] / b["count"] if b["count"] else 0
        cy = b["sum_y"] / b["count"] if b["count"] else 0
        print(f"  #{i:<3d} bbox x[{b['xmin']:4d}..{b['xmax']:4d}] "
              f"y[{b['ymin']:4d}..{b['ymax']:4d}]  count={b['count']:6d}  "
              f"centroid=({cx:.1f},{cy:.1f})")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("frame", help="raw 8-bit grayscale frame (.gray)")
    ap.add_argument("--threshold", "-t", type=int, default=200,
                    help="foreground threshold (pixel >= T); default 200")
    ap.add_argument("--filter", "-F", default="none",
                    choices=["none", "box", "median", "gaussian"],
                    help="spatial pre-filter applied before the detector")
    ap.add_argument("--ksize", "-k", type=int, default=3,
                    help="kernel size for box/median (odd; default 3)")
    ap.add_argument("--sigma", type=float, default=1.0,
                    help="sigma for the gaussian filter (default 1.0)")
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

    filter_on = args.filter != "none"
    model_input = apply_filter(img, args.filter, args.ksize, args.sigma)

    if not filter_on:
        fdesc = "none"
    elif args.filter in ("box", "median"):
        fdesc = f"{args.filter} k={args.ksize}"
    else:
        fdesc = f"gaussian sigma={args.sigma}"

    # Detector output on the RAW frame (the "no filter" baseline, left column)
    # and on the MODEL INPUT (filtered; == raw when no filter, so reuse then).
    raw_blobs = run_functional_model(img, args.threshold)
    blobs = raw_blobs if not filter_on else run_functional_model(model_input,
                                                                 args.threshold)

    print(f"{os.path.basename(args.frame)}  {W}x{H}  threshold={args.threshold}  "
          f"filter={fdesc}")
    if filter_on:
        print(f"  unfiltered -> {len(raw_blobs)} blobs   "
              f"filtered -> {len(blobs)} blobs   "
              f"({len(blobs) - len(raw_blobs):+d})")
    print_blobs("model input", blobs)

    # Threshold of the MODEL INPUT == the exact foreground mask the detector labels.
    binary = np.where(model_input >= args.threshold, 255, 0).astype(np.uint8)

    # (title, image, blobs-detected-on-this-panel's-input). The raw column shows
    # the unfiltered detection; the filtered/thresholded columns show the
    # post-filter detection -- so each title reports that method's blob count.
    columns = [("original (raw)", img, raw_blobs)]
    if filter_on:
        columns.append((f"filtered [{fdesc}]  <- MODEL INPUT", model_input, blobs))
    columns.append((f"thresholded (>= {args.threshold})", binary, blobs))

    ncols = len(columns)
    fig, axs = plt.subplots(2, ncols, figsize=(6.2 * ncols, 8),
                            constrained_layout=True, squeeze=False)
    fig.suptitle(f"{os.path.basename(args.frame)}   {W}x{H}   "
                 f"threshold={args.threshold}   filter={fdesc}   "
                 f"blobs={len(blobs)}")

    for c, (title, image, cblobs) in enumerate(columns):
        base = f"{title}   [{len(cblobs)} blobs]"
        axs[0, c].imshow(image, cmap="gray", vmin=0, vmax=255)
        axs[0, c].set_title(base)
        axs[1, c].imshow(image, cmap="gray", vmin=0, vmax=255)
        axs[1, c].set_title(base + "  + boxes")
        draw_boxes(axs[1, c], cblobs)

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
