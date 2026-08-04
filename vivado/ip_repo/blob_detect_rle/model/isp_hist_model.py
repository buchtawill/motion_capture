"""
isp_hist_model.py — Functional model for the isp_histogram engine as used by
mocap_wrapper.

Given a frame hex file (same $readmemh 32-bit-word, LSB-first pixel packing
used by blob_detect_rle_model) and its W/H, computes:
  hist[256]  — bin counts (np.bincount over all pixels)
  pixel_sum  — sum of all pixel values (int, unbounded / Python int)

Writes JSON {"hist": [256 ints], "pixel_sum": int}.

Usage (standalone, single frame):
  python isp_hist_model.py --input frame_0000.hex --width 640 --height 400 \
      --output hist_model_0000.json

Usage (batch, mirrors run_models.py for blobs):
  python isp_hist_model.py --dir sim_out/tb_mocap_wrapper/
    walks frame_*_meta.json in --dir, writes hist_model_<idx>.json next to it.
"""

import argparse
import glob
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from blob_detect_rle_model import load_hex_frame


def run_hist_model(hex_path: str, width: int, height: int) -> dict:
    """
    Read frame from hex file, compute the 256-bin histogram and pixel sum.

    Returns {"hist": [256 ints], "pixel_sum": int}.
    """
    pixels = load_hex_frame(hex_path, width, height)
    arr = np.array(pixels, dtype=np.uint64) & 0xFF
    hist = np.bincount(arr, minlength=256)[:256]
    pixel_sum = int(arr.sum())
    return {"hist": [int(x) for x in hist], "pixel_sum": pixel_sum}


def run_models(directory: str) -> None:
    """
    Batch entry point: for every frame_*_meta.json in `directory`, run the
    histogram model on the matching frame_*.hex and write
    hist_model_<idx>.json alongside it. Mirrors run_models.py's blob-model
    batch driver.
    """
    metas = sorted(glob.glob(os.path.join(directory, "frame_*_meta.json")))
    if not metas:
        print(f"No frame_*_meta.json files in {directory}")
        sys.exit(1)

    for meta_path in metas:
        idx = os.path.basename(meta_path).split("_")[1]
        hex_path = os.path.join(directory, f"frame_{idx}.hex")

        with open(meta_path) as f:
            meta = json.load(f)

        if not os.path.exists(hex_path):
            print(f"  [{idx}] SKIP — {hex_path} not found")
            continue

        result = run_hist_model(hex_path, meta["width"], meta["height"])

        out_path = os.path.join(directory, f"hist_model_{idx}.json")
        with open(out_path, "w") as f:
            json.dump(result, f, indent=2)

        print(f"  [{idx}] {meta.get('description', '')} — "
              f"pixel_sum={result['pixel_sum']} -> {out_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description='ISP histogram functional model')
    parser.add_argument('--input',  help='Input frame hex file (single-frame mode)')
    parser.add_argument('--width',  type=int, help='Frame width (single-frame mode)')
    parser.add_argument('--height', type=int, help='Frame height (single-frame mode)')
    parser.add_argument('--output', default='hist_model.json',
                         help='Output JSON file (single-frame mode)')
    parser.add_argument('--dir', help='Batch mode: directory with frame_*_meta.json files')
    args = parser.parse_args()

    if args.dir:
        run_models(args.dir)
        return

    if not (args.input and args.width and args.height):
        parser.error('single-frame mode requires --input, --width, --height')

    result = run_hist_model(args.input, args.width, args.height)
    with open(args.output, 'w') as f:
        json.dump(result, f, indent=2)
    print(f"pixel_sum={result['pixel_sum']}. Written to {args.output}")


if __name__ == '__main__':
    main()
