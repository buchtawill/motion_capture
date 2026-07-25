#!/usr/bin/env python3
"""
Run the RLE blob detection model on all test frames.

Reads frame_NNNN_meta.json for width/height/threshold, runs the model,
writes blobs_model_NNNN.json.

Usage:
  python run_models.py --dir sim_out/tb_blob_detect_rle/
"""

import argparse
import glob
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import blob_detect_rle_model as model


def main():
    parser = argparse.ArgumentParser(description="Run RLE model on all test frames")
    parser.add_argument("--dir", required=True, help="Directory with frame hex + meta files")
    args = parser.parse_args()

    metas = sorted(glob.glob(os.path.join(args.dir, "frame_*_meta.json")))
    if not metas:
        print(f"No frame_*_meta.json files in {args.dir}")
        sys.exit(1)

    for meta_path in metas:
        idx = os.path.basename(meta_path).split("_")[1]
        hex_path = os.path.join(args.dir, f"frame_{idx}.hex")

        with open(meta_path) as f:
            meta = json.load(f)

        if not os.path.exists(hex_path):
            print(f"  [{idx}] SKIP — {hex_path} not found")
            continue

        blobs = model.run_model(
            hex_path, meta["width"], meta["height"], meta["threshold"])

        out_path = os.path.join(args.dir, f"blobs_model_{idx}.json")
        with open(out_path, "w") as f:
            json.dump({"blob_count": len(blobs), "blobs": blobs}, f, indent=2)

        print(f"  [{idx}] {meta.get('description', '')} — {len(blobs)} blobs -> {out_path}")


if __name__ == "__main__":
    main()
