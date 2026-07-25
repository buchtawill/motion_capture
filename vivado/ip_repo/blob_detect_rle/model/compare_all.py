#!/usr/bin/env python3
"""
Compare RTL output against model output for all test frames.

Looks for pairs of blobs_rtl_NNNN.hex and blobs_model_NNNN.json.
Skips frames where the all-white frame (0004) is excluded.

Usage:
  python compare_all.py --dir sim_out/tb_blob_detect_rle/
"""

import argparse
import glob
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from compare import load_rtl, load_model, compare


def main():
    parser = argparse.ArgumentParser(description="Compare all RTL vs model outputs")
    parser.add_argument("--dir", required=True, help="Directory with RTL hex + model JSON files")
    args = parser.parse_args()

    rtl_files = sorted(glob.glob(os.path.join(args.dir, "blobs_rtl_*.hex")))
    if not rtl_files:
        print(f"No blobs_rtl_*.hex files in {args.dir}")
        sys.exit(1)

    total = 0
    passed = 0

    for rtl_path in rtl_files:
        idx = os.path.basename(rtl_path).replace("blobs_rtl_", "").replace(".hex", "")
        model_path = os.path.join(args.dir, f"blobs_model_{idx}.json")

        if not os.path.exists(model_path):
            print(f"\n--- [{idx}] SKIP — {model_path} not found ---")
            continue

        total += 1
        print(f"\n--- [{idx}] Comparing ---")

        rtl_blobs = load_rtl(rtl_path)
        model_blobs = load_model(model_path)

        if compare(rtl_blobs, model_blobs):
            passed += 1

    print(f"\n{'='*46}")
    print(f"  Compare: {passed}/{total} passed")
    print(f"{'='*46}")
    sys.exit(0 if passed == total else 1)


if __name__ == "__main__":
    main()
