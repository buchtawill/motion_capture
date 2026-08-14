#!/usr/bin/env python3
"""gen_and_expect.py -- generate GFI test frames + bit-exact expected output.

Outputs into --output-dir:
  frame_NNNN.hex               input frame, 32-bit words, LSB-first 4px/word
                                (same convention as blob_detect_rle's loader)
  gfi_exp_NNNN_sTAG.hex        expected filtered output for that frame under
                                config TAG in {bypass, s0, s1, s2}, produced
                                by gfi_model.gfi_filter (the bit-exact golden)

Frame set (widths are all multiples of 4 per the AXIS packing requirement):
  0000  64x48  flat (uniform mid-grey)
  0001  64x48  isolated single-pixel spikes at/near borders + all 4 corners
  0002  64x48  solid bright block straddling a border
  0003  64x48  two thin features separated by a 2-pixel gap
  0004  64x48  random noise
  0005  1280x8 strip of random noise (realistic row width, few rows)

The frame list (index, width, height, description) mirrored here is also
hardcoded into tb_gfi.sv -- keep the two in sync if you add/remove cases.
"""
import argparse
import os

import numpy as np

import gfi_model as model

STRENGTH_TAGS = {"bypass": None, "s0": 0, "s1": 1, "s2": 2}


def frame_to_hex_words(frame: np.ndarray) -> list:
    flat = frame.flatten().tolist()
    rem = len(flat) % 4
    if rem:
        flat.extend([0] * (4 - rem))
    words = []
    for i in range(0, len(flat), 4):
        w = (flat[i] & 0xFF) | ((flat[i + 1] & 0xFF) << 8) | \
            ((flat[i + 2] & 0xFF) << 16) | ((flat[i + 3] & 0xFF) << 24)
        words.append(w)
    return words


def write_hex(path: str, words: list) -> None:
    with open(path, "w") as f:
        for w in words:
            f.write(f"{w:08x}\n")


def make_frames():
    frames = {}

    # 0000 - flat
    frames[0] = np.full((48, 64), 120, dtype=np.uint8)

    # 0001 - isolated spikes at/near borders + corners
    f = np.full((48, 64), 60, dtype=np.uint8)
    spikes = [
        (0, 0), (0, 63), (47, 0), (47, 63),       # 4 corners
        (0, 32), (47, 32), (24, 0), (24, 63),     # mid-edges
        (1, 1), (1, 62), (46, 1), (46, 62),       # near-corners
        (10, 10), (20, 40),                        # interior
    ]
    for (r, c) in spikes:
        f[r, c] = 255
    frames[1] = f

    # 0002 - solid bright block straddling the top-left border
    f = np.full((48, 64), 30, dtype=np.uint8)
    f[0:10, 0:12] = 250
    f[38:48, 52:64] = 250
    frames[2] = f

    # 0003 - two thin vertical bars separated by a 2px gap, plus a
    #        horizontal pair with a 2px gap
    f = np.full((48, 64), 40, dtype=np.uint8)
    f[10:30, 20] = 220
    f[10:30, 23] = 220   # gap of 2 columns (21,22) between the bars
    f[5, 5:15] = 200
    f[8, 5:15] = 200      # gap of 2 rows (6,7) between the bars
    frames[3] = f

    # 0004 - random noise
    rng = np.random.default_rng(1234)
    frames[4] = rng.integers(0, 256, size=(48, 64), dtype=np.uint16).astype(np.uint8)

    # 0005 - realistic row width, few rows, random noise
    rng2 = np.random.default_rng(5678)
    frames[5] = rng2.integers(0, 256, size=(8, 1280), dtype=np.uint16).astype(np.uint8)

    return frames


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--output-dir", required=True)
    args = ap.parse_args()
    os.makedirs(args.output_dir, exist_ok=True)

    frames = make_frames()

    for idx, frame in sorted(frames.items()):
        h, w = frame.shape
        hex_path = os.path.join(args.output_dir, f"frame_{idx:04d}.hex")
        write_hex(hex_path, frame_to_hex_words(frame))
        print(f"  [{idx:04d}] {w}x{h} -> {hex_path}")

        for tag, strength in STRENGTH_TAGS.items():
            if tag == "bypass":
                out = model.gfi_filter(frame, strength=0, enable=False)
            else:
                out = model.gfi_filter(frame, strength=strength, enable=True)
            exp_path = os.path.join(args.output_dir, f"gfi_exp_{idx:04d}_{tag}.hex")
            write_hex(exp_path, frame_to_hex_words(out))
            print(f"           {tag:7s} -> {exp_path}")

    print("Done.")


if __name__ == "__main__":
    main()
