"""
gen_stimulus.py — Generate test frames for blob_detect_grid IP.

Outputs:
  frame_NNNN.hex       32-bit words, one per line (%08x), pixels packed LSB-first.
                       Padded to a multiple of 4 pixels with zeros.
  frame_NNNN_meta.json {width, height, threshold, seed, num_blobs, blob_centers, blob_radii}

Usage:
  python gen_stimulus.py --width 1280 --height 800 --num-blobs 8 --seed 42 \
                         --threshold 128 --output-dir ./test_data/
"""

import argparse
import json
import os
import struct

import numpy as np


def make_circular_blob_frame(width: int, height: int, num_blobs: int, seed: int,
                              r_min: int = 5, r_max: int = 15,
                              foreground: int = 255) -> tuple[np.ndarray, list, list]:
    """Return (frame_uint8, blob_centers, blob_radii). Frame starts all-black."""
    rng = np.random.default_rng(seed)
    frame = np.zeros((height, width), dtype=np.uint8)
    centers = []
    radii = []

    for _ in range(num_blobs):
        r = int(rng.integers(r_min, r_max + 1))
        cx = int(rng.integers(r, width - r))
        cy = int(rng.integers(r, height - r))
        # Rasterize circle — set pixels within radius
        ys, xs = np.ogrid[:height, :width]
        mask = (xs - cx) ** 2 + (ys - cy) ** 2 <= r * r
        frame[mask] = foreground
        centers.append([cx, cy])
        radii.append(r)

    return frame, centers, radii


def frame_to_hex_words(frame: np.ndarray) -> list[int]:
    """
    Flatten frame in raster order, pack 4 pixels per 32-bit word (LSB-first),
    pad last word with zeros if needed.
    """
    flat = frame.flatten().tolist()
    # Pad to multiple of 4
    rem = len(flat) % 4
    if rem:
        flat.extend([0] * (4 - rem))
    words = []
    for i in range(0, len(flat), 4):
        w = (flat[i] & 0xFF) | ((flat[i+1] & 0xFF) << 8) | \
            ((flat[i+2] & 0xFF) << 16) | ((flat[i+3] & 0xFF) << 24)
        words.append(w)
    return words


def write_hex(path: str, words: list[int]) -> None:
    with open(path, 'w') as f:
        for w in words:
            f.write(f'{w:08x}\n')


def write_meta(path: str, meta: dict) -> None:
    with open(path, 'w') as f:
        json.dump(meta, f, indent=2)


def generate_test_cases(output_dir: str, threshold: int = 128) -> None:
    os.makedirs(output_dir, exist_ok=True)

    cases = [
        # (idx, width, height, frame_fn_or_mode, seed, num_blobs, description)
        (0, 1280,  800, 'blobs',   42,  8,  '1280x800, 8 random circular blobs radius 5-15'),
        (1, 1280,  720, 'blobs',   43,  5,  '1280x720, 5 random blobs'),
        (2,  640,  400, 'blobs',   44, 10,  '640x400, 10 random blobs'),
        (3, 1280,  800, 'black',    0,  0,  '1280x800, all-black (no blobs)'),
        (4, 1280,  800, 'white',    0,  1,  '1280x800, all-white (one giant blob)'),
        (5,  640,  400, 'single',   0,  1,  '640x400, single pixel blob at (100,100)'),
        (6, 1280,  800, 'near',    45,  2,  '1280x800, two blobs nearly touching (gap=2)'),
        (7, 1280,  800, 'overlap', 46,  2,  '1280x800, two blobs overlapping (should merge)'),
        # 640x400 cases distinct from each other (different seeds/blob counts) for
        # mocap_wrapper's double-buffer / race-condition tests (Group C): each must
        # have a different histogram AND a different blob table so a test can prove
        # the RTL published the correct frame's data, not a stale/adjacent one.
        (10, 640,  400, 'blobs',  100,  3,  '640x400, 3 random blobs (race test A)'),
        (11, 640,  400, 'blobs',  101,  7,  '640x400, 7 random blobs (race test B)'),
        (12, 640,  400, 'blobs',  102, 12,  '640x400, 12 random blobs (race test C)'),
    ]

    for idx, width, height, mode, seed, num_blobs, desc in cases:
        hex_path  = os.path.join(output_dir, f'frame_{idx:04d}.hex')
        meta_path = os.path.join(output_dir, f'frame_{idx:04d}_meta.json')

        blob_centers: list = []
        blob_radii:   list = []

        if mode == 'blobs':
            frame, blob_centers, blob_radii = make_circular_blob_frame(
                width, height, num_blobs, seed)

        elif mode == 'black':
            frame = np.zeros((height, width), dtype=np.uint8)

        elif mode == 'white':
            frame = np.full((height, width), 255, dtype=np.uint8)
            blob_centers = [[width // 2, height // 2]]
            blob_radii   = [min(width, height) // 2]

        elif mode == 'single':
            frame = np.zeros((height, width), dtype=np.uint8)
            cx, cy = 100, 100
            frame[cy, cx] = 255
            blob_centers = [[cx, cy]]
            blob_radii   = [0]

        elif mode == 'near':
            # Two blobs with gap of 2 pixels between their edges
            r = 20
            cx1 = width  // 4
            cx2 = cx1 + 2 * r + 2          # gap = 2
            cy  = height // 2
            frame = np.zeros((height, width), dtype=np.uint8)
            for cx in [cx1, cx2]:
                ys, xs = np.ogrid[:height, :width]
                mask = (xs - cx) ** 2 + (ys - cy) ** 2 <= r * r
                frame[mask] = 255
            blob_centers = [[cx1, cy], [cx2, cy]]
            blob_radii   = [r, r]

        elif mode == 'overlap':
            # Two blobs whose circles overlap — pixels merge, model treats as one connected region
            r = 25
            cx1 = width  // 2 - 20
            cx2 = width  // 2 + 20
            cy  = height // 2
            frame = np.zeros((height, width), dtype=np.uint8)
            for cx in [cx1, cx2]:
                ys, xs = np.ogrid[:height, :width]
                mask = (xs - cx) ** 2 + (ys - cy) ** 2 <= r * r
                frame[mask] = 255
            blob_centers = [[cx1, cy], [cx2, cy]]
            blob_radii   = [r, r]

        else:
            raise ValueError(f'Unknown mode: {mode}')

        words = frame_to_hex_words(frame)
        write_hex(hex_path, words)

        meta = {
            'width':       width,
            'height':      height,
            'threshold':   threshold,
            'seed':        seed,
            'num_blobs':   num_blobs,
            'blob_centers': blob_centers,
            'blob_radii':   blob_radii,
            'description':  desc,
        }
        write_meta(meta_path, meta)
        print(f'  [{idx:04d}] {desc}  -> {hex_path}')


def main() -> None:
    parser = argparse.ArgumentParser(description='Generate blob-detect test stimulus')
    parser.add_argument('--width',      type=int, default=1280)
    parser.add_argument('--height',     type=int, default=800)
    parser.add_argument('--num-blobs',  type=int, default=8)
    parser.add_argument('--seed',       type=int, default=42)
    parser.add_argument('--threshold',  type=int, default=128)
    parser.add_argument('--output-dir', type=str, default='./test_data/')
    args = parser.parse_args()

    print(f'Generating fixed test suite in {args.output_dir} ...')
    generate_test_cases(args.output_dir, threshold=args.threshold)
    print('Done.')


if __name__ == '__main__':
    main()
