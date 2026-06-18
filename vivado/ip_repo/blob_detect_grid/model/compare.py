"""
compare.py — Compare RTL output against model output for blob_detect_grid IP.

RTL output format (blobs_rtl.hex):
  Line 0:        blob_count  (hex)
  Lines 1..7N:   7 hex values per blob in order:
                   count, sum_x, sum_y, xmin, xmax, ymin, ymax

Both blob lists are sorted by (ymin, xmin) before comparing.
Exit code 0 on pass, 1 on fail.

Usage:
  python compare.py --rtl-output blobs_rtl.hex --model-output blobs_model.json
"""

import argparse
import json
import sys


FIELDS = ['count', 'sum_x', 'sum_y', 'xmin', 'xmax', 'ymin', 'ymax']


# ---------------------------------------------------------------------------
# Parsers
# ---------------------------------------------------------------------------

def load_rtl(path: str) -> list[dict]:
    """Parse blobs_rtl.hex → list of blob dicts."""
    lines: list[str] = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line and not line.startswith('//'):
                lines.append(line)

    if not lines:
        return []

    blob_count = int(lines[0], 16)
    expected_lines = 1 + blob_count * len(FIELDS)
    if len(lines) < expected_lines:
        raise ValueError(
            f'RTL file has {len(lines)} data lines but header says {blob_count} blobs '
            f'(expected {expected_lines} lines total)')

    blobs: list[dict] = []
    for i in range(blob_count):
        base = 1 + i * len(FIELDS)
        blob = {field: int(lines[base + j], 16) for j, field in enumerate(FIELDS)}
        blobs.append(blob)

    blobs.sort(key=lambda b: (b['ymin'], b['xmin']))
    return blobs


def load_model(path: str) -> list[dict]:
    """Parse blobs_model.json → list of blob dicts (already sorted)."""
    with open(path) as f:
        data = json.load(f)
    blobs = data.get('blobs', [])
    blobs.sort(key=lambda b: (b['ymin'], b['xmin']))
    return blobs


# ---------------------------------------------------------------------------
# Comparison
# ---------------------------------------------------------------------------

def compare(rtl_blobs: list[dict], model_blobs: list[dict]) -> bool:
    """
    Compare two blob lists field-by-field.
    Returns True if all match, False otherwise. Prints a report.
    """
    ok = True

    if len(rtl_blobs) != len(model_blobs):
        print(f'FAIL: blob count mismatch — RTL={len(rtl_blobs)}, model={len(model_blobs)}')
        ok = False
        # Still attempt field-level comparison for the overlapping blobs
    else:
        print(f'Blob count: {len(rtl_blobs)} (match)')

    n = min(len(rtl_blobs), len(model_blobs))
    for i in range(n):
        r = rtl_blobs[i]
        m = model_blobs[i]
        mismatches = [f for f in FIELDS if r[f] != m[f]]
        if mismatches:
            ok = False
            print(f'FAIL blob[{i}]:')
            print(f'  {"field":<8}  {"RTL":>12}  {"model":>12}  {"match"}')
            for field in FIELDS:
                match_str = 'OK' if r[field] == m[field] else '*** MISMATCH ***'
                print(f'  {field:<8}  {r[field]:>12}  {m[field]:>12}  {match_str}')
        else:
            print(f'  blob[{i}]: OK  '
                  f'count={r["count"]}  '
                  f'centroid=({r["sum_x"]/r["count"] if r["count"] else "?":.1f},'
                  f'{r["sum_y"]/r["count"] if r["count"] else "?":.1f})  '
                  f'bbox=({r["xmin"]},{r["ymin"]})-({r["xmax"]},{r["ymax"]})')

    return ok


# ---------------------------------------------------------------------------
# Standalone CLI
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description='Compare RTL vs model output for blob_detect_grid')
    parser.add_argument('--rtl-output',   required=True, help='RTL hex output file')
    parser.add_argument('--model-output', required=True, help='Model JSON output file')
    args = parser.parse_args()

    rtl_blobs   = load_rtl(args.rtl_output)
    model_blobs = load_model(args.model_output)

    passed = compare(rtl_blobs, model_blobs)

    if passed:
        print('\nPASS: RTL output matches model.')
        sys.exit(0)
    else:
        print('\nFAIL: RTL output does not match model.')
        sys.exit(1)


if __name__ == '__main__':
    main()
