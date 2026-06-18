#!/usr/bin/env python3
"""
blob_detect_verify.py — End-to-end blob detection verification with visualization.

Orchestrates:
  1. Generate stimulus frame (random blobs)
  2. Run Python functional model
  3. Compile + run xsim RTL simulation
  4. Compare outputs and display side-by-side matplotlib figure

Layout:
  Row 0: Input frame (full width)
  Row 1: Model results (left) | RTL results (right)

Centroids marked as 1-pixel-thick red X's. Bounding boxes in green.

Usage:
  source mocap_env/bin/activate
  python helper_scripts/blob_detect_verify.py [--width 640] [--height 400] [--num-blobs 5] [--seed 99]
"""

import argparse
import json
import math
import os
import subprocess
import sys
import tempfile

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

PROJ_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IP_DIR    = os.path.join(PROJ_ROOT, "vivado", "ip_repo", "blob_detect_grid")
MODEL_DIR = os.path.join(IP_DIR, "model")
SIM_DIR   = os.path.join(IP_DIR, "sim")
VIVADO_SETTINGS = "/tools/Xilinx/Vivado/2024.1/settings64.sh"

sys.path.insert(0, MODEL_DIR)
import gen_stimulus
import blob_detect_grid_model as model
from compare import load_rtl, load_model, compare, FIELDS


def generate_frame(width, height, num_blobs, seed, threshold, work_dir):
    frame, centers, radii = gen_stimulus.make_circular_blob_frame(
        width, height, num_blobs, seed)
    words = gen_stimulus.frame_to_hex_words(frame)

    hex_path = os.path.join(work_dir, "frame_0000.hex")
    meta_path = os.path.join(work_dir, "frame_0000_meta.json")

    gen_stimulus.write_hex(hex_path, words)
    gen_stimulus.write_meta(meta_path, {
        "width": width, "height": height, "threshold": threshold,
        "seed": seed, "num_blobs": num_blobs,
        "blob_centers": centers, "blob_radii": radii,
    })
    return frame, hex_path


def run_model(hex_path, width, height, threshold, work_dir):
    blobs = model.run_model(hex_path, width, height, threshold)
    out_path = os.path.join(work_dir, "blobs_model.json")
    with open(out_path, "w") as f:
        json.dump({"blob_count": len(blobs), "blobs": blobs}, f, indent=2)
    return blobs, out_path


def run_xsim(width, height, threshold, work_dir):
    sim_out = os.path.join(work_dir, "sim_out", "tb_blob_detect_grid")
    os.makedirs(sim_out, exist_ok=True)

    frame_src = os.path.join(work_dir, "frame_0000.hex")
    frame_dst = os.path.join(sim_out, "frame_0000.hex")
    if os.path.abspath(frame_src) != os.path.abspath(frame_dst):
        import shutil
        shutil.copy2(frame_src, frame_dst)

    print("[xsim] Compiling RTL...")
    hdl_dir = os.path.join(IP_DIR, "hdl")
    rdl_dir = os.path.join(IP_DIR, "rdl", "rdl_out", "rtl")

    srcs = [
        os.path.join(rdl_dir, "blob_detect_grid_regs_pkg.sv"),
        os.path.join(rdl_dir, "blob_detect_grid_regs.sv"),
        os.path.join(hdl_dir, "stream_fifo.sv"),
        os.path.join(hdl_dir, "cell_accumulator.sv"),
        os.path.join(hdl_dir, "grid_scanner.sv"),
        os.path.join(hdl_dir, "blob_emitter.sv"),
        os.path.join(hdl_dir, "blob_detect_grid_top.sv"),
        os.path.join(hdl_dir, "blob_detect_grid_wrapper.v"),
        os.path.join(SIM_DIR, "tb_blob_detect_grid.sv"),
    ]

    def vivado_cmd(cmd):
        full = f". {VIVADO_SETTINGS} && cd {sim_out} && {cmd}"
        r = subprocess.run(["bash", "-c", full], capture_output=True, text=True, timeout=300)
        if r.returncode != 0:
            print(f"[xsim] STDERR:\n{r.stderr[-2000:]}")
            print(f"[xsim] STDOUT:\n{r.stdout[-2000:]}")
            raise RuntimeError(f"xsim command failed: {cmd[:80]}...")
        return r

    vivado_cmd(f"xvlog --sv -i {hdl_dir} {' '.join(srcs)} --log xvlog.log")
    vivado_cmd("xelab tb_blob_detect_grid -snapshot tb_sim -debug all --log xelab.log")

    print("[xsim] Running simulation...")
    tcl_path = os.path.join(SIM_DIR, "wave_vcd.tcl")
    vivado_cmd(f"xsim tb_sim -tclbatch {tcl_path} --log xsim.log")

    rtl_hex = os.path.join(sim_out, "blobs_rtl.hex")
    if not os.path.exists(rtl_hex):
        raise RuntimeError(f"RTL output not found at {rtl_hex}")

    rtl_blobs = load_rtl(rtl_hex)
    print(f"[xsim] Done. Detected {len(rtl_blobs)} blobs.")
    return rtl_blobs


def draw_blobs(ax, frame, blobs, title, width, height):
    ax.imshow(frame, cmap="gray", vmin=0, vmax=255)
    ax.set_title(title, fontsize=10)

    for b in blobs:
        count = b["count"]
        if count == 0:
            continue
        cx = b["sum_x"] / count
        cy = b["sum_y"] / count

        # 1-pixel-thick red X
        arm = max(4, min(width, height) * 0.01)
        ax.plot([cx - arm, cx + arm], [cy - arm, cy + arm],
                color="red", linewidth=1, solid_capstyle="butt")
        ax.plot([cx - arm, cx + arm], [cy + arm, cy - arm],
                color="red", linewidth=1, solid_capstyle="butt")

        # Bounding box
        bw = b["xmax"] - b["xmin"] + 1
        bh = b["ymax"] - b["ymin"] + 1
        ax.add_patch(Rectangle(
            (b["xmin"] - 0.5, b["ymin"] - 0.5), bw, bh,
            fill=False, edgecolor="lime", linewidth=0.8))

        ax.text(b["xmin"], b["ymin"] - 3,
                f"n={count}", color="lime", fontsize=6)

    ax.set_xlim(-0.5, width - 0.5)
    ax.set_ylim(height - 0.5, -0.5)
    ax.set_aspect("equal")


def main():
    parser = argparse.ArgumentParser(description="Blob detection end-to-end verify + visualize")
    parser.add_argument("--width",      type=int, default=1280,
                        help="Frame width (must match tb localparam HRES, default 1280)")
    parser.add_argument("--height",     type=int, default=800,
                        help="Frame height (must match tb localparam VRES, default 800)")
    parser.add_argument("--num-blobs",  type=int, default=5)
    parser.add_argument("--seed",       type=int, default=99)
    parser.add_argument("--threshold",  type=int, default=128)
    parser.add_argument("--work-dir",   type=str, default=None,
                        help="Working directory (default: temp dir)")
    parser.add_argument("--no-xsim",    action="store_true",
                        help="Skip RTL simulation (model only)")
    args = parser.parse_args()

    if args.work_dir:
        work_dir = args.work_dir
        os.makedirs(work_dir, exist_ok=True)
    else:
        work_dir = tempfile.mkdtemp(prefix="blob_verify_")
    print(f"Working directory: {work_dir}")

    # 1. Generate stimulus
    print(f"\n[1] Generating stimulus: {args.width}x{args.height}, "
          f"{args.num_blobs} blobs, seed={args.seed}")
    frame, hex_path = generate_frame(
        args.width, args.height, args.num_blobs, args.seed,
        args.threshold, work_dir)

    # 2. Run functional model
    print(f"\n[2] Running Python functional model...")
    model_blobs, model_json = run_model(
        hex_path, args.width, args.height, args.threshold, work_dir)
    print(f"    Model detected {len(model_blobs)} blobs")
    for i, b in enumerate(model_blobs):
        cx = b["sum_x"] / b["count"] if b["count"] else 0
        cy = b["sum_y"] / b["count"] if b["count"] else 0
        print(f"      [{i}] count={b['count']:5d}  centroid=({cx:.1f},{cy:.1f})")

    # 3. Run xsim (unless --no-xsim)
    rtl_blobs = None
    if not args.no_xsim:
        print(f"\n[3] Running xsim RTL simulation...")
        rtl_blobs = run_xsim(args.width, args.height, args.threshold, work_dir)
        for i, b in enumerate(rtl_blobs):
            cx = b["sum_x"] / b["count"] if b["count"] else 0
            cy = b["sum_y"] / b["count"] if b["count"] else 0
            print(f"      [{i}] count={b['count']:5d}  centroid=({cx:.1f},{cy:.1f})")

        # 4. Compare
        print(f"\n[4] Comparing model vs RTL...")
        passed = compare(rtl_blobs, model_blobs)
        print("    PASS" if passed else "    FAIL")
    else:
        print(f"\n[3] Skipping xsim (--no-xsim)")

    # 5. Visualize
    print(f"\n[5] Generating visualization...")
    if rtl_blobs is not None:
        fig = plt.figure(figsize=(14, 10))
        gs = fig.add_gridspec(2, 2, height_ratios=[1, 1], hspace=0.3, wspace=0.15)

        ax_input = fig.add_subplot(gs[0, :])
        ax_input.imshow(frame, cmap="gray", vmin=0, vmax=255)
        ax_input.set_title(f"Input: {args.width}x{args.height}, "
                           f"{args.num_blobs} blobs, seed={args.seed}, "
                           f"threshold={args.threshold}", fontsize=11)
        ax_input.set_xlim(-0.5, args.width - 0.5)
        ax_input.set_ylim(args.height - 0.5, -0.5)
        ax_input.set_aspect("equal")

        ax_model = fig.add_subplot(gs[1, 0])
        ax_rtl   = fig.add_subplot(gs[1, 1])

        draw_blobs(ax_model, frame, model_blobs,
                   f"Python Model ({len(model_blobs)} blobs)",
                   args.width, args.height)
        draw_blobs(ax_rtl, frame, rtl_blobs,
                   f"RTL / xsim ({len(rtl_blobs)} blobs)",
                   args.width, args.height)
    else:
        fig, (ax_input, ax_model) = plt.subplots(2, 1, figsize=(10, 9))
        ax_input.imshow(frame, cmap="gray", vmin=0, vmax=255)
        ax_input.set_title(f"Input: {args.width}x{args.height}, "
                           f"{args.num_blobs} blobs", fontsize=11)
        ax_input.set_xlim(-0.5, args.width - 0.5)
        ax_input.set_ylim(args.height - 0.5, -0.5)
        ax_input.set_aspect("equal")

        draw_blobs(ax_model, frame, model_blobs,
                   f"Python Model ({len(model_blobs)} blobs)",
                   args.width, args.height)

    plt.tight_layout()
    out_png = os.path.join(work_dir, "blob_verify.png")
    plt.savefig(out_png, dpi=150)
    print(f"    Saved to {out_png}")
    plt.show()


if __name__ == "__main__":
    main()
