# motion_capture

Embedded FPGA camera capture and video output pipeline targeting Xilinx Zybo Z7 and KV260 boards with OV9281 (global shutter) or OV5640 cameras. Hardware is designed in Vivado 2024.1 (IP Integrator block design), simulated with Vivado xsim. Software runs bare-metal on the ARM processor via Vitis, or under Linux via PetaLinux 2024.1 (KV260 only).

## Supported targets

| `PROJ=`         | Board       | Camera | SoC                          | Software        |
|-----------------|-------------|--------|------------------------------|-----------------|
| `zybo_ov9281`   | Zybo Z7-20  | OV9281 | Zynq-7020 / Cortex-A9       | Bare-metal      |
| `zybo_ov5640`   | Zybo Z7-20  | OV5640 | Zynq-7020 / Cortex-A9       | Bare-metal      |
| `kv260_ov9281`  | KV260       | OV9281 | Zynq UltraScale+ / Cortex-A53 | Bare-metal or Linux |

## Requirements

- Vivado 2024.1 and Vitis 2024.1
- PetaLinux 2024.1 (for Linux builds on KV260)
- Python venv for helper scripts and register map generation

```bash
source boot_env.sh    # sources Vivado, Vitis, PetaLinux, activates mocap_env
```

## Quick start

```bash
# Full build (Vivado + Vitis): KV260 target
make hw PROJ=kv260_ov9281

# Hardware only (generates .xsa)
make vivado PROJ=kv260_ov9281

# Linux build (PetaLinux — KV260 only)
make linux

# Clean all build artifacts
make clean
```

### Build outputs

| Artifact           | Path |
|--------------------|------|
| Bitstream          | `vivado/<proj>/<proj>_proj/<proj>_proj.bit` |
| Hardware definition | `vivado/<proj>/<proj>_proj/<proj>_proj.xsa` |
| Vitis ELF          | `vitis/<proj>/workspace/<proj>_app/Debug/<proj>_app.elf` |
| PetaLinux BOOT.BIN | `linux/kv260_ov9281_plnx/images/linux/BOOT.BIN` |
| PetaLinux image.ub | `linux/kv260_ov9281_plnx/images/linux/image.ub` |

## Repository layout

```
motion_capture/
├── Makefile                       # Top-level: vivado, vitis, linux, hw, full, clean
├── boot_env.sh                    # Source Xilinx tools + Python venv
├── requirements.txt               # Python dependencies (pip)
├── mocap_env/                     # Python venv (gitignored)
│
├── vivado/
│   ├── kv260_ov9281/              # KV260 + OV9281 Vivado project
│   │   ├── src/hdl/               # Custom HDL (ISP, stream_fifo)
│   │   ├── src/hdl/isp/           # ISP statistics block (histogram, counters)
│   │   └── src/sim/               # Testbenches (tb_isp_math, tb_isp_histogram, etc.)
│   ├── zybo_ov9281/               # Zybo + OV9281 Vivado project
│   ├── zybo_ov5640/               # Zybo + OV5640 Vivado project
│   └── ip_repo/
│       ├── blob_detect_rle/       # RLE streaming CCL blob detection IP
│       └── blob_detect_grid/      # Grid-based blob detection IP (experimental)
│
├── vitis/
│   ├── kv260_ov9281/src/          # Bare-metal app: ISP stats, camera init, histogram display
│   ├── zybo_ov9281/src/           # Bare-metal app: camera init, HDMI output
│   └── zybo_ov5640/src/           # Bare-metal app: interactive UART menu, resolution/AWB/gamma
│
├── linux/
│   ├── kv260_ov9281_plnx/         # PetaLinux project (see linux/kv260_ov9281_plnx/README.md)
│   │   └── project-spec/meta-user/
│   │       ├── recipes-apps/      # mocap-sanity, mocap-perf, mocap-server, etc.
│   │       ├── recipes-libs/      # mocap-common (shared headers)
│   │       ├── recipes-bsp/       # Device tree overlay, u-boot config
│   │       └── recipes-kernel/    # CSI-2 mono patch, kernel config fragments
│   └── nfs-mount-point/           # NFS-exported rootfs (deployed by make deploy)
│
├── helper_scripts/                # Host-side tools (see helper_scripts/README.md)
│   ├── blob_detect_rle_verify.py  # RLE blob detect end-to-end verify + visualize
│   ├── blob_detect_verify.py      # Grid blob detect end-to-end verify + visualize
│   ├── stream_client.py           # TCP client for mocap-server
│   ├── camera.py                  # Multi-camera OpenCV capture
│   └── ...
│
└── agents/                        # Development plans and session logs
```

## Video pipeline (KV260)

```
OV9281 (MIPI CSI-2, 2-lane)
       │
       ▼
 MIPI D-PHY RX  ──►  MIPI CSI-2 RX  ──►  ISP (snoop)  ──►  AXI VDMA  ──►  DDR4
                                             │
                                        256-bin histogram
                                        pixel sum
                                        cycle/frame counters
                                        frame_done IRQ
```

The ISP block is a **pure snoop** — it observes the AXI-Stream between the CSI-2 RX and VDMA without adding latency or backpressure. Histogram and statistics are read by software via AXI-Lite registers (UIO under Linux, direct register access bare-metal).

The block design is fully described in `vivado/<proj>/src/bd/system.tcl` and recreated by `create_proj.tcl`.

## Custom IP blocks

### ISP statistics (`vivado/kv260_ov9281/src/hdl/isp/`)

Passive AXI-Stream statistics block. 256-bin histogram, pixel sum accumulator, 64-bit cycle counter, frame counter, snapshot registers. PeakRDL register interface. See [`vivado/kv260_ov9281/src/hdl/isp/README.md`](vivado/kv260_ov9281/src/hdl/isp/README.md).

### Blob detection — RLE streaming CCL (`vivado/ip_repo/blob_detect_rle/`)

Streaming connected-component labeling blob detector. Three pipelined stages: run-length encoding, row-by-row merge with union-find, and per-blob accumulation with post-frame flatten (~128 cycles). Produces pixel-exact centroids and bounding boxes for up to 128 blobs. Algorithm matches Bailey (2019) and He, Chao, Suzuki (2008). Fully verified against a bit-exact Python model across 7 test scenarios. See [`vivado/ip_repo/blob_detect_rle/README.md`](vivado/ip_repo/blob_detect_rle/README.md).

### Blob detection — grid-based (`vivado/ip_repo/blob_detect_grid/`)

Experimental grid-based spatial hashing blob detector. Evaluated as sub-par compared to the RLE streaming CCL approach — centroids are limited to cell granularity (32x32 pixels) and the post-frame DFS scan takes thousands of cycles vs ~128 for the RLE flatten. Retained for reference. See [`vivado/ip_repo/blob_detect_grid/README.md`](vivado/ip_repo/blob_detect_grid/README.md).

## Simulation

Testbenches use Xilinx xsim (xvlog/xelab/xsim). Each IP and the ISP block have their own `sim/` directory with a Makefile.

### ISP testbenches (`vivado/kv260_ov9281/src/sim/`)

```bash
cd vivado/kv260_ov9281/src/sim
make sim TB=tb_isp_math        # Full integration test (registers + histogram + FSM)
make sim TB=tb_isp_histogram   # Histogram unit test (256-bin, write hazard, scrub, random stress)
make sim TB=tb_stream_fifo     # FIFO unit test (fill/drain, boundaries, backpressure)
make sim TB=tb_frame_rate      # Legacy FPS counter test
```

### Blob detection testbenches

```bash
# RLE blob detect — 7 multi-frame test cases
cd vivado/ip_repo/blob_detect_rle/sim
make verify    # gen_stim → model → compile → sim → compare (all 7 frames)

# Grid blob detect — single-frame test
cd vivado/ip_repo/blob_detect_grid/sim
make verify    # gen_stim → model → compile → sim → compare
```

## Linux / PetaLinux (KV260)

The KV260 variant has a PetaLinux 2024.1 software path alongside the bare-metal Vitis path. Same FPGA bitstream; Linux on the A53.

```bash
cd linux/kv260_ov9281_plnx
make all       # build + package + deploy (NFS root + TFTP)
```

The capture pipeline is loaded at runtime as a device tree overlay:
```bash
# On the KV260:
fpgautil -o /home/root/mocap-pipeline-overlay.dtbo -b /home/root/kv260_ov9281_proj.bit.bin
```

V4L2 capture apps (`mocap-sanity`, `mocap-perf`, `mocap-server`) are built as PetaLinux recipes and installed to the rootfs. See [`linux/kv260_ov9281_plnx/README.md`](linux/kv260_ov9281_plnx/README.md).

### Key kernel patch

The Xilinx CSI-2 RX driver has no mono format for RAW8 — it coerces Y8 to Bayer8, causing `STREAMON` to fail with `-EPIPE`. A one-line patch adds `MEDIA_BUS_FMT_Y8_1X8` to the CSI driver's LUT. See `recipes-kernel/linux/linux-xlnx/0001-xilinx-csi2rxss-add-Y8_1X8-mono-RAW8-mbus-code.patch`.

## Python helper scripts

```bash
source mocap_env/bin/activate
python helper_scripts/blob_detect_rle_verify.py --seed 42 --num-blobs 8
python helper_scripts/stream_client.py 10.0.0.100 5001
```

See [`helper_scripts/README.md`](helper_scripts/README.md).

Install dependencies: `pip install -r requirements.txt`

## Bare-metal applications (Vitis)

| Target | Entry point | Description |
|--------|-------------|-------------|
| kv260_ov9281 | `vitis/kv260_ov9281/src/main.cc` | ISP stats display: FPS, histogram bar chart, avg brightness. PL I2C via TCA9546A mux. |
| zybo_ov9281 | `vitis/zybo_ov9281/src/main.cc` | Camera init + HDMI output at 1280x720@60. PS I2C, VDMA read+write. |
| zybo_ov5640 | `vitis/zybo_ov5640/src/main.cc` | Interactive UART menu: resolution switching, register R/W, AWB modes, gamma, liquid lens focus. |
