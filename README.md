# motion_capture

Embedded FPGA camera capture and video output pipeline targeting Xilinx Zybo Z7 and KV260 boards with OV9281 (global shutter) or OV5640 cameras. Hardware is designed in Vivado (IP Integrator block design), simulated with Vivado xsim; software runs bare-metal on the ARM processor via Vitis until a port to running linux (TBD if petalinux or straight yocto)

## Supported targets

| `PROJ=`         | Board    | Camera | SoC                        |
|-----------------|----------|--------|----------------------------|
| `zybo_ov9281`   | Zybo Z7-20 | OV9281 | Zynq-7020 / Cortex-A9   |
| `zybo_ov5640`   | Zybo Z7-20 | OV5640 | Zynq-7020 / Cortex-A9   |
| `kv260_ov9281`  | KV260    | OV9281 | Zynq UltraScale+ / Cortex-A53 |

## Requirements

- Vivado 2024.1 and Vitis 2024.1 on `PATH`
- Python venv for helper scripts and register map generation (see below)

## Quick start

```bash
# Full build: Vivado hardware + Vitis software
make vitis_build [PROJ=zybo_ov9281]

# Hardware only (generates .xsa)
make vivado_build [PROJ=zybo_ov9281]

# Vitis only (run from vitis/<proj>/ after vivado_build)
vitis -s create_vitis.py

# Clean all build artifacts
make clean
```

### Build outputs

| Artifact           | Path |
|--------------------|------|
| Bitstream          | `vivado/<proj>/<proj>_proj/<proj>_proj.bit` |
| Hardware definition | `vivado/<proj>/<proj>_proj/<proj>_proj.xsa` |
| Application ELF    | `vitis/<proj>/workspace/<proj>_app/Debug/<proj>_app.elf` |

## Repository layout

```
motion_capture/
├── Makefile                    # Top-level build entry point
├── requirements.txt            # Python dependencies (pip)
├── mocap_env/                  # Python venv (gitignored)
├── helper_scripts/
│   ├── camera.py               # Multi-camera FPS test / image capture
│   ├── display_image.py        # View raw image dumps from DDR3
│   ├── socket_server.py        # TCP server (port 5000) for remote control
│   └── test.py
├── vivado/
│   ├── zybo_ov9281/            # Zybo + OV9281 Vivado project
│   ├── zybo_ov5640/            # Zybo + OV5640 Vivado project
│   └── kv260_ov9281/           # KV260 + OV9281 Vivado project
│       ├── create_proj.tcl     # Creates Vivado project from source
│       ├── export_bitstream.tcl
│       └── src/
│           ├── bd/system.tcl   # Block design
│           ├── hdl/            # Custom HDL (module references)
│           │   ├── isp/        # ISP pipeline (see below)
│           │   └── isp_math_wrapper.v
│           └── sim/            # Simulation testbenches
└── vitis/
    ├── zybo_ov9281/
    ├── zybo_ov5640/
    └── kv260_ov9281/
        └── src/
            ├── main.cc         # ARM bare-metal application
            └── cam/            # Camera / peripheral drivers
```

## Video pipeline

```
OV9281/OV5640 Camera
       │ MIPI CSI-2 (2-lane)
       ▼
 MIPI D-PHY RX  ──►  MIPI CSI-2 RX  ──►  ISP  ──►  AXI VDMA (write)  ──►  DDR3
                                                                               │
DVI/HDMI output  ◄──  rgb2dvi  ◄──  Gamma  ◄──  Bayer-to-RGB  ◄──  AXI VDMA (read)
```

The block design is fully described in `vivado/<proj>/src/bd/system.tcl` and recreated by `create_proj.tcl`.

## ISP module (KV260)

The ISP pipeline sits between the MIPI CSI-2 Rx subsystem and AXI VDMA. Source is in `vivado/kv260_ov9281/src/hdl/isp/`.

| File | Description |
|------|-------------|
| `isp_math_wrapper.v` | Plain Verilog shell for IP Integrator block diagram |
| `isp_math_top.sv` | SystemVerilog top: instantiates register block, AXI-Stream passthrough |
| `isp_regs.rdl` | PeakRDL register map definition |
| `rdl_out/rtl/isp_regs.sv` | Generated AXI4-Lite register block |
| `rdl_out/rtl/isp_regs_pkg.sv` | Generated SystemVerilog package (struct types) |
| `rdl_out/sw/isp_regs.h` | Generated C header for bare-metal driver |
| `stats_engine.sv` | Pixel statistics engine (histogram, avg, frame/cycle counters) |
| `frame_rate_counter.sv` | Standalone 100-frame FPS measurement module |

### Regenerating the register map

The `rdl_out/` directory is generated from `isp_regs.rdl` via [PeakRDL](https://peakrdl.readthedocs.io/).

```bash
source mocap_env/bin/activate
cd vivado/kv260_ov9281/src/hdl/isp
make          # regenerates RTL, C header, and HTML docs
make rtl      # RTL only
make header   # C header only
make html     # HTML register map docs only
```

### ISP register map summary

| Address | Register | Access | Description |
|---------|----------|--------|-------------|
| `0x000` | CTRL | WO | Bit 0: START, Bit 1: RESET, Bit 2: SNAPSHOT |
| `0x004` | STATUS | RO | Bit 0: BUSY, Bit 1: DATA_READY |
| `0x008` | HRES | R/W | Horizontal resolution (default 1280) |
| `0x00C` | VRES | R/W | Vertical resolution (default 800) |
| `0x010` | CYCLE_CNT_LO | RO | Free-running cycle counter [31:0] |
| `0x014` | CYCLE_CNT_HI | RO | Free-running cycle counter [63:32] |
| `0x018` | CYCLE_SNAP_LO | RO | Cycle counter snapshot [31:0] |
| `0x01C` | CYCLE_SNAP_HI | RO | Cycle counter snapshot [63:32] |
| `0x020` | FRAME_CNT | RO | Frame counter |
| `0x024` | FRAME_SNAP | RO | Frame counter snapshot |
| `0x028` | AVG_PIXEL | RO | Mean pixel brightness (0–255) |
| `0x030–0x42C` | HIST[256] | RO | 256-bin pixel histogram |

Trigger CTRL.SNAPSHOT before reading counter pairs to get a coherent 64-bit value.

## Simulation

Testbenches use Xilinx `xvlog`/`xelab`/`xsim`. All outputs go to `sim_out/<TB>/`.

```bash
cd vivado/kv260_ov9281/src/sim

make                        # compile + run tb_isp_math (default)
make TB=tb_frame_rate       # run the frame rate counter testbench
make compile TB=tb_isp_math # compile only
make run     TB=tb_isp_math # run only (after compile)
make clean   TB=tb_isp_math # remove sim_out/tb_isp_math/
make cleanall               # remove all sim outputs
```

## Python helper scripts

```bash
source mocap_env/bin/activate

python helper_scripts/camera.py         # multi-camera FPS test and image capture
python helper_scripts/display_image.py  # view raw DDR3 image dumps
python helper_scripts/socket_server.py  # TCP server on port 5000 for remote control
```

Install dependencies: `pip install -r requirements.txt`
