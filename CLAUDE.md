# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Embedded FPGA-based camera capture and video output pipeline targeting Xilinx Zybo Z7 and KV260 boards with OV9281 (global shutter) or OV5640 cameras. Hardware is designed in Vivado (IP Integrator block design); software runs bare-metal on the ARM processor via Vitis.

**Supported project variants** (`PROJ=`):
- `zybo_ov9281` (default) — Zybo Z7-20, OV9281, Zynq-7020 / Cortex-A9
- `zybo_ov5640` — Zybo Z7-20, OV5640, Zynq-7020 / Cortex-A9
- `kv260_ov9281` — KV260, OV9281, Zynq UltraScale+ / Cortex-A53

## Build Commands

Requires Vivado 2024.1 and Vitis 2024.1 on PATH.

```bash
# Full build: Vivado hardware + Vitis software
make vitis_build [PROJ=zybo_ov9281]

# Hardware only (generates .xsa)
make vivado_build [PROJ=zybo_ov9281]

# Vitis only (from vitis/<proj>/ directory, after vivado_build)
vitis -s create_vitis.py

# Clean all build artifacts
make clean
```

**Python helper scripts** (requires Python venv):
```bash
source mocap_env/bin/activate
python helper_scripts/camera.py       # Multi-camera FPS test/capture
python helper_scripts/display_image.py # View raw image dumps from DDR3
python helper_scripts/socket_server.py # TCP server (port 5000) for remote control
```

## Build Outputs

| Artifact | Path |
|---|---|
| Bitstream | `vivado/<proj>/<proj>_proj/<proj>_proj.bit` |
| Hardware definition | `vivado/<proj>/<proj>_proj/<proj>_proj.xsa` |
| Application ELF | `vitis/<proj>/workspace/<proj>_app/Debug/<proj>_app.elf` |

## Architecture

### Video Pipeline (Hardware)

```
OV9281/OV5640 Camera
       │ MIPI CSI-2 (2-lane)
       ▼
 MIPI D-PHY RX  ──►  MIPI CSI-2 RX  ──►  AXI VDMA (write)  ──►  DDR3 frame buffer
                                                                         │
DVI/HDMI output  ◄──  rgb2dvi  ◄──  Gamma  ◄──  Bayer-to-RGB  ◄──  AXI VDMA (read)
```

The block design is fully described in `vivado/<proj>/src/bd/system.tcl` and recreated by `create_proj.tcl`. All Xilinx IP (MIPI D-PHY, CSI-2, VDMA, video timing controller) is instantiated via IP Integrator; custom logic is added as **module references** (not packaged IP).

### Custom HDL (`vivado/<proj>/src/hdl/`)

**Zybo projects:**
- `DVIClocking.vhd` — BUFIO/BUFR-based 5x/1x clock splitting for DVI serializer
- `SyncAsync.vhd` — Configurable double-FF synchronizer for clock domain crossing
- `SyncAsyncReset.vhd` — Reset bridge (asserts async, de-asserts synchronously)

**KV260 project:**
- `concat_signals.v` — `expand_8_to_12` module: pads 8-bit camera data to 12-bit with 4 LSB zeros (instantiated 3x for RGB channels)

> `ip_repo/` is deprecated Digilent IP and is not used.

### Software (Vitis Bare-Metal)

Entry point: `vitis/<proj>/src/main.cc`

The ARM application:
1. Initializes GPIO (camera reset/power), I2C (camera registers), VDMA (frame buffers), and video output
2. Loads OV9281/OV5640 register tables over I2C to configure resolution and frame rate
3. Starts interrupt-driven video pipeline (Zynq GIC)
4. Runs a UART interactive menu for live control: resolution switching, register read/write, AWB modes, gamma correction, image format (Raw/RGB)

**Key driver headers** (`vitis/<proj>/src/cam/`):

| File | Purpose |
|---|---|
| `OV9281.h` | Camera register tables (720p@60, 1080p@15/30, AWB modes) and I2C init sequence |
| `AXI_VDMA.h` | C++ template driver for AXI VDMA frame buffer DMA |
| `PS_IIC.h` | Zynq PS I2C driver for camera register access |
| `PS_GPIO.h` | GPIO driver for camera control signals |
| `ScuGicInterruptController.h` | Zynq GIC interrupt controller wrapper |

### Vivado Build Flow

`create_proj.tcl` — Creates the Vivado project, adds all `src/` files (HDL, constraints, block design TCL), and sets file properties.

`export_bitstream.tcl` — Runs synthesis → implementation → bitstream generation → exports `.xsa` for Vitis.

`vitis/<proj>/create_vitis.py` — Python script run by Vitis CLI to create the platform component (from `.xsa`) and application component, then build the ELF.
