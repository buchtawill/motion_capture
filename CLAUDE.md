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

### Simulation / Testbenches

All testbenches live in `vivado/kv260_ov9281/src/sim/` and share a single Makefile.
Requires Vivado (xvlog/xelab/xsim) on PATH.

```bash
cd vivado/kv260_ov9281/src/sim

# Compile + run (default TB: tb_isp_histogram), produce FST waveform
make sim [TB=<testbench>]

# Compile only / run only
make compile [TB=<testbench>]
make sim_fst [TB=<testbench>]

# Open waveform in Vivado GUI
make sim_wdb [TB=<testbench>]

# Clean
make clean        # remove sim_out/
make cleanall     # same (alias)
```

Waveforms are written to `sim_out/<TB>/waves.vcd` (converted to `<TB>.fst`).

| Testbench | DUT(s) | What it covers |
|---|---|---|
| `tb_isp_histogram` | `isp_histogram.sv` + `stream_fifo.sv` | 256-bin histogram: counting correctness, write-hazard, RAM scrub, full-image stream, RAM read-port (frontdoor), randomized 100-iteration stress |
| `tb_stream_fifo` | `stream_fifo.sv` | Fill/drain order, full/empty boundaries, backpressure |
| `tb_isp_math` | `isp_math_wrapper.v` + `isp_math_top.sv` | AXI-Lite register reads (HRES, VRES reset defaults) |
| `tb_frame_rate` | `isp_wrapper.v` | FPS measurement, mid-count reset and disable |

**`tb_isp_histogram` verification strategy**

Bin counts are verified through two independent paths on every check:
- **Backdoor** — direct `dut_isp_histogram.hist_mem[]` array access (zero latency, always available)
- **Frontdoor** — hardware `ram_addr_i` / `ram_data_o` port (requires `hist_en_i=0`; DUT presents registered output one cycle after address is driven)

Key tasks: `send_beat`, `wait_drain`, `wait_scrub_done`, `read_bin_ram`, `check_bin`, `check_bin_ram`, `check_all_zero`, `check_all_bins(label, expected, backdoor=1, frontdoor=1)`.

### Vivado Build Flow

`create_proj.tcl` — Creates the Vivado project, adds all `src/` files (HDL, constraints, block design TCL), and sets file properties.

`export_bitstream.tcl` — Runs synthesis → implementation → bitstream generation → exports `.xsa` for Vitis.

`vitis/<proj>/create_vitis.py` — Python script run by Vitis CLI to create the platform component (from `.xsa`) and application component, then build the ELF.

## Linux / PetaLinux (KV260 variant)

The `kv260_ov9281` variant has a second, parallel software path: PetaLinux 2024.1 instead of bare-metal Vitis. Same FPGA bitstream; different OS on the A53.

**Pipeline (Linux view):**
```
OV9281 ──I2C config──► axi_iic_0 (0xa0020000)
       └─MIPI CSI-2───► mipi_csi2_rx_subsyst (0xa0010000) ──► [isp_math_wrapper, transparent] ──► axi_vdma_0 (0xa0000000) ──► DDR
                                                                       │
                                                                  AXI-Lite + UIO (histogram stats, controlled separately from V4L2)
```

The ISP wrapper is **passive** — it snoops the AXI-Stream and exposes histogram registers, but does not modify TDATA/TVALID/TLAST/TUSER/TKEEP. For V4L2's media graph, wire CSI output → VDMA directly and skip ISP. Histogram driven by userspace via UIO (`generic-uio` compatible, blocking `read()` returns on interrupt).

### Project layout

| Path | Purpose |
|---|---|
| `linux/kv260_ov9281_plnx/` | PetaLinux project root |
| `linux/kv260_ov9281_plnx/Makefile` | Wraps `petalinux-config`/`-build`/`-package`/deploy. `make all` = build + package + deploy. |
| `linux/kv260_ov9281_plnx/project-spec/meta-user/` | All user customizations — commit this. |
| `linux/kv260_ov9281_plnx/components/plnx_workspace/device-tree/device-tree/pl.dtsi` | **Auto-generated from .xsa — never edit.** Regenerated on `petalinux-config --get-hw-description`. |
| `linux/kv260_ov9281_plnx/project-spec/meta-user/recipes-bsp/device-tree/files/system-user.dtsi` | Linux device-tree overrides (bootargs, OV9281 node, endpoint wiring). |
| `linux/kv260_ov9281_plnx/project-spec/meta-user/meta-xilinx-tools/recipes-bsp/uboot-device-tree/files/system-user.dtsi` | U-boot device-tree overrides (GEM3 ethernet enable, DP83867 PHY). |
| `linux/kv260_ov9281_plnx/project-spec/meta-user/recipes-bsp/u-boot/files/platform-top.h` | U-boot env compile-ins. Uses `CFG_EXTRA_ENV_SETTINGS` (renamed from `CONFIG_EXTRA_ENV_SETTINGS` in u-boot 2022.07+). |
| `linux/nfs-mount-point/` | NFS-exported rootfs. Owned by root after `sudo tar xzf images/linux/rootfs.tar.gz`. NFS server configured with `no_root_squash` in `/etc/exports`. |

### Boot flow

- **BOOT.BIN** (FSBL + PMU firmware + u-boot) loaded from SD or QSPI. Hardware-aware: regenerate after any XSA change.
- **image.ub** (kernel + DT + initramfs) loaded by u-boot via TFTP from `/srv/tftp` on the host (NOT `/tftpboot` — tftpd-hpa's default).
- **Kernel** mounts root over NFS via bootargs:
  ```
  earlycon console=ttyPS1,115200 root=/dev/nfs rootfstype=nfs \
  nfsroot=10.0.0.70:/home/will/Desktop/motion_capture/linux/nfs-mount-point,tcp,nfsvers=3 \
  ip=dhcp rw cma=32M
  ```
  Set in Linux `system-user.dtsi` under `chosen { bootargs = "..."; };`.
- **U-boot env** (`serverip=10.0.0.70`) compiled into binary via `CFG_EXTRA_ENV_SETTINGS` in `platform-top.h`. KV260 BSP **ships with GEM disabled** in u-boot — the u-boot `system-user.dtsi` re-enables `gem3` with TI DP83867 PHY config and MIO38 reset GPIO.

### Build / iterate

```bash
cd linux/kv260_ov9281_plnx
make config        # petalinux-config --get-hw-description ../../vivado/kv260_ov9281/kv260_ov9281_proj/
make build         # petalinux-build (incremental)
make package       # repacks BOOT.BIN (FSBL + bitstream + u-boot)
make deploy        # extracts rootfs to NFS dir (sudo), removes INITRD from /tftpboot/pxelinux.cfg/default
make all           # build + package + deploy
```

Single-component rebuild: `petalinux-build -c u-boot` / `-c device-tree` / `-c kernel`. Force from scratch: append `-x cleansstate` (two s's — common typo). After an XSA change, **always** rerun `make config` first; FSBL and PL DTSI are derived from it. Bitstream changes also require `make package` to repack BOOT.BIN.

Passwordless sudo for the deploy targets is configured in `/etc/sudoers.d/petalinux-deploy`.

### Camera driver

Use the **mainline `ov9282.c`** driver (`drivers/media/i2c/ov9282.c`). It probes the chip ID and supports OV9281 silicon at runtime, but:

- **DT compatible must be `"ovti,ov9282"`** — the driver's `of_match_table` does NOT also match `"ovti,ov9281"`, even though the silicon does. Wrong compatible = driver never binds.
- Required DT properties (per `Documentation/devicetree/bindings/media/i2c/ovti,ov9282.yaml`): `compatible`, `reg`, `clocks` (`clock-names = "inclk"`), `port`. Supplies and reset-gpios are optional.
- The `clocks` value is used for PLL math (`clk_get_rate()`). Wrong frequency = wrong line rate, not a probe failure.
- Driver exposes `MEDIA_BUS_FMT_Y10_1X10` and `MEDIA_BUS_FMT_Y8_1X8` (mono). RAW10 mono may require Xilinx ISI/IPI driver patches if format negotiation fails downstream.

ArduCAM's out-of-tree OV9281 driver (`github.com/ArduCAM/ov9281_driver`, Rockchip-derived) is an alternative reference — it has explicit power sequencing (8192 reference-clock-cycle delay after XSHUTDOWN before first I2C) and three preset modes (1280×800, 1280×720, 640×400). If the mainline driver's register init causes image artifacts on OV9281 silicon, this is the fallback.

**KV260 camera power/clock note:** The IAS connector's camera power and ref clock are gated through an on-SOM **TCA6408A I2C GPIO expander** (`CONFIG_GPIO_PCA953X`). If the OV9281 doesn't enumerate on I2C at all, either expose the expander in DT and reference its GPIOs from a regulator/clock binding, or flip the relevant pins from userspace (`gpioset`) before launching V4L2.

### Required kernel configs

Enable in `petalinux-config -c kernel`. Use `/` search to confirm exact symbol names — Xilinx fork has renamed several over kernel versions.

```
CONFIG_MEDIA_CONTROLLER=y          # enable first — gates submenus
CONFIG_VIDEO_V4L2_SUBDEV_API=y     # ditto
CONFIG_VIDEO_OV9282=m              # Multimedia → Media ancillary → Camera sensor devices
CONFIG_VIDEO_XILINX=m
CONFIG_VIDEO_XILINX_CSI2RXSS=m
CONFIG_VIDEO_XILINX_DMA=m          # V4L2 wrapper around VDMA (verify exact symbol)
CONFIG_XILINX_DMA=m                # dmaengine backend
CONFIG_I2C_XILINX=m
CONFIG_GPIO_PCA953X=m              # TCA6408A on KV260 SOM
CONFIG_UIO_PDRV_GENIRQ=m           # for ISP histogram via generic-uio
```

### V4L2 capture flow (userspace)

```
open(/dev/videoN)
  → VIDIOC_S_FMT       (negotiate format + resolution)
  → VIDIOC_REQBUFS     (allocate buffers, mmap)
  → VIDIOC_QBUF × N    (queue all buffers)
  → VIDIOC_STREAMON
  → loop: VIDIOC_DQBUF → process frame → VIDIOC_QBUF
  → VIDIOC_STREAMOFF
```

Sanity check the media graph with `media-ctl -p` and `v4l2-ctl --list-devices` before opening a node.

### Gotchas

- **PL clocks gated after `fpgautil` reload.** Check with `cat /sys/kernel/debug/clk/clk_summary | grep pl`. If PL0 is off, either reference the PL clock in DT (clean) or kick it with `devmem 0xFF5E00C0 32 0x01010800` (dirty).
- **Xilinx pipeline drivers may demand stub ops.** Per Virtana writeup, you may need to add empty `link_setup` media entity op and `s_power` subdev op stubs to the sensor driver if the Xilinx subgraph code unconditionally calls them.
- **`/tftpboot` vs `/srv/tftp`.** tftpd-hpa serves from `/srv/tftp` by default. Either edit `/etc/default/tftpd-hpa` to point at `/tftpboot`, or update `make deploy-boot` to write into `/srv/tftp`.
- **Netplan + cloud-init.** `/etc/netplan/50-cloud-init.yaml` can get rewritten on boot; disable cloud-init network management with `network: {config: disabled}` in `/etc/cloud/cloud.cfg.d/99-disable-network.cfg`.
- **`petalinux-build -x mrproper` deletes configs.** Use `distclean` for routine cleaning. Incremental builds are usually sufficient.

### Source control (Linux side)

Commit: `project-spec/`, `.xsa`, helper scripts, Makefile.
Ignore: `build/`, `images/`, `components/`, `project-spec/hw-description/` (all regenerated from `.xsa`).
