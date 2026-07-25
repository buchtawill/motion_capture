# kv260_ov9281_plnx — PetaLinux Project

PetaLinux 2024.1 project for the Kria KV260 board with OV9281 global-shutter camera. Boots via TFTP (image.ub) and NFS root filesystem. The FPGA bitstream and device tree overlay are loaded at runtime via `fpgautil`.

## Build

```bash
cd linux/kv260_ov9281_plnx

make config    # petalinux-config --get-hw-description (points at Vivado XSA)
make build     # petalinux-build (incremental)
make package   # repack BOOT.BIN (FSBL + bitstream + u-boot)
make deploy    # extract rootfs to NFS dir, strip INITRD from PXE config
make all       # build + package + deploy
```

Single-component rebuild: `petalinux-build -c device-tree` / `-c kernel` / `-c u-boot`. Force from scratch: append `-x cleansstate`.

After an XSA change, always rerun `make config` first — FSBL and PL DTSI are derived from it.

## Boot flow

1. **BOOT.BIN** (FSBL + PMU firmware + u-boot) loaded from SD or QSPI
2. **image.ub** (kernel + DT + initramfs) loaded by u-boot via TFTP from `/srv/tftp`. 
3. **Kernel** mounts root over NFS:
   ```
   root=/dev/nfs nfsroot=10.0.0.70:/home/will/Desktop/motion_capture/linux/nfs-mount-point,tcp,nfsvers=3
   ip=dhcp cma=32M clk_ignore_unused fw_devlink=permissive
   ```
4. **FPGA overlay** loaded by user after boot (see below)

Setting up tftp server / nfs and caveats for INITRD:
Super helpful forum post:
Here is my set of markdown notes for getting TFTP and NFS up and running on my ZCU111:
# Booting from TFTP
## Configure TFTP server on the host system:
Install tftpd-hpa and check that it's running:
```
sudo apt update
sudo apt install tftpd-hpa
sudo systemctl status tftpd-hpa
```
Configure TFTP server:
```
code /etc/default/tftpd-hpa
```
Modify the configuration below to your liking, but leave the username as is.
"--create" allows uploads to the tftp server
"TFTP_DIRECTORY" is where the hosted files live, and your petalinux configuration needs to point to it.
```
TFTP_USERNAME="tftp"
TFTP_DIRECTORY="/home/b/srv/tftpboot"
TFTP_ADDRESS=":69"
TFTP_OPTIONS="--secure --create"
```
Restart the server for the new settings to take effect
```
sudo systemctl restart tftpd-hpa
```
## Configure U-boot to boot via tftp
You will need to get a u-boot binary with working ethernet onto the target. JTAG or SD card could be used to do this.
The environment variables `serverip` and `ipaddr` need to be set in u-boot. You could hard-code these in your petalinux configuration before building, or you can modify them in u-boot real-time and then do a `saveenv` to save them in flash on the board. This assumed you have flash to save to. UG1144 outlines this in their TFTP section.
## Configure petalinux to output images to your tftp server directory
In the petalinux-confing menu, go to "Image Packaging Configuration"
Select "Copy final images to tftpboot"
Set tftp boot directory to the your TFTP_DIRECTORY
Save and build
# Setting up NFS
## Install and configure NFS Server
Install nfs
```
sudo apt update
sudo apt install nfs-kernel-server
```
Make a directory to host the file system
```
mkdir /home/b/srv/nfs/zcu111_lx2
```
Add this line (but your directory and IP) to /etc/exports:
```
/home/b/srv/nfs 10.99.99.0/24(rw,no_root_squash,no_all_squash,crossmnt)
```
Restart nfs
```
sudo systemctl restart nfs-kernel-server
```
## Configure petalinux for NFS root
Add the following to system-user.dtsi:
```
chosen {
bootargs = " earlycon console=ttyPS0,115200 clk_ignore_unused root=/dev/nfs rootfstype=nfs nfsroot=10.99.99.100:/home/b/srv/nfs/rootfs,tcp,nfsvers=3 ip=dhcp rw";

};
```
Then you need to modify the output config /images/linux/pxelinux.cfg/default and remove the INITRD call.
Petalinux seems to have configuration for NFS root, but it doesn't seem to do anything.


## Loading the capture pipeline

```bash
# On the KV260:
fpgautil -o /home/root/mocap-pipeline-overlay.dtbo -b /home/root/kv260_ov9281_proj.bit.bin

# Or use the helper script (does load → reset → reload as a workaround):
/home/root/apply_camera_overlay.sh
```

After loading, verify with `media-ctl -p` and `v4l2-ctl --list-devices`.

## Directory structure

Only `project-spec/` is committed. `build/`, `images/`, `components/` are all generated.

```
project-spec/meta-user/
├── conf/petalinuxbsp.conf              # BSP config (Kria KV260 carrier board)
├── recipes-bsp/
│   ├── device-tree/                    # Device tree overlay + base DT
│   ├── u-boot/                         # U-boot env, Ethernet PHY, UBIFS patch
│   └── (meta-xilinx-tools/)            # U-boot device tree (GEM3 Ethernet)
├── recipes-kernel/linux/linux-xlnx/    # Kernel patches + config fragments
├── recipes-apps/                       # Userspace applications
│   ├── mocap-sanity/                   # Single-frame V4L2 capture tool
│   ├── mocap-perf/                     # V4L2 FPS benchmark
│   ├── mocap-server/                   # TCP streaming server
│   ├── camera-fpga-files/              # Installs bitstream + overlay + loader script
│   ├── hello/                          # Default PetaLinux template
│   └── rootfs-bashrc/                  # Shell config for root + petalinux users
└── recipes-libs/
    └── mocap-common/                   # Shared header-only library
```

## Device tree

### mocap-pipeline-overlay.dts

The capture pipeline is a **runtime overlay** (`/plugin/`), not a base-tree edit. It `/include/`s the auto-generated `pl.dtsi` so all PL labels resolve at compile time.

Contents:
- **sensor_xclk**: dummy `fixed-clock` at 24 MHz — the OV9281 module self-clocks, but `ov9282.c` requires a clock provider
- **vcap_csi**: `xlnx,video` capture node, DMA via `axi_vdma_0` S2MM channel 1
- **OV9281 sensor** (`sensor@60`): I2C mux channel 2 of TCA9546A at 0x74 on `axi_iic_0`. Compatible `"ovti,ov9281"`, 2 data lanes, link frequency 400 MHz
- **CSI-2 RX endpoint wiring**: sensor → CSI sink, CSI source → VDMA (bypasses ISP wrapper in the media graph)
- **ISP math wrapper**: `compatible = "generic-uio"` for userspace histogram access via UIO

### system-user.dtsi

Base-tree overrides only: sets bootargs for NFS root. Cannot reference PL labels — those belong in the overlay.

## Kernel patches

| File | Description |
|------|-------------|
| `0001-xilinx-csi2rxss-add-Y8_1X8-mono-RAW8-mbus-code.patch` | Adds `MEDIA_BUS_FMT_Y8_1X8` to the CSI-2 RX driver's RAW8 LUT. Without this, the driver coerces mono Y8 to Bayer8 and `STREAMON` fails with `-EPIPE`. |
| `user_2026-05-14-*.cfg` | Enables `VIDEO_OV9282`, `XILINX_VDMATEST`, `VIDEO_ADV_DEBUG` |
| `user_2026-05-26-01-47-00.cfg` | Enables `OF_FPGA_REGION` (required for PL clock/device instantiation) |
| `user_2026-05-26-01-29-00.cfg` | Enables `FPGA_BRIDGE`, `FPGA_REGION` |

## U-boot configuration

- **platform-top.h**: sets `serverip=10.0.0.70` for TFTP
- **system-user.dtsi** (u-boot variant): enables GEM3 Ethernet with TI DP83867 PHY at address 1, RGMII-ID mode, reset on GPIO 38
- **0001-ubifs-distroboot-support.patch**: adds UBIFS boot path to QSPI boot command

## Applications

### mocap-sanity

Single-frame V4L2 capture tool. Walks the media controller topology, resolves sensor + CSI-RX subdevs, sets mbus formats on all pads, captures one frame. Outputs PNG (via `stb_image_write.h`) or raw bytes.

```bash
mocap-sanity                           # default: 1280x800, saves ./image_capture.png
mocap-sanity --mode 640x400 -o out.raw # lower resolution, raw output
mocap-sanity --skip-setup              # only touch video node (pads already configured)
```

### mocap-perf

V4L2 FPS benchmark. Streams frames with zero per-frame work (immediate DQBUF/QBUF). Reports per-second FPS, average FPS, inter-frame jitter, and CPU idle percentage.

```bash
mocap-perf                     # default mode, minimize VBLANK for max FPS
mocap-perf --mode 1280x720     # specific resolution
mocap-perf --fps 60            # target frame rate
```

### mocap-server

TCP streaming server for the capture pipeline. A capture thread dequeues V4L2 frames into a 2-deep ring buffer; the main thread streams frames to a single TCP client on port 5001 using a custom binary protocol (MCAP stream header + FRAM per-frame headers).

```bash
mocap-server                           # default: 1280x800, port 5001
mocap-server --ae                      # enable auto-exposure control loop
mocap-server --ae --isp                # AE with hardware ISP histogram (zero CPU cost)
mocap-server --mode 640x400 --fps 120  # lower res, higher FPS
```

Connect from the host with `python helper_scripts/stream_client.py 10.0.0.100 5001`.

### camera-fpga-files

Data recipe that installs the FPGA bitstream (`.bit.bin`), device tree overlay (`.dtbo`), and `apply_camera_overlay.sh` loader script into `/home/root`.

## Shared library: mocap-common

Header-only library installed to `${includedir}/mocap/` in the sysroot. All three apps depend on it.

| Header | Description |
|--------|-------------|
| `ov9281_pipeline.hpp` | Media controller topology walker, subdev format setter, VBLANK/FPS control, sensor mode table (1280x800, 1280x720, 640x400) |
| `auto_exposure.hpp` | Closed-loop AE controller (exposure → gain → black level). Can use SW subsampled histogram or HW ISP histogram. |
| `isp_stats.hpp` | UIO-based ISP histogram driver. Auto-discovers UIO device by scanning `/sys/class/uio/*/name`. Provides histogram read, avg brightness, HW FPS measurement. |
| `argparse.hpp` | Vendored p-ranav/argparse v2.x (C++17 argument parser) |
| `stb_image_write.h` | Vendored stb v1.16 (single-header PNG/BMP writer) |

**Note:** `isp_regs.h` is listed in the recipe's `SRC_URI` but is missing from `files/`. This will cause a build failure until the file is provided (generate from `vivado/kv260_ov9281/src/hdl/isp/rdl_out/sw/isp_regs.h`).

## Source control

**Commit:** `project-spec/`, `.xsa`, `Makefile`

**Ignore:** `build/`, `images/`, `components/`, `project-spec/hw-description/` (all regenerated)
