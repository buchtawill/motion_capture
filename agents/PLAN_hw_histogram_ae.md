# Use HW histogram for auto-exposure instead of software

## Context

The auto-exposure loop in `auto_exposure.hpp` currently computes a subsampled software histogram (every 4th pixel in both dimensions) on every Nth frame to derive mean brightness and 2nd percentile. This works but has CPU cost that scales with frame rate — at 340 fps with 640×400, it's ~5.4M pixel reads/s plus histogram bookkeeping on the A53.

The FPGA PL already has a 256-bin histogram engine (`isp_math_wrapper` at 0xA0011000) snooping the AXI-Stream pipeline at 200 MHz. It computes exact full-frame histograms and a PIXEL_SUM register — all at zero CPU cost. The AE loop should use it.

## Why the HW histogram is better

| Aspect | Software (current) | Hardware (proposed) |
|---|---|---|
| CPU cost | ~16K pixel reads + histogram per update | 1 register write + 1-257 register reads |
| Accuracy | 1/16 subsample | Every pixel counted |
| Mean brightness | Computed from subsampled sum | Single PIXEL_SUM register read |
| Percentile | From subsampled histogram | From exact 256-bin histogram |
| Scales with fps | More CPU at higher fps | Constant (PL fabric) |

At 340 fps with AE interval=15, we'd replace ~368 software histograms/s with ~23 register read bursts/s.

## Completed steps

### 1. Hardware interrupt
- Added `frame_done_irq_o` output to `isp_math_wrapper.v` with `X_INTERFACE_INFO` / `X_INTERFACE_PARAMETER` attributes for Vivado to recognize it as an interrupt
- Wired to `xlconcat_0/In3` → `pl_ps_irq0` in the block design (`system.tcl`)
- Requires Vivado rebuild + XSA re-export + `make config` + `petalinux-build -c device-tree` for `pl.dtsi` to pick up the `interrupts` property

### 2. Device tree overlay
- Added `generic-uio` compatible to `&isp_math_wrapper_0` in `mocap-pipeline-overlay.dts`
- After overlay load, the kernel creates `/dev/uioN`; verify with `ls /dev/uio*` and `cat /sys/class/uio/uio*/name`

### 3. Shared register header: `mocap-common/files/isp_regs.h`
- Copied from `vivado/.../rdl_out/sw/isp_regs.h` (generated PeakRDL header)
- Standalone C, `isp_regs_t` struct + bitmask defines, no Xilinx dependencies
- Shared source-of-truth between bare-metal (Vitis) and Linux (PetaLinux)

### 4. UIO driver: `mocap-common/files/isp_stats.hpp`
- Header-only `IspStats` class mirroring the bare-metal API
- `init(uio_path)` — opens `/dev/uioN`, mmaps register region
- `find_uio_device()` — scans `/sys/class/uio/uio*/name` for "isp_math_wrapper"
- `wait_histogram_valid()` — uses UIO interrupt (blocking `read()` with `poll()` timeout), falls back to register polling if no interrupt
- `capture_histogram()` — one-shot: start + wait + dump 256 bins + pixel_sum
- `snapshot()` / `compute_fps()` — frame/cycle counter support

### 5. Recipe updated: `mocap-common.bb`
- Added `isp_regs.h` and `isp_stats.hpp` to SRC_URI and do_install

### 6. Integrate with `auto_exposure.hpp`
- Refactored both `IspStats` and `AutoExposure` to RAII (factory methods returning `unique_ptr`, destructor cleanup, no `init()`/`close()`)
- `AutoExposure::create(sensor_path, cfg, isp*)` takes optional non-owning `IspStats*`
- `update()` checks `is_hist_data_valid()`: reads PIXEL_SUM + 256-bin histogram from HW, computes mean + 2nd percentile, kicks `start_histogram()` for next frame
- When `IspStats*` is null, falls back to the SW subsampled histogram path
- `create()` kicks the first `start_histogram()` so the capture loop finds valid data

### 7. Integrate in `mocap-server.cpp`
- Added `--isp` flag (auto-discovers UIO device via `IspStats::discover()`)
- After pipeline setup: creates `unique_ptr<IspStats>`, sets resolution, passes `isp.get()` to `AutoExposure::create()`
- Capture loop unchanged — `ae->update()` transparently uses HW or SW path

## Verification

1. After Vivado rebuild + XSA export: confirm `pl.dtsi` has `interrupts` on `isp_math_wrapper_0`
2. After PetaLinux build + overlay load: `ls /dev/uio*`, `cat /sys/class/uio/uio*/name`
3. Minimal test: open UIO, read HRES/VRES (should be 1280/800), `capture_histogram()`, print PIXEL_SUM
4. Run `mocap-server --ae --isp` — AE log lines show HW-derived mean
5. Compare PIXEL_SUM / (W×H) with software mean — should match
6. At high fps (`--mode 640x400 --fps max --ae`), compare CPU idle% with and without `--isp`
