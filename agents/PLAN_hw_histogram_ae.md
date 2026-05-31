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

## Implementation

### 1. New shared header: `mocap-common/files/isp_histogram.hpp`

Header-only class `IspHistogram` that wraps `/dev/mem` mmap access to the ISP registers:

- `init(phys_addr, hres, vres)` — mmap one page at the ISP base address via `/dev/mem`, cast to `isp_regs_t*`, set HRES/VRES registers
- `start()` — pulse CTRL.HISTOGRAM_START
- `is_valid()` — read STATUS.HIST_DATA_VALID
- `read_pixel_sum()` — return PIXEL_SUM register
- `read_histogram(uint32_t bins[256])` — set HIST_ADDR=0, read 256 × HIST_DATA with auto-increment
- `close()` — munmap

Reuse the generated `isp_regs_t` struct and bitmask defines from `vivado/.../rdl_out/sw/isp_regs.h`. Copy it into `mocap-common/files/` as `isp_regs.h` (it's standalone, no Xilinx dependencies).

### 2. Modify `auto_exposure.hpp`

Add an optional `IspHistogram*` parameter. When non-null, `update()` skips the software `compute_stats()` entirely and instead:

1. Read `is_valid()` — if the previous measurement completed, read PIXEL_SUM (and optionally 256 bins for percentile)
2. Kick `start()` for the next measurement
3. Feed the hardware-derived mean and percentile into the existing exposure/gain/black-level control math (unchanged)

When the `IspHistogram*` is null, fall back to the current software path. This keeps the header usable by apps that don't have ISP access.

### 3. Integrate in `mocap-server.cpp`

- Add `--isp-addr` CLI flag (default `0xA0011000`, `0` to disable)
- After pipeline setup, if isp-addr != 0: create `IspHistogram`, call `init()`, pass pointer to `AutoExposure`
- The capture loop is unchanged — `ae->update()` now uses HW data internally

### 4. Update `mocap-common.bb`

Add `isp_regs.h` and `isp_histogram.hpp` to SRC_URI and do_install.

## Files to modify

- **New:** `recipes-libs/mocap-common/files/isp_regs.h` (copy from `vivado/.../rdl_out/sw/isp_regs.h`)
- **New:** `recipes-libs/mocap-common/files/isp_histogram.hpp` (mmap wrapper)
- **Edit:** `recipes-libs/mocap-common/files/auto_exposure.hpp` (add HW histogram path)
- **Edit:** `recipes-libs/mocap-common/mocap-common.bb` (add new files)
- **Edit:** `recipes-apps/mocap-server/files/mocap-server.cpp` (add `--isp-addr`, init HW histogram)

## Verification

1. Run `mocap-server --ae --isp-addr 0xA0011000` on the KV260 — AE log lines should show mean/dark values derived from hardware (confirm PIXEL_SUM / (W×H) matches expected brightness)
2. Run without `--isp-addr` (or `--isp-addr 0`) — should fall back to software histogram identically to current behavior
3. At high fps (`--mode 640x400 --fps max --ae`), compare CPU idle% with and without HW histogram to confirm reduced CPU load

## Prerequisite

Validate the software-only auto-exposure loop first (current `auto_exposure.hpp`). Once confirmed working on-target, implement the HW histogram integration described above.
