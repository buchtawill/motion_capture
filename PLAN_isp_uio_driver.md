# ISP histogram UIO driver for Linux

## Context

The ISP histogram module (`isp_math_wrapper` at 0xA0011000) is currently accessed via raw `/dev/mem` mmap. This works but requires root and has no device-model integration. A `generic-uio` binding gives us a proper `/dev/uioN` node with per-device mmap and (future) interrupt support, plus it doesn't need root if the device permissions are set.

The bare-metal driver (`vitis/.../isp_stats.cpp`) is ~265 lines, but half is xil_printf diagnostics. The core register logic is ~100 lines of volatile pointer dereferences — same pattern on Linux, just sourced from UIO mmap instead of a physical address cast. **Don't `#ifdef` the Vitis code.** Share `isp_regs.h` (the generated register struct, already standalone C), and write a clean Linux-native class.

## What to share vs. rewrite

- **Share:** `isp_regs.h` — the generated `isp_regs_t` struct and bitmask defines. Copy into `mocap-common/files/`. It has zero Xilinx dependencies.
- **Rewrite:** The driver class. New header-only `isp_stats.hpp` in `mocap-common/files/` for Linux. Mirrors the bare-metal `IspStats` API but uses UIO mmap and standard C++/POSIX types.

## Implementation

### 1. Device tree: add generic-uio compatible

In `mocap-pipeline-overlay.dts`, override the ISP node's compatible to add `generic-uio`:

```dts
&isp_math_wrapper_0 {
    compatible = "generic-uio", "xlnx,isp-math-wrapper-1.0";
    status = "okay";
};
```

This makes the kernel's UIO platform driver claim the device and create `/dev/uioN`. The `reg` property from `pl.dtsi` provides the mmap region automatically. No interrupt (the wrapper has no IRQ output), so UIO operates in mmap-only mode.

Requires `CONFIG_UIO_PDRV_GENIRQ=m` (already in the kernel config per CLAUDE.md).

### 2. Copy `isp_regs.h` into mocap-common

Source: `vivado/kv260_ov9281/src/hdl/isp/rdl_out/sw/isp_regs.h`
Dest: `recipes-libs/mocap-common/files/isp_regs.h`

This is the generated PeakRDL header with the `isp_regs_t` struct and all `ISP_REGS__*` bitmask defines. Standalone C, no edits needed. Stage it into the sysroot as `<mocap/isp_regs.h>`.

### 3. New shared header: `mocap-common/files/isp_stats.hpp`

Header-only `IspStats` class for Linux. API mirrors the bare-metal version:

```
class IspStats {
    init(uio_device_path)     // open /dev/uioN, mmap region 0 → isp_regs_t*
    close()                   // munmap, close fd

    // Resolution
    set_resolution(hres, vres)
    get_resolution() → {hres, vres}

    // Status
    is_ready() → bool
    is_hist_data_valid() → bool
    is_hist_fifo_err() → bool

    // Measurement
    start_histogram() → bool
    poll_histogram_valid(max_us) → bool   // usleep-based, not busy-spin
    dump_histogram(uint32_t bins[256]) → bool
    read_pixel_sum() → uint32_t
    capture_histogram(bins, &pixel_sum) → bool   // start + poll + dump

    // Counters
    snapshot() → {cycle_cnt, frame_cnt}
    compute_fps(snap1, snap2, clock_hz) → double

    // Convenience
    compute_avg_brightness() → double
    sw_reset()
};
```

Differences from the bare-metal version:
- `init()` opens `/dev/uioN` and mmaps the register region (UIO region 0), stores `volatile isp_regs_t*`
- `poll_histogram_valid()` uses `usleep()` between polls (not bare-metal busy-spin), with a timeout in microseconds
- Returns `bool` instead of `XStatus`
- No `print_status()` / `print_histogram()` (diagnostic methods that use xil_printf/outbyte — not needed; callers can read the data and format however they want)
- Internal `pulse_ctrl()` helper is identical (same register logic)

The register access code (`start_histogram`, `dump_histogram`, `read_pixel_sum`, etc.) is essentially copied from the bare-metal `isp_stats.cpp` with type changes — the `volatile isp_regs_t*` dereference pattern is identical.

### 4. UIO device discovery

The class needs to find which `/dev/uioN` corresponds to the ISP. Two approaches:

**Option A — pass the path explicitly:** `isp.init("/dev/uio0")`. Simple, but fragile if UIO device numbering changes.

**Option B — scan sysfs by name:** Walk `/sys/class/uio/uio*/name`, match against `"isp_math_wrapper"` (the DT node name that UIO uses). Return the matching `/dev/uioN` path. More robust.

Recommend Option B with Option A as a CLI override (`--isp-dev /dev/uio0`).

### 5. Update `mocap-common.bb`

Add `isp_regs.h` and `isp_stats.hpp` to `SRC_URI` and `do_install`.

### 6. Integrate with `auto_exposure.hpp` (later)

Per the HW histogram plan (`PLAN_hw_histogram_ae.md`), `AutoExposure::update()` will accept an optional `IspStats*`. When non-null, it reads PIXEL_SUM and histogram bins from hardware instead of computing a software histogram. The control math is unchanged.

## Files

| Action | File |
|---|---|
| Copy | `vivado/.../rdl_out/sw/isp_regs.h` → `recipes-libs/mocap-common/files/isp_regs.h` |
| New | `recipes-libs/mocap-common/files/isp_stats.hpp` |
| Edit | `recipes-libs/mocap-common/mocap-common.bb` (add to SRC_URI + do_install) |
| Edit | `recipes-bsp/device-tree/files/mocap-pipeline-overlay.dts` (add generic-uio compatible) |

## Verification

1. After PetaLinux build + overlay load: `ls /dev/uio*` — confirm a UIO device appears
2. `cat /sys/class/uio/uio*/name` — confirm it shows `isp_math_wrapper`
3. Write a minimal test (or extend mocap-sanity) that opens the UIO device, reads HRES/VRES (should be 1280/800), runs `capture_histogram()`, prints PIXEL_SUM and a few bin values
4. Compare PIXEL_SUM / (1280×800) with the software-computed mean from auto_exposure — they should match

## Notes

- **No interrupt:** The ISP wrapper has no IRQ output port. UIO will work in mmap-only mode. If an interrupt is added later (e.g., HIST_DATA_VALID edge → GIC SPI), the DT node gets an `interrupts` property and UIO `read()` blocks until it fires — zero code changes to the driver, just replace `poll_histogram_valid()` busy-poll with a blocking `read(uio_fd)`.
- **The Vitis bare-metal code is not modified.** It continues to work independently for bare-metal builds. The two versions share only `isp_regs.h`.
