# Session: ISP Histogram Interrupt + Multi-Resolution Tests

**Date:** 2026-05-31

## Summary

Added a latched frame-completion interrupt to the ISP module and expanded
testbench coverage to all three OV9281 resolutions (1280x800, 640x400, 1280x720).

## Changes Made

### Interrupt (`frame_done_irq_o`)

The interrupt is sourced from the measurement FSM in `isp_math_top.sv`, not from
the histogram module. It latches high on the FLUSH→IDLE transition (same condition
as `STATUS.HIST_DATA_VALID`) and is cleared by software via `CTRL.IRQ_CLEAR`
(bit 6, singlepulse) or `CTRL.RESET`.

**Files modified:**

- `src/hdl/isp/isp_math_top.sv` — added `frame_done_irq_o` output port, IRQ
  latch driven by `measurement_done` (FLUSH→IDLE), `irq_clear` wire from CTRL
- `src/hdl/isp/isp_math_wrapper.v` — wired `frame_done_irq_o` through to top level
- `src/hdl/isp/isp_regs.rdl` — added `CTRL.IRQ_CLEAR` (bit 6, singlepulse) and
  `STATUS.FRAME_DONE_IRQ` (bit 3, hw=w/sw=r)
- `src/hdl/isp/isp_regs_defines.svh` — added corresponding defines
- `src/hdl/isp/rdl_out/` — regenerated RTL, C header, HTML docs via `make rtl`
  and `make header`

Initially the interrupt was a pulse generated inside `isp_histogram.sv` on the
rising edge of `ram_data_o_vld`. This fired spuriously during post-reset scrub
(when the histogram goes idle, `ram_data_o_vld` rises). The fix was to remove
the interrupt from `isp_histogram.sv` entirely and source it from the FSM in
`isp_math_top.sv` using the FLUSH→IDLE transition, which only occurs after a
real measurement.

### Multi-Resolution Tests

#### `tb_isp_histogram.sv`

- Extracted `stream_full_image_zeros(w, h)` task (scrub + stream + check)
- Test 6 now calls it for 1280x800, 640x400, and 1280x720
- Removed all interrupt plumbing (port, checker, test 9) since interrupt lives
  at the wrapper level
- Timeout watchdog changed from `$finish` to `$fatal`

#### `tb_isp_math.sv`

- Replaced fixed `FRAME_HRES`/`FRAME_VRES` localparams with module-scope variables
- Added `run_frame_test(hres, vres, seed)` task: resets DUT, programs resolution,
  streams a frame, verifies bins + pixel sum + IRQ latch + IRQ clear
- Phase 2 now runs `run_frame_test` at all three resolutions
- Added Phase 3: dedicated test for clearing IRQ via `CTRL.RESET` (alternate path)
- Connected `frame_done_irq_o` to DUT instance
- Updated Phase 1 STATUS check to include `FRAME_DONE_IRQ` bit

### Build Infrastructure

- `src/sim/Makefile` — fixed `SRCS_tb_frame_rate` (was referencing nonexistent
  `isp_top.sv` and `isp_wrapper.v`), added `INCDIRS_tb_frame_rate`
- `vivado/kv260_ov9281/Makefile` — added `make gui` target to open Vivado GUI
- Top-level `Makefile` — rewritten for KV260-only targets:
  `vivado` (clean+bitstream+bit-bin), `vitis` (clean+build_all),
  `linux` (clean+all), `hw` (vivado+vitis), `full` (all three)

### Documentation

- `src/hdl/isp/README.md` — new file documenting RTL hierarchy, register
  generation flow, and all four testbenches with their roles

## Key Design Decision

The interrupt is a **latched level** (not a pulse) because the target consumer
is Linux UIO with blocking `read()`. A pulse risks being missed if the CPU is
busy. Software clears the latch explicitly after reading the histogram data.
