# kv260_ov9281 — RTL Overview

KV260 + OV9281 global-shutter camera capture pipeline.  
Hardware: Zynq UltraScale+ (xck26), Vivado 2024.1.

---

## System-Level Video Pipeline

```
OV9281 (MIPI CSI-2, 2-lane)
        │
        ▼
  MIPI D-PHY RX  ──►  MIPI CSI-2 RX Subsystem
                              │  AXI-Stream (32-bit, 4 px/beat)
                              ▼
                    ┌─────────────────────┐
                    │  isp_math_wrapper   │  ← pure snoop (no backpressure)
                    │  (isp_math_top.sv)  │    • 256-bin histogram
                    │                     │    • pixel sum
                    │                     │    • cycle + frame counters
                    └─────────────────────┘
                              │  AXI-Stream (passthrough)
                              ▼
                       AXI VDMA (write)  ──►  DDR4 frame buffer
                                                      │
DVI output  ◄──  rgb2dvi  ◄──  Bayer→RGB  ◄──  AXI VDMA (read)
```

The ISP blocks are **pure snoops**: they observe the AXI-Stream between the
MIPI CSI-2 RX subsystem and the AXI VDMA without adding any latency or
backpressure. The datapath is wired straight through at the top level.

This module is primarily designed for **8-bit pixel values**. Frame counters
and cycle counters are **enabled by default**; histogram collection is
**disabled by default** and must be explicitly triggered via
`CTRL.HISTOGRAM_START`. Average pixel brightness and frame rate are computed
in software by snapshotting the frame-count and cycle-count registers at
known time deltas.

---

## RTL Hierarchy

```
isp_math_wrapper.v              (Vivado IP Integrator shell — Verilog 2001)
  └── isp_wrapper               (isp_math_top.sv — top ISP logic)
        ├── isp_regs            (isp_regs.sv — PeakRDL-generated AXI4-Lite slave)
        └── isp_histogram       (isp_histogram.sv — 256-bin histogram)
              └── stream_fifo   (stream_fifo.sv — AXI-Stream FIFO, depth=16)

frame_rate_counter.sv           (standalone FPS measurement block, separate from above)
```

Supporting files (not RTL modules):

| File | Role |
|---|---|
| `isp_regs.rdl` | PeakRDL source — authoritative register-map spec |
| `rdl_out/rtl/isp_regs_pkg.sv` | PeakRDL-generated SV package (hwif structs) |
| `rdl_out/rtl/isp_regs.sv` | PeakRDL-generated AXI4-Lite register block |
| `isp_regs_defines.svh` | `\`define` macros for register addresses / bit fields |

---

## Block Descriptions

### `isp_math_wrapper.v`

Plain Verilog-2001 shell required because Vivado IP Integrator cannot
directly instantiate a SystemVerilog module as a block-design IP. Contains
**no logic whatsoever** — every port is wired directly through to the
`isp_wrapper` instance inside. This is the module Vivado sees as the IP
boundary.

---

### `isp_math_top.sv` — module `isp_wrapper`

The central ISP logic block. It connects all the sub-blocks and implements
the measurement FSM.

**Datapath:** The AXI-Stream slave and master ports are wired together with
pure `assign` statements — `tdata`, `tuser`, `tlast`, `tvalid`, and the
backpressure `tready` are all passthrough wires. The block observes traffic
but never holds it. `TUSER` is asserted by the MIPI CSI-2 RX Subsystem
coincident with the first valid pixel beat of each frame.

**Measurement FSM:**

```
RTL reset / CTRL.RESET
        │
        ▼
POST_RESET_SCRUB  ──►  WAIT_POST_RESET_SCRUB  ──►  IDLE
                                                      │
                                           CTRL.HISTOGRAM_START
                                           (HRES≠0 && VRES≠0)
                                                      │
                                                      ▼
                                               START_SCRUB
                                                      │
                                                      ▼
                                               WAIT_SCRUB
                                                      │
                                               hist_rdy rises
                                                      │
                                                      ▼
                                               WAIT_TUSER
                                                      │
                                           first TUSER beat accepted
                                                      │
                                                      ▼
                                                  MEASURE  ──── (HRES×VRES/4 beats)
                                                      │
                                                      ▼
                                                   FLUSH  ──── (50 cycles, drains hist FIFO)
                                                      │
                                                      ▼
                                              IDLE + HIST_DATA_VALID=1
```

**Counters maintained by this module (published to `isp_regs` each cycle):**

| Counter | Description |
|---|---|
| `cycle_cnt` (64-bit) | Free-running clock counter. Clearable via `CTRL.CYCLE_CNT_RESET`. |
| `cycle_snap` (64-bit) | Latched coherent copy of `cycle_cnt` on `CTRL.SNAPSHOT`. |
| `frame_cnt` (32-bit) | Increments once per accepted TUSER beat (= start of frame). |
| `frame_snap` (32-bit) | Latched copy of `frame_cnt` on `CTRL.SNAPSHOT`. |
| `pixel_sum` (32-bit) | Sum of all 8-bit pixel values during the last measurement. |
| `beat_cnt` (32-bit) | Internal; counts beats during MEASURE to know when to stop. |

**HISTOGRAM_START guard:** The FSM only begins a measurement when HRES and
VRES are both non-zero, preventing a divide-by-zero in the beat target
(`HRES × VRES / 4`). The beat target also gates exactly how many beats
reach the histogram FIFO even if new frames arrive before the FIFO drains.
Writing `HISTOGRAM_START` also immediately de-asserts `HIST_DATA_VALID`; it
is ignored if `STATUS.READY = 0` (scrub in progress or measurement running).

**HIST_FIFO_ERR:** If `isp_histogram` asserts `err_o` (FIFO overflow), this
block latches the error sticky into `STATUS.HIST_FIFO_ERR`. Clears only on
RTL reset or `CTRL.RESET`.

---

### `isp_histogram.sv`

Accumulates a 256-bin histogram of 8-bit pixel values. Pixels arrive 4 per
AXI-Stream beat (32-bit `tdata`), and the module processes one pixel per
clock cycle on the read side.

**Internal structure:**

```
AXI-Stream snoop
     │  (fifo_s_valid = pix_data_vld & pix_data_rdy & hist_en)
     ▼
stream_fifo (depth=16, 32-bit wide)   ← absorbs bursts while RAM is busy
     │
     ▼  (1 pixel/clock)
  S_ACTIVE FSM
     │  reads pixel byte, looks up hist_mem[pixel], increments, writes back
     ▼
hist_mem[0:255]  (256 × 20-bit BRAM)
```

**States:**

| State | Description |
|---|---|
| `S_IDLE` | Waiting. Exposes `ram_data_o = hist_mem[ram_addr_i]` (frontdoor read port). |
| `S_ACTIVE` | Popped a beat from FIFO; processing each of the 4 bytes in turn (one per cycle). |
| `S_SCRUB` | Zeroing all 256 bins sequentially (one per cycle, 256 cycles total). |

**Write hazard handling:** If two consecutive pixels have the same value,
the read-modify-write pipeline would read a stale value for the second
write. The module detects this (`hazard = pixel_d == pixel_q`) and forwards
the write register value (`ram_wr_val_q + 1`) instead of the RAM read value.

**Scrub:** Triggered by a one-cycle `ram_scrub_i` pulse from the FSM in
`isp_math_top`. The module takes 256 cycles to zero all bins, then asserts
`hist_rdy_o = 1`. The parent FSM waits for `hist_rdy` to go low then high
again to confirm the scrub actually ran (not a false-ready on entry).

**Frontdoor read port:** When `hist_en_i = 0`, `ram_data_o` reflects
`hist_mem[ram_addr_i]` with one cycle of registered latency. This is wired
through `isp_regs` so the ARM can read histogram bins via `HIST_ADDR` /
`HIST_DATA`.

**Overflow error:** If a pixel beat arrives while the FIFO is full, `err_o`
pulses high. The parent block latches this sticky.

---

### `stream_fifo.sv`

Generic AXI-Stream FIFO. Parameters: `DATA_WIDTH`, `DEPTH` (must be power
of 2). Used by `isp_histogram` with `DATA_WIDTH=32`, `DEPTH=16`.

Standard pointer-based implementation: one extra bit on the count distinguishes
full from empty. `s_ready` deasserts when `count == DEPTH`; `m_valid`
deasserts when `count == 0`. Supports simultaneous push and pop in the same
cycle.

---

### `isp_regs.sv` / `isp_regs_pkg.sv` (PeakRDL-generated)

AXI4-Lite register block generated from `isp_regs.rdl` by
[PeakRDL-regblock](https://github.com/SystemRDL/PeakRDL-regblock). Do not
edit these files by hand — regenerate from the RDL source.

`isp_regs_pkg.sv` defines the `hwif_in` / `hwif_out` packed structs that
form the hardware interface between the register block and `isp_math_top`.
Hardware-writable fields use `.next` (written combinationally every cycle by
`isp_math_top`). Hardware-readable fields use `.value` (driven by the
register block to RTL logic).

---

### `isp_regs_defines.svh`

Header file (`\`include`d by `isp_math_top.sv` and testbenches) that
mirrors the register address offsets and bit-field positions from
`isp_regs.rdl` as `` `define `` macros. Kept in sync with the RDL by hand
— if you add a register to the RDL, add the corresponding define here.

---

### `frame_rate_counter.sv`

An older, simpler standalone FPS measurement module. Also a pure AXI-Stream
snoop. Measures frame rate by counting clock cycles across exactly 100
frames (TUSER strobes) and latching the result.

**States:** `S_IDLE → S_WAIT_SOF → S_COUNTING → S_DONE`

**Register map (3 registers, AXI4-Lite):**

| Offset | Name | Description |
|---|---|---|
| `0x00` | CTRL | `[0]` enable, `[1]` sw_reset (write-only, always reads 0) |
| `0x04` | STATUS | `[0]` idle, `[1]` busy, `[2]` done |
| `0x08` | CYCLE_COUNT | 32-bit elapsed cycles across 100 frames |

This block pre-dates `isp_math_top` and has a simpler hand-written register
interface with no PeakRDL involvement. It is used in simulation
(`tb_frame_rate`) but may or may not be wired into the block design
depending on the current project state.

---

## Register Map (`isp_math_top` / `isp_regs`)

Base address set by the Vivado block design. 11-bit address space.

| Offset | Name | Access | Reset | Description |
|---|---|---|---|---|
| `0x000` | CTRL | W | `0x8` | Control. Write-pulse bits auto-clear. `HIST_ADDR_AUTOINC` [3] sticky R/W (default 1). |
| `0x004` | STATUS | RO | `0x0` | `[0]` READY, `[1]` HIST_DATA_VALID, `[2]` HIST_FIFO_ERR (sticky). |
| `0x008` | HRES | R/W | `1280` | Horizontal resolution in pixels. Preserved across SW reset. |
| `0x00C` | VRES | R/W | `800` | Vertical resolution in lines. Preserved across SW reset. |
| `0x010` | CYCLE_CNT_LO | RO | `0` | Free-running cycle counter, low 32 bits. |
| `0x014` | CYCLE_CNT_HI | RO | `0` | Free-running cycle counter, high 32 bits. |
| `0x018` | CYCLE_SNAP_LO | RO | `0` | Cycle counter snapshot, low 32 bits (latched by CTRL.SNAPSHOT). |
| `0x01C` | CYCLE_SNAP_HI | RO | `0` | Cycle counter snapshot, high 32 bits. |
| `0x020` | FRAME_CNT | RO | `0` | Frame counter (increments on TUSER & TVALID & TREADY). |
| `0x024` | FRAME_SNAP | RO | `0` | Frame counter snapshot (latched by CTRL.SNAPSHOT). |
| `0x028` | PIXEL_SUM | RO | `0` | Sum of all 8-bit pixel values from the last measurement. Redundant for a 256-bin histogram (sum equals Σ bin[i]×i), but provided as a convenience. |
| `0x02C` | HIST_ADDR | R/W | `0` | Histogram bin index (8-bit). Auto-increments on HIST_DATA read. |
| `0x030` | HIST_DATA | RO | `0` | Histogram bin count at HIST_ADDR (20-bit). |

**CTRL bit fields:**

| Bit | Name | Type | Description |
|---|---|---|---|
| 0 | RESET | W-pulse | SW reset. Clears counters, kicks scrub. Does not reset HRES/VRES. |
| 1 | HISTOGRAM_START | W-pulse | Begin measurement. Immediately de-asserts HIST_DATA_VALID and issues a RAM scrub. Ignored if STATUS.READY=0 or HRES/VRES=0. |
| 2 | SNAPSHOT | W-pulse | Latch coherent cycle+frame snapshot. |
| 3 | HIST_ADDR_AUTOINC | R/W | Auto-increment HIST_ADDR after each HIST_DATA read (default 1). |
| 4 | FRAME_CNT_RESET | W-pulse | Reset frame counter and FRAME_SNAP only. |
| 5 | CYCLE_CNT_RESET | W-pulse | Reset cycle counter (LO+HI) and its snapshot only. |

**Typical measurement sequence:**
1. Write HRES and VRES to match the camera resolution.
2. Poll `STATUS.READY` until it is 1 (post-reset scrub completes automatically).
3. Write `CTRL.HISTOGRAM_START = 1`.
4. Poll `STATUS.HIST_DATA_VALID` until it is 1.
5. Read `PIXEL_SUM` for average brightness (divide by `HRES × VRES` for mean).
6. Set `HIST_ADDR = 0`; burst-read `HIST_DATA` 256 times for the full histogram
   (auto-increment advances `HIST_ADDR` after each read).

---

## Key Design Decisions

**No datapath backpressure.** The ISP logic never stalls the camera stream.
If the histogram FIFO overflows (stream faster than the RAM can absorb),
`HIST_FIFO_ERR` latches and partial data is flagged rather than dropping
frames.

**RAM scrub before every measurement.** The histogram RAM is zeroed before
each new measurement to guarantee clean bin counts. Post-reset the scrub
also runs automatically so bins are never uninitialized.

**PeakRDL for register generation.** Registers are defined once in
`isp_regs.rdl` and the SV implementation (`isp_regs.sv`, `isp_regs_pkg.sv`)
plus C header (`rdl_out/sw/isp_regs.h`) are generated from it.
`isp_regs_defines.svh` is the manually-maintained companion for testbench
`` `define `` macros and must be kept in sync with the RDL.

**`isp_math_wrapper.v` is a Verilog-2001 shim.** Vivado IP Integrator
requires a non-SystemVerilog top for module-reference IPs. All logic lives
in the `.sv` files below it.
