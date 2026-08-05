# ISP Module

Passive AXI-Stream statistics block for the KV260 / OV9281 capture pipeline.
Snoops the MIPI CSI-2 RX to VDMA pixel stream without stalling or modifying it,
and exposes measurements over an AXI4-Lite register interface.

## RTL Hierarchy

```
isp_math_wrapper.v          Verilog-2001 shell for Vivado IP Integrator
  isp_math_top.sv            Measurement FSM, counters, interrupt latch
    isp_regs.sv               AXI4-Lite register block (PeakRDL-generated)
    isp_histogram.sv          256-bin histogram engine
      stream_fifo.sv            Ingress FIFO (absorbs burst from 4-wide stream)
```

### isp_math_wrapper.v

Plain Verilog wrapper with no logic. Exists solely because Vivado IP Integrator
module references require a Verilog-2001 top. Every port is wired 1:1 to
`isp_math_top`.

### isp_math_top.sv (`isp_wrapper`)

Owns the measurement state machine and all top-level counters:

- **Cycle counter** (64-bit, free-running) and **frame counter** (TUSER-triggered),
  each with snapshot registers for coherent readback.
- **Measurement FSM**: POST_RESET_SCRUB -> IDLE -> START_SCRUB -> WAIT_SCRUB ->
  WAIT_TUSER -> MEASURE -> FLUSH -> IDLE. Controls `hist_en` and `ram_scrub`
  to the histogram, gates `tvalid` so exactly HRES*VRES/4 beats are captured.
- **Pixel sum** accumulator (sum of all pixel byte values in the measured frame).
- **frame_done_irq_o**: Latched interrupt output. Set when `ram_data_o_vld` rises
  (histogram measurement complete). Cleared by writing `CTRL.IRQ_CLEAR` or
  `CTRL.RESET`. Readable in `STATUS.FRAME_DONE_IRQ`.

### isp_histogram.sv

Core histogram engine. Pops 32-bit beats from its internal FIFO one byte at a
time (one pixel per clock at 200 MHz), incrementing the corresponding bin in a
256-entry RAM. Handles read-after-write hazards when consecutive pixels map to
the same bin.

Key ports:
- `hist_en_i` / `ram_scrub_i` — controlled by the FSM in `isp_math_top`
- `ram_addr_i` / `ram_data_o` — frontdoor read port, active when `hist_en_i=0`
- `frame_done_irq_o` — single-cycle pulse on the rising edge of `ram_data_o_vld`

### isp_regs (PeakRDL)

AXI4-Lite slave generated from `isp_regs.rdl`. Regenerate after RDL changes:

```bash
cd src/hdl/isp && make rtl    # also: make header, make html
```

Register map is documented in `rdl_out/html/index.html` and
`isp_regs_defines.svh` (the single source of truth for addresses and bit
positions used by RTL and testbenches).

### stream_fifo.sv

Generic valid/ready FIFO parameterized by width and depth. Shared utility,
lives in the common IP dir at `ip_repo/common/stream_fifo.sv`.

## Testbenches

All testbenches live in `src/sim/` and share a common Makefile.

```bash
cd src/sim
make sim TB=<testbench>     # compile + run, produces .fst waveform
make sim_wdb TB=<testbench> # compile + run, produces .wdb for Vivado GUI
```

### Test strategy

```
                        Unit tests                Integration test
                        ----------                ----------------
stream_fifo.sv    <--  tb_stream_fifo
isp_histogram.sv  <--  tb_isp_histogram
isp_math_top.sv   <--                            tb_isp_math
isp_math_wrapper.v                                    |
                                                      +-- exercises the full
                                                          wrapper including
                                                          AXI-Lite regs, FSM,
                                                          and histogram via
                                                          frontdoor reads
```

### tb_stream_fifo

Unit test for the shared FIFO primitive. Validates fill/drain ordering, full and
empty boundaries, simultaneous push+pop at both extremes, and backpressure
mid-stream.

### tb_isp_histogram

Core functional testbench for the histogram engine in isolation. Drives pixel
beats directly into the histogram (bypassing the measurement FSM and register
block) and verifies bin counts through two independent paths:

- **Backdoor**: direct `hist_mem[]` array access (zero latency, always available)
- **Frontdoor**: hardware `ram_addr_i` / `ram_data_o` port (requires `hist_en_i=0`)

Test coverage:
1. Post-reset zero check (all 256 bins)
2. Back-to-back beats with accumulation and write-hazard
3. Distinct pixel values
4. Accumulation (repeat same beat)
5. RAM scrub
6. Full-image streams at each supported resolution (1280x800, 640x400, 1280x720)
7. RAM read-port sanity (vld gating, frontdoor after stream, frontdoor after scrub)
8. Randomized bins (100 iterations, backdoor + frontdoor verification each)
9. `frame_done_irq_o` pulse checker (concurrent monitor validates single-cycle
   pulse timing and coincidence with `ram_data_o_vld` throughout all tests)

### tb_isp_math

Integration testbench for the full `isp_math_wrapper` including the AXI-Lite
register block and measurement FSM. Exercises the module the way software would:
register reads/writes over AXI-Lite and pixel data over AXI-Stream.

**Phase 1 -- Register peek/poke**: Walks every register in the map verifying
reset defaults, write+readback, snapshot latching, HIST_ADDR autoinc on/off and
wrap, per-counter SW resets, and that SW reset preserves HRES/VRES.

**Phase 2 -- Functional single-frame test**: Triggers `HISTOGRAM_START` via
AXI-Lite, streams a full 1280x800 frame with TUSER marking frame start, waits
for `HIST_DATA_VALID`, then reads all 256 bins through the frontdoor autoinc
register interface and compares against a golden software model.

### tb_frame_rate

Testbench for the standalone `frame_rate_counter` module (legacy, separate from
the `isp_math` hierarchy). Tests FPS measurement, mid-count SW reset, and
mid-count disable.
