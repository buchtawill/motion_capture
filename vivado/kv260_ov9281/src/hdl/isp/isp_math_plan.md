# ISP math helper — implementation plan

Two-phase plan to finish `isp_math_top` and its testbench. Phase A adds the
remaining RTL (histogram instance + FSM + pixel-sum counter). Phase B extends
`tb_isp_math` with a full-frame functional test on top of the existing
per-register sanity checks.

## Key facts from `isp_histogram.sv`

- Internal FIFO enqueues only when
  `pix_data_vld_i & pix_data_rdy_i & hist_en_i` — pure snoop, no
  backpressure to the datapath.
- 4 pixels per 32-bit beat, LSB first (`beat_shift_q[7:0]` consumed first).
- `ram_scrub_i` is honored only in `S_IDLE` with `hist_en_i=0`; scrub takes
  256 cycles.
- `hist_rdy_o = (state == S_IDLE)`. After scrub or after the last beat
  drains, `hist_rdy_o` returns high.
- `ram_data_o_vld = (S_IDLE) & ~hist_en_i` — frontdoor reads only valid when
  counting is off.
- `err_o` is a sticky flop inside the histogram (FIFO overflow). OR it into
  our own sticky `HIST_FIFO_ERR`.

## Phase A — RTL (do first; the TB depends on it)

### A1. Instantiate `isp_histogram` in `isp_math_top`

- Snoop: `pix_data_i = s_axis_tdata`, `pix_data_vld_i = s_axis_tvalid`,
  `pix_data_rdy_i = m_axis_tready`.
- `hist_en_i` / `ram_scrub_i` from the FSM (A2).
- `ram_addr_i = hwif_out.HIST_ADDR.HIST_ADDR.value`;
  `hwif_in.HIST_DATA.HIST_DATA.next = ram_data_o` (20-bit width matches RDL).
- OR `err_o` into the existing `hist_fifo_err_sticky` flop (replaces the
  `hist_fifo_err_raw = 1'b0` tie).

### A2. Top-level FSM (6 states)

```
POST_RESET_SCRUB  pulse ram_scrub_i, wait hist_rdy_o rising
IDLE              READY=1; on HISTOGRAM_START && HRES!=0 && VRES!=0:
                    clear HIST_DATA_VALID, go START_SCRUB
START_SCRUB       pulse ram_scrub_i for 1 cycle, go WAIT_SCRUB
WAIT_SCRUB        wait hist_rdy_o, go WAIT_TUSER
WAIT_TUSER        wait TUSER & TVALID & m_axis_tready;
                    go MEASURE same cycle
MEASURE           hist_en_i=1; count beats; when
                    beat_count == HRES*VRES/4 go FLUSH
FLUSH             hist_en_i=0; wait hist_rdy_o;
                    set HIST_DATA_VALID, go IDLE
```

- `READY = (state == IDLE)`.
- `CTRL.RESET` forces `POST_RESET_SCRUB` and clears `HIST_DATA_VALID`.

### A3. Pixel-sum counter (separate per module hierarchy)

- 32-bit reg. Cleared at `START_SCRUB` entry.
- In `MEASURE`, when `s_axis_tvalid & m_axis_tready`, add the four bytes of
  `s_axis_tdata` into it.
- Drive `hwif_in.PIXEL_SUM.PIXEL_SUM.next`.

### A4. Wire STATUS

- `READY`           = `(state == IDLE)`
- `HIST_DATA_VALID` = latched at `FLUSH -> IDLE` transition, cleared on
  reset / new measurement
- `HIST_FIFO_ERR`   = sticky flop of `isp_histogram.err_o`

## Phase B — Testbench two-phase structure

Refactor the `initial` block into two clearly labeled phases.

### Phase 1: register peek/poke

Keep the existing walk of every register (already passing, 29 checks). One
small change: do the full walk first, then trigger an RTL reset
(`aresetn` low → high) to return to a clean slate for the functional test.

### Phase 2: functional — one frame, check, second frame, check deltas

#### B1. Helpers

- `task send_beat(logic [31:0] data, logic tuser)` — drives `s_axis_t*`,
  handshakes on `m_axis_tready` (tied 1 in this TB), updates
  golden-reference accumulators in the same step.
- `task send_frame(input int seed)` — 800 × 320 beats; first beat
  `TUSER=1`, rest 0. Pattern: `$urandom(seed)` for reproducibility.
- `task read_all_bins()` — iterate 0..255 via `HIST_ADDR` / `HIST_DATA`
  frontdoor (uses the autoinc feature already tested in Phase 1).

#### B2. Golden reference

- `int exp_bin[256]`, `longint exp_sum` updated inside `send_beat` from the
  same four bytes.

#### B3. Sequence

1. RTL reset → release.
2. Write `CTRL = HISTOGRAM_START | HIST_ADDR_AUTOINC`.
3. `repeat(300) @(posedge clk);` (covers 256-cycle scrub + slack).
4. `send_frame(seed = 1)`.
5. Wait until `STATUS.HIST_DATA_VALID == 1` (poll with a timeout).
6. `read_all_bins()` → compare each to `exp_bin[i]`; read `PIXEL_SUM` →
   compare to `exp_sum[31:0]`.
7. Write `CTRL.SNAPSHOT`; read `FRAME_CNT_SNAP`, `CYCLE_CNT_SNAP_{LO,HI}`;
   stash them.
8. Write `CTRL.HISTOGRAM_START` again; wait 300; `send_frame(seed = 2)`;
   wait for `HIST_DATA_VALID`; `read_all_bins` + `PIXEL_SUM` check.
9. `CTRL.SNAPSHOT` again; verify:
   - `FRAME_SNAP_new - FRAME_SNAP_old == 2` (two frames since first
     snapshot — TUSER ticks on the first beat of each new frame).
   - `CYCLE_SNAP_new - CYCLE_SNAP_old` ≥ cycles to send one frame +
     overhead and ≤ a loose upper bound.

#### B4. Reporting

Running tally with the existing `report_check` helper so Phase 1 + Phase 2
share the pass/fail summary.

## Open questions (answer before coding)

1. **Pixel pattern** for `send_frame` — `$urandom` with a fixed seed, or a
   deterministic ramp (e.g. `pix = (line*HRES + col) & 0xFF`) for easier
   waveform inspection? Default pick: `$urandom` with fixed seed.
2. **Beat cadence** — `tvalid=1` continuously (one beat per clock) for
   max throughput, or gap occasionally to exercise the histogram's FIFO?
   Default pick: continuous for the first pass.
3. **Second-frame `HISTOGRAM_START` re-issue** — spec says the start bit
   re-issues a scrub, so `read_all_bins` for frame 2 sees only frame 2's
   histogram. Plan assumes this.
4. **Output `m_axis_tready`** — stays tied to 1 (testbench is the sink).
