# After-Action Review — Blob Detection Bring-up on KV260 (mocap_wrapper)

Session date: 2026-08-06. Branch: `ping-pong-mocap`.
Scope: took the fused ISP-histogram + RLE-blob-detector wrapper (`mocap_wrapper`)
from "synthesizes but won't place" to a clean, timing-closed KV260 implementation
at 200 MHz, plus a register-map refactor. This AAR captures what was wrong, what
fixed it, the final numbers, and the reusable lessons.

---

## 1. Starting state

- `mocap_wrapper` (SystemVerilog core `mocap_top` + Verilog IP-Integrator shell
  `mocap_wrapper.v`) had been swapped into the KV260 `kv260_ov9281` block design
  in place of `isp_math_wrapper`. All sims passed.
- **Implementation failed**: `place_design` never ran. DRC error
  `[DRC UTLZ-1] LUT as Logic over-utilized: requires 125669 but only 117120
  available`. `mocap_wrapper` alone drew **113,567 LUTs (97%)** and 137,852 FFs
  with ~35k F7/F8 muxes but only 1 BRAM tile.

## 2. Root causes and fixes (in the order found)

### 2a. `wrapper_blob_buf` async read → fabric FFs + 256:1 mux (area)
`mocap_top.sv` read the blob double-buffer combinationally:
```
wire [159:0] blob_rd_rec = wrapper_blob_buf[read_bank_q][BLOB_ADDR[6:0]];
```
A BRAM has no async read port, so Vivado built all `2*128*160 = 40,960` bits as
flip-flops **plus a 256:1 × 160-bit combinational mux** — the entire LUT/FF
blowup. Two-part fix:
1. **Register the read** (synchronous → BRAM-eligible). Safe because AXI-Lite
   AR→R latency + `BLOB_ADDR` held stable across a blob's field reads absorb the
   1-cycle latency — the exact scheme the histogram readback already used.
2. **Flatten the array**: a 3D `[bank][idx]` unpacked array with *different* bank
   indices on the read vs write port is NOT inferred as BRAM (Synth 8-11357).
   Flatten to 1D `wrapper_blob_buf[0:2*MAX_BLOBS-1]`, address `{bank, idx}`.
   → maps to a simple-dual-port BRAM (1× RAMB36 + 2× RAMB18).

**Lesson:** any array read combinationally at a runtime index becomes FFs + a big
mux. For a register file feeding a CPU readback, register the read and give it a
1D `{bank,idx}` address so it infers BRAM.

### 2b. The real elephants were the *reused* blob engine modules (area)
After 2a, `mocap_top` was still ~98.7k LUTs. Hierarchical util revealed the
dominant consumers were the reused, unmodified blob core:
- `blob_table`: **88,820 LUTs** (union-find `parent`/`rank` + 7 per-blob attribute
  arrays, all `[0:MAX_BLOBS-1]`, randomly accessed → register file + 64:1 muxes).
- `row_merger`: **50,381 FFs** (two full-row run buffers of `MAX_RUNS_PER_ROW`).

These are register files **by necessity** — single-cycle random-access RMW +
pointer-chasing (`parent[parent[find_x]]`) — not async-read bugs. They can't
trivially be BRAM. Cost scales directly with the params:
- `blob_table` ≈ **694 LUT per blob** (`MAX_BLOBS`)
- `row_merger` ≈ **79 FF per run** (`MAX_RUNS_PER_ROW`)

**Fix = parameters.** `MAX_BLOBS 128→64` and `MAX_RUNS_PER_ROW 640→64` (sparse IR
markers need far fewer of each). Set on the BD cell in `src/bd/system.tcl`:
```
set_property -dict [list CONFIG.MAX_BLOBS {64} CONFIG.MAX_RUNS_PER_ROW {64} ] $mocap_wrapper_0
```
Whole-device LUT dropped **84% → 45%** — placed and routed.

**Param semantics (important, non-obvious):** `MAX_BLOBS` is the max number of
distinct labels *allocated* per frame, **including labels that later merge away**,
NOT the final blob count and NOT rows. A concave/multi-top shape (U, comb, ring)
or noise speckle each consume labels before merging. Counter resets per frame.
`MAX_RUNS_PER_ROW` bounds RLE runs in one row (worst case ⌈HRES/2⌉ = 320 for
640-wide); overflow silently drops runs (`row_merger.sv:258`). Size both to
real-world worst-case + noise margin, not to expected final blob count.

### 2c. `blob_table` failed 200 MHz timing (WNS −0.710 ns)
With area fixed, the design placed/routed but missed 200 MHz. Every critical path
was inside `blob_table`: `root → 64:1 read-mux → arithmetic → write-back`, all in
one cycle (12 logic levels, 71% routing). Also the `parent[parent[find_x]]`
chained double-mux in the FIND states.

**Fix = pipeline the register-file RMW** (commit `9f3c8cf`). The FSM already
processes one run/merge at a time and returns to `BT_READY` between ops, so a
latched read is always consumed by the immediately following write — **no RAW
hazard, no forwarding needed**. Split each RMW state into read + write:
- `BT_ACCUM` → `BT_ACCUM_RD` (latch `blob_*[find_root]` **and register the DSP run
  products** `sum_x_add` / `w_row*run_len`) + `BT_ACCUM` (clean 32-bit adds).
- `BT_MERGE_UF` → `BT_MERGE_RD` (latch both roots' descriptors + ranks) + merge.
- `BT_FLATTEN` → read stage + `BT_FLAT_EMIT` (write `result_ram` from latched regs).
- `BT_FIND*`: dropped grandparent path-halving (the chained double-mux). Union-by-
  rank alone bounds tree height to ≤ log2(MAX_BLOBS) < the 8-iteration cap, so
  finds still resolve. (Removes an optimization, preserves correctness.)

**Two build iterations, each ~30 min:**
1. Read/write split (isolates the routing-heavy 64:1 mux): WNS **−0.710 → −0.215**.
   New worst path exposed: the combinational **DSP multiply** for `sum_x_add` in
   series with the accumulate adder (15 levels, now 68% *logic*/DSP).
2. Register the DSP products in the RD stage: WNS **−0.215 → +0.083** ✅.

**Lesson:** pipelining a register file is a two-stage story — first isolate the
read-mux (fixes routing), which then *exposes* the arithmetic/DSP path (fixes
logic). Watch how the failing path's logic/route split flips between iterations.

## 3. Register-map refactor: CTRL/CMD split (commit `3ec77ec`)

The old `CTRL` (0x00) mixed sticky settings (`ENABLE`, `*_AUTOINC`, `THRESHOLD`)
with write-only single-pulse commands (`RESET`, `RESULTS_ACK`, `CYCLE_SNAPSHOT`).
Because a normal 32-bit `writel` drives `wstrb=0xF`, pulsing `RESULTS_ACK` every
frame would clobber `THRESHOLD` to 0 (→ `pixel>=0` = everything foreground) unless
SW did a byte-write or read-modify-write / shadow. (The regblock *does* honor
`wstrb`, and `THRESHOLD` is in byte 1 vs the pulses in byte 0 — but relying on
byte-strobed writes is non-portable.)

**Fix:** split into two registers.
- `CTRL @ 0x00` — sticky R/W settings only: `ENABLE`(b0), `HIST_ADDR_AUTOINC`(b1),
  `BLOB_ADDR_AUTOINC`(b2), `THRESHOLD`[15:8].
- `CMD @ 0x68` — write-only single-pulse doorbell: `RESET`(b0), `RESULTS_ACK`(b1),
  `CYCLE_SNAPSHOT`(b2). Reads 0; a full-word write never disturbs settings.
- Region grew 0x68 → 0x6C. `RESET`/`ENABLE` are now orthogonal (RESET clears
  dynamic state; ENABLE is independent policy — SW must clear ENABLE to land in
  READY after a reset). Regenerated PeakRDL pkg/regs/defines; 3 refs in
  `mocap_top.sv` moved `CTRL.* → CMD.*`.

**Lesson (drives the SW driver):** a control register that mixes sticky config
with per-event pulses is a footgun. Separate CONFIG (sticky, shadowed in SW) from
a COMMAND/doorbell (write-1-to-pulse, nothing to preserve). The SW pipeline driver
should keep a CTRL shadow and write CMD for pulses.

## 4. Final result (KV260 xck26, `MAX_BLOBS=64`, `MAX_RUNS_PER_ROW=64`)

| Metric | Value |
|---|---|
| Timing (clk_out200 = 200 MHz) | **WNS +0.083 ns, TNS 0, 0 failing endpoints** |
| Hold | WHS +0.010 ns, 0 failing |
| "All user specified timing constraints are met." | ✅ |
| CLB LUTs | 56,172 (48%) |
| CLB Registers | 49,757 (21%) |
| Block RAM | 72.5 tiles (50%) — 70× RAMB36 + 5× RAMB18 |
| DSP | 2 |

Implementation was run **to route only** (no bitstream) via
`kv260_ov9281/build_synth_impl.tcl` (synth_1 → impl_1 `-to_step route_design`).

## 5. Verification approach (kept bit-exact throughout)

Every RTL change was gated on three suites, all green after each edit:
- **`tb_blob_detect_rle`** (standalone blob core) — `make verify`: 36/36 self-checks
  + **7/7 frames bit-exact** vs the Python golden (`blob_detect_rle_model.py` +
  `compare_all.py`). This is the authoritative check for the shared core.
- **`tb_mocap_wrapper`** — 51/51 (datapath cosim + register/ownership/race groups).
- **`tb_mocap_wrapper_v`** — 17/17 (instances the Verilog shell; frontdoor only).

The pipeline change (adding cycles to `blob_table`) is behavior-preserving because
the mocap FC/copy/ownership FSMs are handshake/`flatten_done`-driven, not
cycle-count-dependent. **Lesson:** a strong cosim TB lets you do aggressive
timing surgery on a verified core with confidence.

## 6. Key files

- Core: `vivado/ip_repo/mocap/hdl/mocap_top.sv`, shell `mocap/hdl/mocap_wrapper.v`
- Reused blob core (shared with standalone IP): `vivado/ip_repo/blob_detect_rle/hdl/{blob_table,row_merger,run_extractor}.sv`
- Reg map: `vivado/ip_repo/mocap/rdl/mocap_regs.rdl` (+ `mocap_regs_defines.svh`);
  generated products in `rdl_out/` (gitignored) — `rtl/mocap_regs*.sv`,
  **`sw/mocap_regs.h`** (the C header the Linux driver will use), `html/`.
  Regenerate: `make -C vivado/ip_repo/mocap/rdl`.
- BD source: `vivado/kv260_ov9281/src/bd/system.tcl` (mocap_wrapper_0 @ 0xA0011000,
  CONFIG.MAX_BLOBS/MAX_RUNS_PER_ROW = 64).
- Build: `vivado/kv260_ov9281/Makefile` (`make proj`), `build_synth_impl.tcl`.
- TBs/sims: `vivado/ip_repo/mocap/sim/`, `vivado/ip_repo/blob_detect_rle/sim/`.

## 7. Register map (region 0x6C, AXI-Lite @ 0xA0011000)

| Off | Reg | Notes |
|----|----|----|
| 0x00 | CTRL | ENABLE(b0), HIST_ADDR_AUTOINC(b1), BLOB_ADDR_AUTOINC(b2), THRESHOLD[15:8]=128 — **sticky R/W** |
| 0x04 | STATUS | READY(b0), RESULTS_VALID(b1), FRAME_DONE_IRQ(b2), READ_BANK(b3), HIST_FIFO_ERR(b4,sticky), BLOB_OVERFLOW(b5,sticky), OVERRUN(b6,sticky), BLOB_COUNT[15:8] |
| 0x08/0x0C | HRES/VRES | default 640/400 |
| 0x10 | FRAME_ID | RO monotonic id of published buffer |
| 0x14 | DROPPED_FRAMES | RO keep-latest overrun count |
| 0x18 | PIXEL_SUM | RO |
| 0x20/0x24 | HIST_ADDR/HIST_DATA | indirect, autoinc |
| 0x28 | BLOB_ADDR | RW 8b, autoinc-on-BLOB_COUNT_RD-read |
| 0x2C..0x44 | BLOB_COUNT_RD, BLOB_SX, BLOB_SY, BLOB_XMIN, BLOB_XMAX, BLOB_YMIN, BLOB_YMAX | RO per-blob descriptor at BLOB_ADDR (published bank) |
| 0x48 | MAX_BLOBS_CFG | RO reports MAX_BLOBS |
| 0x50..0x5C | DMA_* | reserved |
| 0x60/0x64 | CYCLE_SNAP_LO/HI | RO 64b cycle snapshot |
| 0x68 | CMD | WO pulses: RESET(b0), RESULTS_ACK(b1), CYCLE_SNAPSHOT(b2) |

Ownership/IRQ contract (race-free double buffer): HW raises `frame_done_irq` at
publish; SW reads the published bank (hist + blobs + FRAME_ID coherent) then
pulses `CMD.RESULTS_ACK` to clear the IRQ and release the buffer. Keep-latest on
overrun (`DROPPED_FRAMES` counts). Blob field order matches the golden:
count, sum_x, sum_y, xmin, xmax, ymin, ymax. Centroid = sum_x/count, sum_y/count.

## 8. Open items / follow-ups

- **`timing.xdc`**: the `set_clock_groups -asynchronous` line is currently
  commented out (committed as-is at user request). Timing closes anyway, but it
  should be restored so genuine async CDC crossings aren't timed as real paths.
- **No bitstream generated** — impl runs to route only. Generate `.bit`/`.bit.bin`
  once ready.
- **Throughput note:** pipelining adds ~1 cycle/run and ~1 cycle/merge to
  `blob_table`. Fine for sparse markers; a pathologically dense row (approaching
  `MAX_RUNS_PER_ROW` runs) has less slack to keep up with line rate — the sim
  can't catch a real-time violation (it just backpressures). Watch on hardware.
- **row_merger FFs** are the next-largest block if further shrink is needed;
  independent of `MAX_BLOBS` (scales with `MAX_RUNS_PER_ROW`).

## 9. Commits (branch `ping-pong-mocap`)

- `3ec77ec` mocap: BRAM blob buffer, CTRL/CMD register split, 64/64 sizing
- `9f3c8cf` blob_table: pipeline the descriptor RMW paths to close 200 MHz
