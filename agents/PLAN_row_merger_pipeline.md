# PLAN — Pipeline the blob core scan path to close 200 MHz independent of the ILA

## Goal

Recover post-route timing margin on `clk_out200` (200 MHz / 5.000 ns) **without
relying on the ILA being absent**. After the passthrough/tuser fixes (commit
e00ff6c) the design meets timing at **WNS = 0.000 ns** (0 failing endpoints) with
an ILA instrumented; the worst path is a self-loop inside `row_merger`. Target:
**WNS ≥ +0.10 ns with 0 failing endpoints in the ILA-less build**, and the new
worst path no longer the `scan_ptr` loop. Keep the change **bit-exact** so both
consumers of the blob core stay green.

## Root cause (from the routed report)

Worst setup path (WNS 0.000), entirely in `u_mocap_top/u_row_merger`:

```
Source:      row_merger/scan_ptr_reg[0]/C          (FDCE)
Destination: row_merger/scan_ptr_reg[1]/CE         (clock-enable)
Data path:   4.738 ns  ->  logic 0.997 ns (21%) | ROUTE 3.741 ns (79%)
Logic levels: 8  (LUT6 -> MUXF7 -> MUXF8 -> LUT6 -> LUT5 -> CARRY8 -> LUT2 -> LUT5)
```

In `RM_SCAN` (row_merger.sv:116-121 comb, 218-241 ff) each cycle does, in series:
1. **array read** `prev_xs[scan_ptr]`/`prev_xe[scan_ptr]` — a MAX_RUNS_PER_ROW:1
   mux (64:1 for mocap = the MUXF7+MUXF8; wide mux -> route-heavy), then
2. **16-bit compares** (`prev_xs[scan_ptr] > cur_xe+1`, `prev_xe[scan_ptr]+1 >=
   cur_xs`, ... = the CARRY8), then
3. **feeds back** to `scan_ptr`'s next value and its clock-enable.

So the loop is *register -> 64:1 mux -> comparator -> same register*. Near-critical
(0.006-0.010 ns) is the same shape in `blob_table`: `root_b_reg`/`state_reg ->
blob_sum_y_reg[*]/D` (union-find root lookup feeding an accumulate in one cycle).

## Fix — split the array read out of the compare (bit-exact)

Replace `RM_SCAN` with two sub-states so the wide read and the comparator never
share a cycle:

- **`RM_SCAN_RD`**: drive `scan_ptr` as the address; register the read into
  pipeline regs `ps_xs_q <= prev_xs[scan_ptr]`, `ps_xe_q <= prev_xe[scan_ptr]`,
  `ps_bid_q <= prev_bid[scan_ptr]`, `ps_ptr_q <= scan_ptr`,
  `ps_valid_q <= (scan_ptr < prev_count)`. Path = mux -> FF only. -> `RM_SCAN_CMP`.
- **`RM_SCAN_CMP`**: operate on the registered `ps_*_q` (no array mux in cone):
  - early-out `!ps_valid_q` OR `ps_xs_q > cur_xe+1` -> `RM_ALLOC` (same guard as
    row_merger.sv:117-120);
  - else reproduce the overlap/match/merge writes (lines 222-233) using
    `ps_xe_q/ps_xs_q/ps_bid_q`; `scan_start <= ps_ptr_q + 1` when
    `ps_xe_q + 1 < cur_xs` (line 235-237); `scan_ptr <= ps_ptr_q + 1`; ->
    `RM_SCAN_RD`.
- `RM_ACCEPT` now transitions to `RM_SCAN_RD` (was `RM_SCAN`); `scan_ptr <=
  scan_start` on accept is unchanged.

Keep the repo's mandatory **separate `always_comb` (next-state) / `always_ff`
(registers)** FSM style.

**Bit-exactness argument:** the same `prev` runs are visited in the same order,
`cur_xs/cur_xe` are stable across a scan, so the `cur_blob_id` / `merge_queue` /
`scan_start` sequence is identical. Only the scan takes 2x cycles. That latency is
absorbed by the existing `run_ready` backpressure to `run_extractor`, and — thanks
to the datapath fork — a slower blob core cannot stall video; the blob result just
lands a few cycles later.

## Work items

1. **row_merger.sv** (primary — the WNS path): enum `RM_SCAN` -> `RM_SCAN_RD` +
   `RM_SCAN_CMP`; add `ps_xs_q/ps_xe_q/ps_bid_q/ps_ptr_q/ps_valid_q`; rewrite the
   scan comb + ff blocks per above.
2. **blob_table.sv** (secondary — decide AFTER re-timing row_merger): it was
   already RD/WR-split once. Re-run impl (ILA-less) first; only if
   `root_b -> blob_sum_y` still limits, register the union-find root's attribute
   read before the accumulate (same technique, same bit-exactness argument).
3. **No change** to run_extractor, blob_detect_rle_top, mocap_top, the RDL, or the
   app — the fix is handshake-compatible.

## Verification (BOTH consumers — row_merger/blob_table feed two IPs)

- `cd vivado/ip_repo/blob_detect_rle/sim && make verify TB=tb_blob_detect_rle`
  (standalone IP, MAX_RUNS_PER_ROW=640)
- `cd vivado/ip_repo/mocap/sim && make verify TB=tb_mocap_wrapper`
  (51/51 self-check + blob/hist cosim, 64/64)
- Bit-exact gate: `compare.py` reports 0 field mismatches vs
  `blob_detect_rle_model.py` on both suites.

## Timing sign-off (ILA-independent)

- Rebuild WITHOUT the ILA in the BD (the production config):
  `cd vivado/kv260_ov9281 && make proj && make bitstream`.
- Pass: `clk_out200` WNS >= +0.10 ns, 0 failing endpoints; worst path no longer
  the `scan_ptr` self-loop. Keep a SEPARATE ILA project for debug; never ship it.

## Risks / rollback

- Off-by-one in the RD->CMP handoff (e.g. using `scan_ptr` instead of `ps_ptr_q`
  for `scan_start`). The two cosim suites catch pixel-level divergence at once.
- Micro-commit row_merger alone first; confirm both cosims green before touching
  blob_table. Each module is an isolated commit; revert if cosim regresses.
- Throughput: scan cycles double — negligible for sparse markers, harmless to
  video. If cycles ever matter, fall back to a Pblock, not un-pipelining.

## Commit cadence (branch ping-pong-mocap, author = user, NO Claude trailers)

1. row_merger pipeline + both cosims green.
2. Re-time ILA-less; record WNS.
3. blob_table only if still needed.
4. Final impl.
