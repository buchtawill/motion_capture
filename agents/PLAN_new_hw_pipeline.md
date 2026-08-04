** Prompt **
I want to implement a new hardware pipeline based off of the ISP module and blob detector. Right now, there is no ping-pong buffering in hardware, and once a frame is complete, there is a race condition to read the ISP and blob detector data from SW, otherwise data gets overwritten. 

I want you to create a new module, mocap_wrapper, which instantiates the following:
1. the ISP histogram module (instanced as u_isp). Change the RTL so that it buffers the pipeline instead of snooping it
2. the blob detector (instanced as u_blob_detect_rle). 
3. A register file and general control signals / debug info.

Help me brainstorm how to architect this module. Do not take effort into consideration; this pipeline should be architecturally pure. Do not take into consideration what is currently implemented either; just the high level concepts of the ISP and run length encoded blob detector. 

There are two data architectures:
1. SW waits for a HW interrupt, the module asserts an interrupt when data is ready, and SW needs to read data from registers in the HW (256 histogram bins, n blobs detected, cycle counts, etc)
2. SW sets up a DMA buffer and waits for an interrupt, HW streams data as it comes into memory (metadata about the frame - the actual data capture pipeline is still the same), then HW asserts the interrupt when done.

Start by planning an arch / spec for data arch number 1, keeping in mind that transitioning to data arch number 2 is a possibility in the future. The main goal of this brainstorm session is to come up with an architecture that has no race conditions between hardware and software.

** Working Area for agent **

# mocap_wrapper — Architecture Brainstorm (Data Arch #1)

## 0. Root cause of the race (name it precisely)

The results of a frame live in **HW-side memory**: 256 histogram bins (`hist_mem`,
20b each) and up to `MAX_BLOBS` blob records (~160b each). SW reads them through
indirect address/data register pairs (`HIST_ADDR`/`HIST_DATA`,
`BLOB_ADDR`/`BLOB_*`). A full drain is *hundreds* of AXI-Lite reads → many µs.

That memory is **single-buffered**. Today's mitigation is the single-shot
`START`: HW parks in IDLE after a frame and won't touch the memory until SW
re-arms. That is race-free but throughput-limited — to not *miss* the next frame,
SW is tempted to re-arm before it finishes reading, and then HW scrubs/overwrites
the memory SW is still reading. **That is the race.** It is not a bug in either
engine; it is a missing producer/consumer decoupling.

> The fix is not "read faster." A design that depends on SW winning the race is
> race-*unlikely*, not race-*free*. The fix is to make the buffer HW writes and
> the buffer SW reads never be the same buffer.

## 1. Design principles

- **Producer/consumer decoupling via double-buffering (ping-pong).** HW writes
  bank X while SW reads bank Y. Guaranteed no shared cell.
- **Hardware owns the swap; software owns a release.** The bank SW is reading is
  never reused by HW until SW explicitly hands it back. Correctness does not
  depend on SW latency — only *throughput* (dropped-frame count) does.
- **Atomic snapshot.** Everything SW needs for one frame (scalars + which bank
  holds the arrays) is latched in one cycle at frame-done, so any later read is
  coherent.
- **One frame, one control domain.** Both engines run in lockstep off a single
  ingress buffer, one SOF/EOF, one HRES/VRES, one IRQ, one register file.
- **Asymmetric datapath posture, chosen per engine:**
  - **Histogram = snoop, double-buffered by instancing two `isp_histogram`.**
    The module is self-contained BRAM that already serves SW reads on its
    `ram_addr_i`/`ram_data_o` port whenever `hist_en_i=0`. Instance it **twice**
    on the same snooped stream; only one has `hist_en_i` asserted at a time. The
    active instance counts the current frame; the idle one holds the last frame
    for SW. **Zero edits to `isp_histogram.sv`.** A snoop is fine here precisely
    because the two instances give the race-free decoupling.
  - **Blob detector = in series** (`… → blob → FIFO → out`), because it is the
    expensive engine and duplicating it is not worth it. Double-buffer only its
    **result memory** (see §3.1), not the whole engine.
- **Uniform sub-block contract.** Each engine sub-block takes `{hres, vres,
  start}` and produces `{done, ready}`. The wrapper drives `start`/dims and
  consumes `done`/`ready`; the **top-level IRQ is `isp_done & blob_done`**
  (both latched), so SW gets one interrupt only when *both* results are ready.
- Datapath latency is a non-issue and not a design constraint.

## 2. Block architecture

```
                                     mocap_wrapper
  s_axis      ┌──────────────────────────────────────────────────────────────┐   m_axis
  (CSI) ──┬──►│  FIFO ─────────────────────► u_blob_detect_rle ─► FIFO ───────┼─► (VDMA)
          │   │                                    │                          │
          │   │  snoop           ┌── u_hist_0 (hist_en ping) ──┐  result_ram  │
          └───┼─────────────────►┤                            ├── [2] banks  │
              │  (no backpressure)└── u_hist_1 (hist_en pong) ─┘              │
              │                                                               │
              │   frame-control FSM: dims → start; count beats; on both       │
              │      done → publish/swap + snapshot + IRQ(=isp_done&blob_done)│
              │                                                               │
              │   register file (AXI4-Lite)  ─────────────────────────────────┼─► IRQ
              └──────────────────────────────────────────────────────────────┘
```

Two datapath postures, one per engine:

- **Histogram (snoop, no backpressure):** two `isp_histogram` instances tap the
  input stream. The FSM asserts `hist_en_i` on exactly one at a time (ping-pong).
  Active instance counts the current frame; idle instance holds the previous
  frame and answers SW reads on its `ram_addr_i`/`ram_data_o` port. `isp_done`
  for the frame = active instance has counted `hres*vres/4` beats **and** flushed
  its internal FIFO. Unmodified `isp_histogram.sv`.
- **Blob detector (series):** a single `u_blob_detect_rle` sits inline
  (`FIFO → blob → FIFO → out`), consuming beats with backpressure and re-driving
  the stream. `blob_done` = `flatten_done`.

Both engines **lose their own HRES/VRES, START/IRQ, frame counters, and register
blocks** — the wrapper owns all control/status via the uniform
`{hres,vres,start}→{done,ready}` contract, and points each engine's result
memory at the correct bank.

## 3. The race-free mechanism (the core of the whole design)

Two banks per result store. Three bits of state track ownership:

- `write_bank` — bank the currently-capturing frame writes into (HW-owned).
- `read_bank`  — bank holding the most-recently-*published* completed frame.
- `sw_owns`    — set when a frame is published; cleared by `CTRL.RESULTS_ACK`.

**At frame-done (both engines done for the frame), in one cycle:**
1. latch scalar snapshot for this frame (frame_id, blob_count, pixel_sum,
   cycle/frame counts, per-engine error flags) into `snapshot[write_bank]`;
2. `read_bank <= write_bank` (publish);
3. `sw_owns <= 1`;
4. `write_bank <= other bank`;
5. pulse `RESULTS_VALID` + IRQ.

**SW read protocol (arch #1):**
1. wait for IRQ → read `STATUS` (gets `FRAME_ID`, `BLOB_COUNT`, flags);
2. drain arrays via `HIST_ADDR/DATA` + `BLOB_ADDR/*` — HW muxes the RAM read
   address as `{read_bank, addr}`, so SW's register map is unchanged and always
   hits the published bank;
3. write `CTRL.RESULTS_ACK` (single-pulse) → clears `sw_owns`, releasing the bank.

**Backpressure / overrun policy (why it's truly race-free):**
While `sw_owns==1`, HW's *only* free bank is `write_bank`. If a new frame
completes before SW ACKs:
- HW **must not** publish into the SW-owned bank. It keeps overwriting
  `write_bank` frame-over-frame (keep-latest), incrementing `DROPPED_FRAMES` for
  each completion it couldn't publish and latching `STATUS.OVERRUN` sticky.
- When SW finally ACKs, the next frame boundary publishes the freshest completed
  frame.

→ The bank SW reads is **never** touched by HW. Corruption is structurally
impossible; slow SW costs *frames*, not *integrity*, and the cost is counted.
Under normal 60 fps operation (16 ms/frame ≫ µs of reads) `DROPPED_FRAMES`
stays 0.

### 3.1 The wrapper owns ALL double-buffering; the engines are unmodified

Both engines are reused **as-is**. Neither engine knows it is being
double-buffered — banking, ownership, keep-latest, and the error counter live
entirely in the wrapper. A single shared `bank_select` / `sw_owns` / ACK /
`DROPPED_FRAMES` steers *both* result stores together, so at any instant
`buffer[N] = { histogram N, blob records N, frame_count N }` is one coherent
unit with one IRQ and one ACK.

- **Histogram — two `isp_histogram` instances (no copy needed).** The two banks
  *are* the two instances. `hist_en_i` is asserted on the write instance only;
  the idle instance holds the previous frame and answers SW reads on its
  `ram_addr_i`/`ram_data_o` port. On publish, `bank_select` flips which instance
  is enabled and which instance's `ram_data_o` the register file muxes onto
  `HIST_DATA`. Reuse of an instance needs an `ram_scrub_i` pulse first (already
  supported). **Zero edits to `isp_histogram.sv`.**

- **Blob detector — copy-out into two wrapper buffers (`blob_detect_rle`
  untouched).** The engine keeps its single internal `result_ram` as HW-private
  scratch (wiped by `BT_CLEAR` every frame; SW never sees it). The wrapper adds a
  ping-pong buffer `wrapper_blob_buf[2][MAX_BLOBS]` (160b/entry) that SW actually
  reads. On `blob_done` (`flatten_done`), a small wrapper copy-FSM:
  1. streams `result_rd_addr` `0 → blob_count-1`, latching `result_rd_data` into
     `wrapper_blob_buf[write_bank]` (**mind the 1-cycle registered read
     latency** — data lags address by one clock), and snapshots `blob_count`;
  2. only *then* lets the wrapper re-issue `start` to the engine — otherwise the
     next frame's `BT_CLEAR` would wipe `result_ram` mid-copy.
  Cost ~`blob_count+1` cycles (≤~130, <1 µs); hides entirely in frame blanking in
  continuous mode. `HRES`/`VRES` come from the wrapper via the engine's existing
  `hres`/`vres` inputs.

> This copy-FSM is deliberately generic: it walks a `{done + read-port}` source
> into a destination buffer. In Arch #2 the destination changes from
> `wrapper_blob_buf` to a DMA master and the same FSM streams the record to
> memory — see §7.

- **Shared 2-bank ownership rule (both stores, one FSM):** if SW hasn't
  `RESULTS_ACK`ed the buffer it holds before the next frame completes, HW writes
  the new frame into the *other* (non-SW) buffer — the two histogram instances
  ping-pong as usual, and the blob copy-FSM targets the non-SW `wrapper_blob_buf`
  bank — keeping SW's buffer untouched and incrementing `DROPPED_FRAMES`.

> Alternative considered — **seqlock / generation counter** (SW reads a
> gen-counter before & after the full drain, retries on mismatch): zero extra
> RAM, but non-deterministic (retry can thrash at high frame rate) and wastes the
> whole drain on a tear. Rejected for arch #1; noted because it's the natural
> fit if we ever want lock-free single-buffer. Double-buffer + ACK is
> deterministic and is the same primitive arch #2 needs.

## 4. Shared control FSM (sketch, separate comb/ff per house style)

```
ST_IDLE      : READY=1. On CTRL.START (single-shot) or MODE.CONTINUOUS, and
               dims_ok, and a free write_bank exists → ST_SCRUB
ST_SCRUB     : scrub hist bank[write_bank] / clear blob bank[write_bank]
ST_WAIT_SOF  : arm engines; wait for TUSER beat
ST_PROCESS   : engines consume the frame into write_bank
ST_FINALIZE  : wait for both engines done (blob flatten + hist flush)
ST_PUBLISH   : atomic publish+swap+snapshot+IRQ (section 3)
               CONTINUOUS & free bank → ST_SCRUB ; else → ST_IDLE
```

- **CONTINUOUS mode is the usability win:** SW never re-arms in a tight window;
  HW free-runs and SW just consumes published banks. Single-shot stays available
  for debug/determinism.
- **`frame_done = isp_done & blob_done`** (both latched). `isp_done` = active
  histogram counted `hres*vres/4` beats + FIFO flushed; `blob_done` =
  `flatten_done`. The histogram snoops the input so it tends to finish first;
  blob receives the frame a FIFO-delay later and flattens after its last beat, so
  `blob_done` is normally the later edge. The AND fires on whichever is last —
  publish both banks together on that edge → one IRQ, lockstep swap. No
  assumption about their relative timing is baked in.
- Free cycle counter + per-frame frame counter live in the wrapper (moved out of
  the ISP), snapshotted into the per-bank snapshot at publish.

## 5. Register map sketch (arch #1) — unified, one AXI4-Lite slave

```
0x00 CTRL     RW  RESET | START | IRQ_CLEAR | RESULTS_ACK(pulse) |
                  MODE_CONTINUOUS | HIST_ADDR_AUTOINC | BLOB_ADDR_AUTOINC |
                  THRESHOLD[8] | ENABLE_ISP | ENABLE_BLOB
0x04 STATUS   RO  READY | RESULTS_VALID | OVERRUN(sticky) | FRAME_DONE_IRQ |
                  HIST_ERR(sticky) | BLOB_OVERFLOW(sticky) | READ_BANK |
                  BLOB_COUNT[8]
0x08 HRES     RW
0x0C VRES     RW
0x10 FRAME_ID RO  monotonic id of the *published* frame (coherency check)
0x14 DROPPED_FRAMES RO
0x18 CYCLE_SNAP_LO / 0x1C HI   RO  (of published frame)
0x20 PIXEL_SUM RO
0x24 HIST_ADDR RW / 0x28 HIST_DATA RO      (indirect, reads read_bank)
0x2C BLOB_ADDR RW / 0x30.. BLOB_* RO       (indirect, reads read_bank)
0x40.. reserved for arch #2 (DMA_BASE_LO/HI, DMA_LEN, DMA_CTRL, RECORD_VER)
```

Key change vs. today's two maps: **one `RESULTS_ACK`, one IRQ, one `FRAME_ID`,
`READ_BANK` exposed for debug**. SW never sees the two engines as separate
lifecycles.

## 6. Coherency guarantees for SW

- Scalars (`STATUS`, `FRAME_ID`, `BLOB_COUNT`, `PIXEL_SUM`, cycle snap) are
  latched together at publish → any read before ACK is self-consistent.
- Arrays (bins, blobs) are in the protected `read_bank` → coherent for the whole
  drain.
- `FRAME_ID` read at start and end of a drain is a cheap tamper-check; in this
  design it will match (bank is protected), but it lets SW *assert* coherency.

## 7. Forward path to Data Arch #2 (DMA streaming) — keep it cheap now

Do these now so #2 is a drop-in of a new *consumer* behind the same publish/ACK:
- **Freeze a versioned result-record layout** (header: `RECORD_VER, FRAME_ID,
  HRES, VRES, BLOB_COUNT, FLAGS`, then `hist[256]`, then `blob[N]`). Arch #1's
  register field order mirrors it.
- Keep result stores as clean addressable RAMs with a single read port (already
  true) so a **result serializer** can walk `read_bank` and push the record to a
  DMA/MM2S master.
- Reserve `0x40..` for `DMA_BASE/LEN/CTRL`.
- In #2, `RESULTS_ACK` becomes "DMA descriptor complete." Publish/swap/overrun
  logic is **identical**; only the transport (SW register reads → HW push to
  memory) changes.

## 8. Decisions — all settled

- **Datapath posture** — histogram = snoop w/ two `isp_histogram` instances;
  blob = series. (§1, §2)
- **Wrapper owns all double-buffering; engines unmodified** — histogram via two
  instances, blob via copy-out into `wrapper_blob_buf[2]`. `blob_detect_rle` and
  `isp_histogram` are reused as-is. (§3.1)
- **One shared ownership FSM** — a single `bank_select`/`sw_owns`/ACK/
  `DROPPED_FRAMES` governs both stores together; `buffer[N]` = {hist, blobs,
  frame_count} as one coherent unit.
- **Overrun policy** — keep-latest: on overrun HW overwrites the non-SW buffer
  and bumps `DROPPED_FRAMES`; SW's buffer is never touched. (§3)
- **Mode** — continuous, no SW re-priming. HW free-runs; SW just consumes
  published buffers.
- **Handshake** — one `CTRL.RESULTS_ACK` pulse = clear IRQ + release buffer.
- **Sub-block contract** — `{hres,vres,start}→{done,ready}`; top IRQ =
  `isp_done & blob_done`, lockstep publish. (§4)
- **Latency** — not a constraint.

