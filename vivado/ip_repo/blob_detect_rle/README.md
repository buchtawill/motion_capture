# blob_detect_rle — RLE-Based Streaming CCL Blob Detection IP

Streaming connected-component labeling (CCL) blob detector for 8-bit grayscale AXIS pixel streams. Sits in the pixel datapath as an AXIS slave/master passthrough, detecting up to 128 foreground blobs per frame with pixel-exact centroids and bounding boxes. Results are reported via AXI4-Lite registers and a frame-done IRQ.

Algorithm matches Bailey's *Design for Embedded Image Processing on FPGAs* (2nd ed., 2019, Ch. 10) and He, Chao, Suzuki (2008) — single-pass RLE + union-find CCL.

## Directory layout

```
blob_detect_rle/
├── hdl/
│   ├── run_extractor.sv          # Stage 1: threshold + run-length encoding
│   ├── row_merger.sv             # Stage 2: run matching + union-find merge
│   ├── blob_table.sv             # Stage 3: accumulator + flatten
│   ├── blob_detect_rle_top.sv    # Top-level FSM, FIFOs, register wiring
│   ├── blob_detect_rle_wrapper.v # Verilog-2001 wrapper for IP Integrator
│   └── stream_fifo.sv            # Reusable AXIS FIFO (shared with grid IP)
├── model/
│   ├── blob_detect_rle_model.py  # Python functional model (bit-exact reference)
│   ├── gen_stimulus.py           # Test frame generator (random circular blobs)
│   ├── compare.py                # Single-frame RTL vs model comparator
│   ├── compare_all.py            # Multi-frame batch comparator
│   └── run_models.py             # Batch runner for the Python model
├── rdl/
│   ├── blob_detect_rle_regs.rdl  # PeakRDL register map specification
│   ├── Makefile                  # Regenerate RTL/header/docs from RDL
│   └── rdl_out/
│       ├── rtl/                  # Generated AXI4-Lite register block (.sv)
│       ├── sw/                   # Generated C header (blob_detect_rle_regs.h)
│       └── html/                 # Generated HTML register docs
└── sim/
    ├── tb_blob_detect_rle.sv     # Multi-frame xsim testbench
    ├── wave_vcd.tcl              # xsim waveform dump script
    └── Makefile                  # Build/run/verify targets
```

## RTL module hierarchy

```
blob_detect_rle_wrapper.v          (Verilog-2001 shell for IP Integrator)
└── blob_detect_rle_top.sv         (top-level FSM + register + FIFO wiring)
    ├── blob_detect_rle_regs.sv    (PeakRDL-generated AXI4-Lite register block)
    ├── stream_fifo (u_in_fifo)    (input AXIS FIFO, depth 16)
    ├── stream_fifo (u_out_fifo)   (output AXIS FIFO, depth 16)
    ├── run_extractor.sv           (threshold + RLE per row)
    ├── row_merger.sv              (run matching + blob ID assignment + merge events)
    └── blob_table.sv              (union-find + per-blob accumulator + flatten)
```

### Data flow

```
AXIS slave ──► in_fifo ──┬──► run_extractor ──► row_merger ──► blob_table
                         │
                         └──► out_fifo ──► AXIS master  (pixel passthrough)
```

Pixels flow through the input FIFO and are consumed by the run_extractor while simultaneously being forwarded to the output FIFO. The three processing stages form a streaming pipeline connected by valid/ready handshakes. The pixel stream passes through unmodified; blob detection is a side computation.

### Top-level FSM (`blob_detect_rle_top.sv`)

```
ST_IDLE ──(START)──► ST_CLEAR ──► ST_WAIT_SOF ──(TUSER[0])──► ST_PROCESS
                                                                    │
                                                            (all pixels consumed)
                                                                    │
                         ST_DONE ◄── ST_FINALIZE ◄──────────────────┘
                           │          (blob_table flatten, ~128 cycles)
                           └──► ST_IDLE
```

- **ST_IDLE**: waits for software START pulse. HRES/VRES must be configured.
- **ST_CLEAR**: resets all sub-modules (blob_table clears its BRAM).
- **ST_WAIT_SOF**: waits for first beat with TUSER[0]=1 (start of frame).
- **ST_PROCESS**: all three sub-modules run concurrently as a streaming pipeline.
- **ST_FINALIZE**: triggered when run_extractor reports frame_done. Signals blob_table to flatten its union-find structure and compact blob descriptors into the result BRAM. Takes ~128 cycles.
- **ST_DONE**: latches blob_count into the status register, asserts frame_done IRQ. Returns to IDLE on next cycle.

### Stage 1: run_extractor

Consumes 4 pixels per beat (32-bit AXIS, LSB-first). Compares each pixel against the threshold register. Tracks foreground runs across beats using `in_run` and `run_xs` registers. Outputs `(xs, xe, row, last_in_row)` per contiguous foreground run.

A single beat can produce 0, 1, or 2 run outputs (e.g., `fg=4'b1001` closes one run and opens+closes another). A 2-entry skid buffer handles the rare 2-output case with one stall cycle.

Row boundaries: when `pix_cnt + 4 >= hres`, trailing pixels beyond hres are masked. A partial run is flushed with `xe = hres - 1`. Emits `last_in_row` on the final run of each row. Reports `frame_done` after `vres` rows.

### Stage 2: row_merger

Compares current-row runs against previous-row runs to assign blob IDs and detect merges. Uses a BRAM-backed ping-pong buffer (MAX_RUNS_PER_ROW=640 entries per bank) to store previous-row runs as `(xs, xe, blob_id)` tuples.

**Per current run:**
1. Scan previous-row buffer for 8-connected overlaps: `xs_c <= xe_p + 1 AND xs_p <= xe_c + 1`
2. First overlap → inherit that blob_id
3. Additional overlaps with a different blob_id → emit `merge(a, b)` event to blob_table
4. No overlap → allocate `next_blob_id++`, flag `is_new_blob`

Row boundary detection uses a `last_seen_row` register that compares against `in_row` — this is more robust than relying on `last_in_row` from run_extractor, which only fires when a row has foreground pixels. Gap detection: if `in_row != last_seen_row + 1`, clears the previous-row buffer (non-adjacent rows can't merge).

Outputs labeled runs `(xs, xe, row, blob_id, is_new)` and merge events `(merge_a, merge_b)` to blob_table.

### Stage 3: blob_table

**Union-find**: `parent[0:127]` register array (128 x 7 bits). `find(x)` iterates the parent chain with an 8-iteration cap and path compression. `union(a, b)` uses union-by-rank.

**Accumulator**: Per-blob descriptor BRAM, 160-bit entries:
- `count[31:0]` — foreground pixel count
- `sum_x[31:0]` — sum of x coordinates (centroid = sum_x / count)
- `sum_y[31:0]` — sum of y coordinates
- `xmin[15:0], xmax[15:0], ymin[15:0], ymax[15:0]` — bounding box

**Run accumulation math** (integer-exact, no floats):
- `run_len = xe - xs + 1`
- `count += run_len`
- `sum_x += (xs + xe) * run_len / 2` (arithmetic series, always integer)
- `sum_y += row * run_len`
- bbox: min/max updates

Uses one DSP48 for the `(xs + xe) * run_len` multiply.

**Flatten phase** (post-frame): linear scan of the parent array. For each root with count > 0, assigns a compact sequential output ID and copies its descriptor to the result BRAM. ~128 cycles.

## Register interface

Defined in `rdl/blob_detect_rle_regs.rdl`, generated via PeakRDL.

| Offset | Name           | R/W | Description |
|--------|---------------|-----|-------------|
| 0x00   | CTRL          | R/W | RESET (pulse), START (pulse), IRQ_CLEAR (pulse), BLOB_ADDR_AUTOINC, THRESHOLD[7:0] |
| 0x04   | STATUS        | RO  | READY, FRAME_DONE, OVERFLOW, FRAME_DONE_IRQ, BLOB_COUNT[7:0] |
| 0x08   | HRES          | R/W | Horizontal resolution (16b, default 1280) |
| 0x0C   | VRES          | R/W | Vertical resolution (16b, default 800) |
| 0x10   | BLOB_ADDR     | R/W | Blob table read index (8b), auto-increments on BLOB_COUNT_RD read when AUTOINC=1 |
| 0x14   | BLOB_COUNT_RD | RO  | Pixel count for blob at BLOB_ADDR |
| 0x18   | BLOB_SX       | RO  | Sum of X for blob at BLOB_ADDR (divide by count for centroid) |
| 0x1C   | BLOB_SY       | RO  | Sum of Y for blob at BLOB_ADDR |
| 0x20   | BLOB_XMIN     | RO  | Bounding box min X |
| 0x24   | BLOB_XMAX     | RO  | Bounding box max X |
| 0x28   | BLOB_YMIN     | RO  | Bounding box min Y |
| 0x2C   | BLOB_YMAX     | RO  | Bounding box max Y |
| 0x30   | FRAME_CNT     | RO  | Frames processed since reset |
| 0x34   | MAX_BLOBS_CFG | RO  | Reports MAX_BLOBS parameter (128) to software |

**Software read sequence** (after IRQ):
1. Read STATUS (0x04) to get blob_count and overflow flag
2. For each blob 0..blob_count-1: write BLOB_ADDR (0x10), then read the 7 descriptor registers (0x14–0x2C)

Alternatively, enable AUTOINC and read sequentially — but write BLOB_ADDR=0 first, then read BLOB_COUNT_RD (which triggers auto-increment), then the remaining 6 fields, repeated per blob. Explicit BLOB_ADDR writes per blob are less error-prone.

### Regenerating from RDL

```bash
source mocap_env/bin/activate
cd rdl/
make          # generates rtl/, sw/, html/
```

## Python functional model

`model/blob_detect_rle_model.py` implements the same 3-stage algorithm in Python, producing bit-exact integer results matching RTL:

1. **Run extraction** (`extract_runs`): threshold + RLE per row, returns `(xs, xe)` tuples
2. **Row merge** (`run_model` main loop): compare current runs against previous-row runs using 8-connected overlap test, union-find for merge tracking, per-run accumulation into descriptor table
3. **Flatten** (`flatten_blobs`): resolve union-find roots, merge descriptors, sort by `(ymin, xmin)`

All arithmetic is integer — no floats. The model uses path-compressing union-find with union-by-rank, matching the RTL's 8-iteration-capped find.

### Supporting scripts

| Script | Purpose |
|--------|---------|
| `gen_stimulus.py` | Generates test frames with random circular blobs. Outputs `frame_NNNN.hex` (32-bit words, `$readmemh` format) + `frame_NNNN_meta.json` (resolution, threshold, seed, blob positions). |
| `run_models.py` | Batch-runs the Python model on all `frame_NNNN_meta.json` files in a directory. Outputs `blobs_model_NNNN.json`. |
| `compare.py` | Compares one RTL hex output against one model JSON output. Sorts both by `(ymin, xmin)`, checks all 7 fields per blob. |
| `compare_all.py` | Finds all `blobs_rtl_NNNN.hex` / `blobs_model_NNNN.json` pairs and runs `compare.py` on each. |

## Simulation

Uses Xilinx xsim (xvlog/xelab/xsim) from Vivado 2024.1. All outputs go to `sim/sim_out/tb_blob_detect_rle/`.

### Makefile targets

```bash
cd sim/

make verify           # Full flow: gen_stim → model → compile → sim_fst → compare
make gen_stim         # Generate all test frames
make model            # Run Python model on all test frames
make compile          # Compile RTL + testbench
make sim_fst          # Run simulation, produce VCD → FST waveform
make compare          # Compare RTL output against model output
make clean            # Remove sim_out/
```

### Testbench (`tb_blob_detect_rle.sv`)

Multi-frame test suite. The `run_frame` task handles the full lifecycle per frame: configure HRES/VRES, load hex, start, stream pixels with random inter-beat gaps, wait for IRQ, read status + blob descriptors, write `blobs_rtl_NNNN.hex`, verify AXIS passthrough.

**Test cases:**

| Frame | Resolution | Description | Expected blobs |
|-------|-----------|-------------|---------------|
| 0000  | 1280x800  | 8 random circular blobs | 8 |
| 0001  | 1280x720  | 5 random blobs | 5 |
| 0002  | 640x400   | 10 random blobs | 10 |
| 0003  | 1280x800  | All-black (no foreground) | 0 |
| 0005  | 640x400   | Single pixel blob | 1 |
| 0006  | 1280x800  | Two blobs, gap=2 pixels (should NOT merge) | 2 |
| 0007  | 1280x800  | Two overlapping blobs (should merge to 1) | 1 |

**Per-frame verification:**
- Blob count matches expected value
- All 7 descriptor fields (count, sum_x, sum_y, xmin, xmax, ymin, ymax) verified bit-exact against Python model via `compare_all.py`
- AXIS passthrough: every output word checked against input
- IRQ fires and clears correctly
- STATUS.FRAME_DONE bit set

**Stress features:**
- LFSR-driven random downstream backpressure (`m_axis_tready`)
- Random inter-beat gaps on input stream
- Back-to-back frames with resolution changes (tests internal state clearing)
- 200 ms watchdog timeout

### Visualization

```bash
source mocap_env/bin/activate
python helper_scripts/blob_detect_rle_verify.py --seed 42 --num-blobs 8

# Model-only (no xsim):
python helper_scripts/blob_detect_rle_verify.py --seed 42 --no-xsim

# Custom resolution:
python helper_scripts/blob_detect_rle_verify.py --width 640 --height 400 --num-blobs 5
```

Produces a 3-panel matplotlib figure: input frame (top), model results (bottom-left), RTL results (bottom-right). Centroids marked as red X's, bounding boxes in green.
