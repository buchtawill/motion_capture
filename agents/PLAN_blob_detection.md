# Blob Detection IP Development Plan

## Goal
Develop two fully verified blob-detection IPs as drop-in AXIS streaming modules.
Each IP sits **in the pixel datapath** (AXIS slave in → blob processing → AXIS master out to VDMA),
with input/output FIFOs for timing isolation. Blob descriptors are reported via AXI-Lite registers + frame-done IRQ.

## Constraints

- 8-bit grayscale pixels, 32-bit AXIS data (4 pixels/beat), 200 MHz target clock
- **Variable resolution**: 1280×800, 1280×720, 640×400, etc. — HRES/VRES set via AXI-Lite registers, no hardcoded dimensions anywhere. All beat-count / grid-size logic derived dynamically from HRES×VRES.
- MAX_BLOBS = 128, defined as a SystemVerilog parameter / macro
- Register interface via PeakRDL (`.rdl` → `peakrdl regblock` → AXI4-Lite slave)
- Verification: Python functional model ↔ xsim RTL comparison using shared stimulus files
- **One Vivado/xsim process at a time** (sequential RTL verification)
- Development: `/home/will/Desktop/motion_capture/vivado/ip_repo/`
- Venv: `source /home/will/Desktop/motion_capture/mocap_env/bin/activate`
- Vivado: `/tools/Xilinx/Vivado/2024.1/bin/vivado` (xvlog/xelab/xsim on PATH after `source settings64.sh`)

## Git Workflow

- **Branch**: `feature/blob-detection` off `main`
- **Author**: `Claude <noreply@anthropic.com>` (Co-Authored-By in commit messages)
- **Remote**: `origin` → `git@github.com:buchtawill/motion_capture.git`
- **Push frequently**: after each milestone (model done, RDL done, each RTL module passing, etc.)
- Commit granularity: one commit per logical unit of work (not one giant commit at the end)

## Architecture A — RLE + Run-Merge (Streaming CCL)
Streaming connected-component labeling in 3 pipelined stages:
1. **Run Extractor**: Threshold + run-length encode each row → (xs, xe) per run
2. **Row Merger**: Match current-row runs against previous-row runs via overlap check.
   Union-find to merge blob IDs when a current run bridges two previous blobs.
3. **Blob Table**: Register-file accumulator (count, Σx, Σy, xmin, xmax, ymin, ymax).
   Emits completed blob descriptors when a blob is not touched by the next row.

**Pros**: Pixel-accurate centroids and bounding boxes. Standard algorithm.
**Cons**: Union-find merge logic is the most complex part. Worst-case blob table usage
is width/2 (alternating pixel comb pattern), but 128 entries covers all practical cases.

## Architecture B — Grid-Based Spatial Hashing
Divide image into NxN cells (e.g. 20x25 grid of 64x32-pixel cells for 1280×800).
1. **Cell Accumulator**: As pixels stream in, increment cell's foreground count and
   accumulate x/y sums. One BRAM for the grid (small: 500 cells × ~80 bits).
2. **Grid Scanner**: After frame-done, a small FSM flood-fills adjacent non-empty cells
   into blobs using a visited-bit array and a small stack/queue.
3. **Blob Emitter**: Merge cell accumulators into blob descriptors.

**Pros**: Very simple streaming datapath (no run-length encoding, no union-find).
**Cons**: Centroid accuracy limited to sub-cell interpolation. Two-pass: streaming
accumulation + post-frame scan. Grid size trades off resolution vs BRAM.

## Register Interface (both architectures, same map)
| Offset | Name           | R/W | Description |
|--------|---------------|-----|-------------|
| 0x00   | CTRL          | R/W | RESET (singlepulse), START (singlepulse), IRQ_CLEAR (singlepulse), THRESHOLD[7:0] |
| 0x04   | STATUS        | RO  | READY, FRAME_DONE, OVERFLOW, BLOB_COUNT[7:0] |
| 0x08   | HRES          | R/W | Horizontal resolution (16b, default 1280) |
| 0x0C   | VRES          | R/W | Vertical resolution (16b, default 800) |
| 0x10   | BLOB_ADDR     | R/W | Blob table index (0..MAX_BLOBS-1), auto-increment on BLOB_DATA reads |
| 0x14   | BLOB_COUNT_RD | RO  | Pixel count for blob at BLOB_ADDR |
| 0x18   | BLOB_CX       | RO  | Centroid X (fixed-point or integer sum, SW divides by count) |
| 0x1C   | BLOB_CY       | RO  | Centroid Y |
| 0x20   | BLOB_XMIN     | RO  | Bounding box min X |
| 0x24   | BLOB_XMAX     | RO  | Bounding box max X |
| 0x28   | BLOB_YMIN     | RO  | Bounding box min Y |
| 0x2C   | BLOB_YMAX     | RO  | Bounding box max Y |
| 0x30   | FRAME_CNT     | RO  | Frame counter |
| 0x34   | MAX_BLOBS_CFG | RO  | Reports MAX_BLOBS parameter value to SW |

## Execution Steps

### Step 1: Research & Confirm Algorithms ✅
Two architectures chosen above. Research agent validates feasibility.

### Step 2: Python Functional Models
- [ ] **A**: RLE Run-Merge model (refine existing `blob_detector_functional.py` into
      FIFO-primitive form with binary I/O for verification)
- [ ] **B**: Grid-based model from scratch
- [ ] Shared stimulus generator (`gen_stimulus.py`): random frames → `.bin` files
- [ ] Shared comparator (`compare.py`): read model output + RTL output → pass/fail

### Step 3: RDL Register Specs
- [ ] **A**: `blob_detect_rle_regs.rdl` + Makefile → generate RTL + headers
- [ ] **B**: `blob_detect_grid_regs.rdl` + Makefile → generate RTL + headers
- [ ] README architecture docs for each

### Step 4: RTL Implementation & Verification
- [ ] **A**: SystemVerilog modules + testbench + xsim pass
- [ ] **B**: SystemVerilog modules + testbench + xsim pass
- [ ] (Sequential — one xsim at a time)

### Step 5: Tradeoff Report
- [ ] Throughput analysis at 200 MHz
- [ ] Resource estimates (LUT, FF, BRAM)
- [ ] Accuracy comparison (pixel-accurate vs cell-granularity)
- [ ] Complexity comparison

## Progress Log

### Grid-Based (Architecture B) — COMPLETE
- [x] Directory scaffold + git branch `feature/blob-detection`
- [x] RDL register spec (`blob_detect_grid_regs.rdl`) + generated RTL/header/docs
- [x] Python functional model (gen_stimulus, model, compare) — all 8 test cases pass
- [x] RTL implementation (cell_accumulator, grid_scanner, blob_emitter, top, wrapper)
- [x] Opus sanity review — found divider and off-by-one blob limit → fixed
- [x] Testbench + xsim verification — 7 blobs match model exactly, AXIS passthrough verified
- [x] All pushed to `feature/blob-detection` branch

**Bugs found and fixed during verification:**
1. cell_accumulator overflow check wrapped to 0 (ADDR_W'(MAX_CELLS) = 0 for power-of-2)
2. FRAME_DONE was 1-cycle pulse, not sticky — SW couldn't read it
3. Testbench CTRL register bit positions (THRESHOLD at [11:4], not [15:8])
4. grid_scanner BRAM read address dropped during WAIT/CHK states — root cause of blob_count=0
5. grid_scanner used hardware dividers (% and /) — replaced with stack-carried col/row
6. Off-by-one max blob count (MAX_BLOBS-1 usable, not MAX_BLOBS)

**Known follow-up:** Sub-module FSMs should be refactored to separate always_comb/always_ff pattern.

### RLE-Based (Architecture A) — NOT STARTED
Waiting for user review of grid-based IP before starting.
