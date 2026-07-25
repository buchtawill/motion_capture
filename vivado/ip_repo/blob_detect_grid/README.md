# blob_detect_grid — Grid-Based Spatial Hashing Blob Detection IP

> **Status: Experimental.** This architecture was evaluated as sub-par compared to the RLE streaming CCL approach ([`blob_detect_rle`](../blob_detect_rle/)). The grid quantization limits centroid accuracy to cell granularity (32x32 pixels vs pixel-exact), and the post-frame DFS scan takes thousands of cycles vs ~128 for the RLE flatten. Retained for reference only. **Use `blob_detect_rle` for production.**

Grid-based blob detector for 8-bit grayscale AXIS pixel streams. Divides the image into a grid of fixed-size cells, accumulates foreground pixel statistics per cell during streaming, then runs a post-frame DFS flood-fill to group adjacent non-empty cells into blobs. Reports up to 128 blobs per frame via AXI-Lite registers and a frame-done IRQ.

## Directory layout

```
blob_detect_grid/
├── hdl/
│   ├── cell_accumulator.sv        # Stage 1: pixel-to-cell accumulation
│   ├── grid_scanner.sv            # Stage 2: DFS flood-fill on cell grid
│   ├── blob_emitter.sv            # Stage 3: per-blob descriptor aggregation
│   ├── blob_detect_grid_top.sv    # Top-level FSM, FIFOs, register wiring
│   ├── blob_detect_grid_wrapper.v # Verilog-2001 wrapper for IP Integrator
│   └── stream_fifo.sv             # Reusable AXIS FIFO (shared with other IPs)
├── model/
│   ├── blob_detect_grid_model.py  # Python functional model (bit-exact reference)
│   ├── gen_stimulus.py            # Test frame generator (random circular blobs)
│   ├── compare.py                 # Single-frame RTL vs model comparator
│   └── compare_all.py             # Multi-frame batch comparator
├── rdl/
│   ├── blob_detect_grid_regs.rdl  # PeakRDL register map specification
│   ├── Makefile
│   └── rdl_out/                   # Generated RTL, C header, HTML docs
└── sim/
    ├── tb_blob_detect_grid.sv     # Single-frame xsim testbench
    ├── wave_vcd.tcl
    └── Makefile
```

## RTL module hierarchy

```
blob_detect_grid_wrapper.v         (Verilog-2001 shell for IP Integrator)
└── blob_detect_grid_top.sv        (top-level FSM + register + FIFO wiring)
    ├── blob_detect_grid_regs.sv   (PeakRDL-generated AXI4-Lite register block)
    ├── stream_fifo (u_in_fifo)    (input AXIS FIFO, depth 16)
    ├── stream_fifo (u_out_fifo)   (output AXIS FIFO, depth 16)
    ├── cell_accumulator.sv        (pixel-to-cell foreground accumulation)
    ├── grid_scanner.sv            (post-frame 4-connected DFS flood-fill)
    └── blob_emitter.sv            (per-blob descriptor aggregation)
```

### Data flow

```
AXIS slave ──► in_fifo ──┬──► cell_accumulator ──► (cell BRAM)
                         │                              │
                         └──► out_fifo ──► AXIS master  │  (pixel passthrough)
                                                        │
                              grid_scanner ◄────────────┘  (post-frame DFS)
                                   │
                              blob_emitter ──► result BRAM ──► AXI-Lite readback
```

Pixels flow through the input FIFO to the cell_accumulator (processing) and output FIFO (passthrough) simultaneously. After all pixels are consumed, the grid_scanner runs a DFS flood-fill on the cell grid, feeding results to the blob_emitter.

### Top-level FSM (`blob_detect_grid_top.sv`)

```
ST_IDLE ──(START)──► ST_CLEAR ──► ST_WAIT_SOF ──(TUSER[0])──► ST_PROCESS
                                                                    │
                                                            (all pixels consumed)
                                                                    │
                           ST_DONE ◄── ST_SCAN ◄────────────────────┘
                             │          (grid_scanner DFS + blob_emitter)
                             └──► ST_IDLE
```

- **ST_IDLE**: waits for software START pulse.
- **ST_CLEAR**: zeros cell BRAM.
- **ST_WAIT_SOF**: waits for first beat with TUSER[0]=1.
- **ST_PROCESS**: cell_accumulator runs, pixels stream through.
- **ST_SCAN**: grid_scanner DFS + blob_emitter aggregate. This is a post-frame phase that can take thousands of cycles depending on how many cells are occupied.
- **ST_DONE**: latches blob_count, asserts frame_done IRQ.

### Stage 1: cell_accumulator

Divides the image into a grid of CELL_W x CELL_H pixel cells (default 32x32). Unpacks 4 pixels per beat. For each foreground pixel (value >= threshold), does read-modify-write on an 80-bit cell BRAM entry: `count[19:0]`, `sum_x[31:0]`, `sum_y[27:0]`. Handles same-cell write hazards via forwarding register.

Parameters: MAX_CELLS=2048. Dual-port BRAM: port A for RMW during streaming, port B read-only for grid_scanner.

### Stage 2: grid_scanner

Post-frame 4-connected flood-fill blob detector. 12-state FSM with linear scan + iterative DFS using a hardware stack (register array, depth=64). Scans all cells linearly; when an unvisited non-empty cell is found, pushes it as a DFS seed and explores all 4-connected neighbors. Each cell in a connected component is emitted to blob_emitter with its blob_id and cell statistics. Tracks `blob_count` and `overflow` (too many blobs or stack exhaustion).

### Stage 3: blob_emitter

Receives per-cell blob assignments from grid_scanner and aggregates into per-blob descriptors in a 160-bit result BRAM (MAX_BLOBS=128 entries). Fields: `count[31:0]`, `sum_x[31:0]`, `sum_y[31:0]`, `xmin[15:0]`, `xmax[15:0]`, `ymin[15:0]`, `ymax[15:0]`. Computes cell bounding boxes from cell coordinates (shift-based multiplication) with image-edge clamping.

## Register interface

Same register map layout as `blob_detect_rle` (both share the same RDL structure).

| Offset | Name           | R/W | Description |
|--------|---------------|-----|-------------|
| 0x00   | CTRL          | R/W | RESET, START, IRQ_CLEAR, BLOB_ADDR_AUTOINC, THRESHOLD[7:0] |
| 0x04   | STATUS        | RO  | READY, FRAME_DONE, OVERFLOW, FRAME_DONE_IRQ, BLOB_COUNT[7:0] |
| 0x08   | HRES          | R/W | Horizontal resolution (default 1280) |
| 0x0C   | VRES          | R/W | Vertical resolution (default 800) |
| 0x10   | BLOB_ADDR     | R/W | Blob table read index |
| 0x14   | BLOB_COUNT_RD | RO  | Pixel count for blob at BLOB_ADDR |
| 0x18   | BLOB_SX       | RO  | Sum of X (divide by count for centroid) |
| 0x1C   | BLOB_SY       | RO  | Sum of Y |
| 0x20–0x2C | BLOB bbox  | RO  | xmin, xmax, ymin, ymax |
| 0x30   | FRAME_CNT     | RO  | Frames processed since reset |
| 0x34   | MAX_BLOBS_CFG | RO  | MAX_BLOBS parameter (128) |

## Python functional model

`model/blob_detect_grid_model.py` implements the same algorithm in Python with bit-exact integer arithmetic:

1. **Cell accumulation**: for each foreground pixel, increment the cell's count and coordinate sums
2. **4-connected flood-fill**: iterative DFS on the cell grid (matching RTL's stack-based approach)
3. **Blob aggregation**: merge cell statistics into per-blob descriptors with cell-granularity bounding boxes

Results sorted by `(ymin, xmin)`. `gen_stimulus.py` generates 8 test frames (random blobs, all-black, all-white, single-pixel, nearly-touching, overlapping).

## Simulation

```bash
cd sim/
make verify    # gen_stim → model → compile → sim_fst → compare
```

### Testbench (`tb_blob_detect_grid.sv`)

Single-frame integration test. Loads `frame_0000.hex`, streams 1280x800 with random inter-beat gaps, applies ~25% LFSR downstream backpressure, waits for IRQ, reads all blob descriptors via AXI-Lite (auto-increment disabled), writes `blobs_rtl.hex`, verifies AXIS passthrough (all 256000 words match input). 50ms watchdog timeout.

## Key differences from blob_detect_rle

| | blob_detect_grid | blob_detect_rle |
|---|---|---|
| Centroid accuracy | Cell granularity (32x32 px) | Pixel-exact |
| Bounding box accuracy | Cell-aligned | Pixel-exact |
| Post-frame latency | Thousands of cycles (DFS) | ~128 cycles (flatten) |
| Streaming complexity | Simple (cell RMW) | Complex (union-find + run matching) |
| Algorithm | Spatial hashing + flood-fill | RLE + streaming CCL |
