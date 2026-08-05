// mocap_regs_defines.svh
// Single source of truth for mocap_wrapper register-map offsets, field positions,
// widths, and reset defaults. Mirrors mocap_regs.rdl (the authoritative spec) so
// RTL and testbenches never duplicate the address map in-line. Include from any
// SV module or testbench that references these registers.
//
// Keep in sync with mocap_regs.rdl. The generated regblock uses a 7-bit AXI4-Lite
// byte address (region size 0x68).

`ifndef MOCAP_REGS_DEFINES_SVH
`define MOCAP_REGS_DEFINES_SVH

// -----------------------------------------------------------------------------
// Register address offsets (AXI4-Lite byte addresses, 7-bit region)
// -----------------------------------------------------------------------------
`define MOCAP_REG_CTRL           7'h00
`define MOCAP_REG_STATUS         7'h04
`define MOCAP_REG_HRES           7'h08
`define MOCAP_REG_VRES           7'h0C
`define MOCAP_REG_FRAME_ID       7'h10
`define MOCAP_REG_DROPPED_FRAMES 7'h14
`define MOCAP_REG_PIXEL_SUM      7'h18
`define MOCAP_REG_HIST_ADDR      7'h20
`define MOCAP_REG_HIST_DATA      7'h24
`define MOCAP_REG_BLOB_ADDR      7'h28
`define MOCAP_REG_BLOB_COUNT_RD  7'h2C
`define MOCAP_REG_BLOB_SX        7'h30
`define MOCAP_REG_BLOB_SY        7'h34
`define MOCAP_REG_BLOB_XMIN      7'h38
`define MOCAP_REG_BLOB_XMAX      7'h3C
`define MOCAP_REG_BLOB_YMIN      7'h40
`define MOCAP_REG_BLOB_YMAX      7'h44
`define MOCAP_REG_MAX_BLOBS_CFG  7'h48
`define MOCAP_REG_DMA_BASE_LO    7'h50
`define MOCAP_REG_DMA_BASE_HI    7'h54
`define MOCAP_REG_DMA_LEN        7'h58
`define MOCAP_REG_DMA_CTRL       7'h5C
`define MOCAP_REG_CYCLE_SNAP_LO  7'h60
`define MOCAP_REG_CYCLE_SNAP_HI  7'h64

// -----------------------------------------------------------------------------
// CTRL (0x00) bit positions
//   - RESET / RESULTS_ACK are write-only singlepulse (auto-clear).
//   - ENABLE / *_AUTOINC / THRESHOLD are sticky R/W.
// -----------------------------------------------------------------------------
`define MOCAP_CTRL_RESET_B             0
`define MOCAP_CTRL_ENABLE_B            1
`define MOCAP_CTRL_RESULTS_ACK_B       2
`define MOCAP_CTRL_HIST_ADDR_AUTOINC_B 3
`define MOCAP_CTRL_BLOB_ADDR_AUTOINC_B 4
`define MOCAP_CTRL_CYCLE_SNAPSHOT_B    5
`define MOCAP_CTRL_THRESHOLD_LSB       8
`define MOCAP_CTRL_THRESHOLD_WIDTH     8

// One-hot masks for convenience
`define MOCAP_CTRL_RESET             (32'h1 << `MOCAP_CTRL_RESET_B)
`define MOCAP_CTRL_ENABLE            (32'h1 << `MOCAP_CTRL_ENABLE_B)
`define MOCAP_CTRL_RESULTS_ACK       (32'h1 << `MOCAP_CTRL_RESULTS_ACK_B)
`define MOCAP_CTRL_HIST_ADDR_AUTOINC (32'h1 << `MOCAP_CTRL_HIST_ADDR_AUTOINC_B)
`define MOCAP_CTRL_BLOB_ADDR_AUTOINC (32'h1 << `MOCAP_CTRL_BLOB_ADDR_AUTOINC_B)
`define MOCAP_CTRL_CYCLE_SNAPSHOT    (32'h1 << `MOCAP_CTRL_CYCLE_SNAPSHOT_B)
`define MOCAP_CTRL_THRESHOLD(v)      ((32'h0 | ((v) & 32'hFF)) << `MOCAP_CTRL_THRESHOLD_LSB)

// CTRL reset value (sticky bits only: both AUTOINCs = 1, THRESHOLD = 128)
`define MOCAP_CTRL_RESET_VALUE \
    (`MOCAP_CTRL_HIST_ADDR_AUTOINC | `MOCAP_CTRL_BLOB_ADDR_AUTOINC | `MOCAP_CTRL_THRESHOLD(8'd128))

// -----------------------------------------------------------------------------
// STATUS (0x04) bit positions (all RO, HW-driven)
// -----------------------------------------------------------------------------
`define MOCAP_STATUS_READY_B          0
`define MOCAP_STATUS_RESULTS_VALID_B  1
`define MOCAP_STATUS_FRAME_DONE_IRQ_B 2
`define MOCAP_STATUS_READ_BANK_B      3
`define MOCAP_STATUS_HIST_FIFO_ERR_B  4
`define MOCAP_STATUS_BLOB_OVERFLOW_B  5
`define MOCAP_STATUS_OVERRUN_B        6
`define MOCAP_STATUS_BLOB_COUNT_LSB   8
`define MOCAP_STATUS_BLOB_COUNT_WIDTH 8

`define MOCAP_STATUS_READY          (32'h1 << `MOCAP_STATUS_READY_B)
`define MOCAP_STATUS_RESULTS_VALID  (32'h1 << `MOCAP_STATUS_RESULTS_VALID_B)
`define MOCAP_STATUS_FRAME_DONE_IRQ (32'h1 << `MOCAP_STATUS_FRAME_DONE_IRQ_B)
`define MOCAP_STATUS_READ_BANK      (32'h1 << `MOCAP_STATUS_READ_BANK_B)
`define MOCAP_STATUS_HIST_FIFO_ERR  (32'h1 << `MOCAP_STATUS_HIST_FIFO_ERR_B)
`define MOCAP_STATUS_BLOB_OVERFLOW  (32'h1 << `MOCAP_STATUS_BLOB_OVERFLOW_B)
`define MOCAP_STATUS_OVERRUN        (32'h1 << `MOCAP_STATUS_OVERRUN_B)

// Extract published blob count from a STATUS read value
`define MOCAP_STATUS_GET_BLOB_COUNT(s) (((s) >> `MOCAP_STATUS_BLOB_COUNT_LSB) & 32'hFF)

// -----------------------------------------------------------------------------
// Field widths / reset defaults
// -----------------------------------------------------------------------------
`define MOCAP_HRES_WIDTH      16
`define MOCAP_VRES_WIDTH      16
`define MOCAP_HRES_RESET      16'd640
`define MOCAP_VRES_RESET      16'd400
`define MOCAP_THRESHOLD_RESET 8'd128

`define MOCAP_HIST_ADDR_WIDTH 8
`define MOCAP_HIST_DATA_WIDTH 20
`define MOCAP_BLOB_ADDR_WIDTH 8

`endif // MOCAP_REGS_DEFINES_SVH
