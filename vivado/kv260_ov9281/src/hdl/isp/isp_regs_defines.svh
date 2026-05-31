// isp_regs_defines.svh
// Single source of truth for KV260 ISP register-map offsets, field positions,
// field widths, and reset defaults. Mirrors isp_regs.rdl (the authoritative
// hardware spec) so the RTL and testbenches never duplicate the address map
// in-line. Include this file from any SV module or testbench that references
// these registers.
//
// Keep in sync with isp_regs.rdl. Any new register added in the RDL should
// also be added here.

`ifndef ISP_REGS_DEFINES_SVH
`define ISP_REGS_DEFINES_SVH

// -----------------------------------------------------------------------------
// Register address offsets (AXI4-Lite byte addresses, 11-bit region)
// -----------------------------------------------------------------------------
`define ISP_REG_CTRL            11'h000
`define ISP_REG_STATUS          11'h004
`define ISP_REG_HRES            11'h008
`define ISP_REG_VRES            11'h00C
`define ISP_REG_CYCLE_CNT_LO    11'h010
`define ISP_REG_CYCLE_CNT_HI    11'h014
`define ISP_REG_CYCLE_SNAP_LO   11'h018
`define ISP_REG_CYCLE_SNAP_HI   11'h01C
`define ISP_REG_FRAME_CNT       11'h020
`define ISP_REG_FRAME_SNAP      11'h024
`define ISP_REG_PIXEL_SUM       11'h028
`define ISP_REG_HIST_ADDR       11'h02C
`define ISP_REG_HIST_DATA       11'h030

// -----------------------------------------------------------------------------
// CTRL (0x00) bit positions
//   - RESET / HISTOGRAM_START / SNAPSHOT / FRAME_CNT_RESET / CYCLE_CNT_RESET
//     are write-only singlepulse bits (auto-clear).
//   - HIST_ADDR_AUTOINC is sticky R/W (default 1).
// -----------------------------------------------------------------------------
`define ISP_CTRL_RESET_B             0
`define ISP_CTRL_HISTOGRAM_START_B   1
`define ISP_CTRL_SNAPSHOT_B          2
`define ISP_CTRL_HIST_ADDR_AUTOINC_B 3
`define ISP_CTRL_FRAME_CNT_RESET_B   4
`define ISP_CTRL_CYCLE_CNT_RESET_B   5
`define ISP_CTRL_IRQ_CLEAR_B         6

// One-hot masks for convenience
`define ISP_CTRL_RESET             (32'h1 << `ISP_CTRL_RESET_B)
`define ISP_CTRL_HISTOGRAM_START   (32'h1 << `ISP_CTRL_HISTOGRAM_START_B)
`define ISP_CTRL_SNAPSHOT          (32'h1 << `ISP_CTRL_SNAPSHOT_B)
`define ISP_CTRL_HIST_ADDR_AUTOINC (32'h1 << `ISP_CTRL_HIST_ADDR_AUTOINC_B)
`define ISP_CTRL_FRAME_CNT_RESET   (32'h1 << `ISP_CTRL_FRAME_CNT_RESET_B)
`define ISP_CTRL_CYCLE_CNT_RESET   (32'h1 << `ISP_CTRL_CYCLE_CNT_RESET_B)
`define ISP_CTRL_IRQ_CLEAR         (32'h1 << `ISP_CTRL_IRQ_CLEAR_B)

// CTRL reset value (only HIST_ADDR_AUTOINC is sticky R/W with default 1)
`define ISP_CTRL_RESET_VALUE       `ISP_CTRL_HIST_ADDR_AUTOINC

// -----------------------------------------------------------------------------
// STATUS (0x04) bit positions (all RO, HW-driven)
//   - READY           : 1 when idle and ready to accept HISTOGRAM_START
//   - HIST_DATA_VALID : 1 when last measurement results are valid
//   - HIST_FIFO_ERR   : sticky; set on histogram FIFO overflow
// -----------------------------------------------------------------------------
`define ISP_STATUS_READY_B           0
`define ISP_STATUS_HIST_DATA_VALID_B 1
`define ISP_STATUS_HIST_FIFO_ERR_B   2

`define ISP_STATUS_READY             (32'h1 << `ISP_STATUS_READY_B)
`define ISP_STATUS_HIST_DATA_VALID   (32'h1 << `ISP_STATUS_HIST_DATA_VALID_B)
`define ISP_STATUS_HIST_FIFO_ERR     (32'h1 << `ISP_STATUS_HIST_FIFO_ERR_B)

`define ISP_STATUS_FRAME_DONE_IRQ_B  3
`define ISP_STATUS_FRAME_DONE_IRQ    (32'h1 << `ISP_STATUS_FRAME_DONE_IRQ_B)

// -----------------------------------------------------------------------------
// Field widths / reset defaults
// -----------------------------------------------------------------------------
`define ISP_HRES_WIDTH        16
`define ISP_VRES_WIDTH        16
`define ISP_HRES_RESET        16'd1280
`define ISP_VRES_RESET        16'd800

`define ISP_HIST_ADDR_WIDTH   8
`define ISP_HIST_DATA_WIDTH   20

`endif // ISP_REGS_DEFINES_SVH
