// blob_detect_grid_top.sv
// Top-level for the grid-based blob detection IP.
//
// Processing phases:
//   ST_IDLE       – waiting for CTRL.START
//   ST_CLEAR      – pulse clear to cell_accumulator and blob_emitter
//   ST_WAIT_SOF   – wait for first TUSER beat (start of frame)
//   ST_PROCESS    – stream pixels through cell_accumulator; passthrough to output
//   ST_SCAN       – grid_scanner flood-fills cells into blobs
//   ST_EMIT       – blob_emitter is draining (scanner feeds it); wait for done
//   ST_DONE       – set FRAME_DONE, raise IRQ, return to IDLE
//
// AXIS passthrough: pixels enter the input FIFO. The cell_accumulator reads
// from the FIFO. The same beat is simultaneously written to the output FIFO.
// Both FIFOs share the same pop event (beat consumed by accumulator = written
// to output FIFO in the same cycle).

`timescale 1ns / 1ps

import blob_detect_grid_regs_pkg::*;

module blob_detect_grid_top #(
    parameter int MAX_BLOBS      = 128,
    parameter int CELL_W         = 32,
    parameter int CELL_H         = 32,
    parameter int MAX_CELLS      = 2048,
    parameter int AXIS_DATA_WIDTH = 32
)(
    // -------------------------------------------------------------------------
    // Clock / reset
    // -------------------------------------------------------------------------
    input  wire         aclk,
    input  wire         aresetn,

    // -------------------------------------------------------------------------
    // AXI4-Lite Slave (6-bit address, matches PeakRDL output)
    // -------------------------------------------------------------------------
    input  wire [5:0]   s_axi_awaddr,
    input  wire [2:0]   s_axi_awprot,
    input  wire         s_axi_awvalid,
    output wire         s_axi_awready,

    input  wire [31:0]  s_axi_wdata,
    input  wire [3:0]   s_axi_wstrb,
    input  wire         s_axi_wvalid,
    output wire         s_axi_wready,

    output wire [1:0]   s_axi_bresp,
    output wire         s_axi_bvalid,
    input  wire         s_axi_bready,

    input  wire [5:0]   s_axi_araddr,
    input  wire [2:0]   s_axi_arprot,
    input  wire         s_axi_arvalid,
    output wire         s_axi_arready,

    output wire [31:0]  s_axi_rdata,
    output wire [1:0]   s_axi_rresp,
    output wire         s_axi_rvalid,
    input  wire         s_axi_rready,

    // -------------------------------------------------------------------------
    // AXI-Stream Slave (pixel input)
    // -------------------------------------------------------------------------
    input  wire [AXIS_DATA_WIDTH-1:0] s_axis_tdata,
    input  wire [0:0]                 s_axis_tuser,
    input  wire                       s_axis_tlast,
    input  wire                       s_axis_tvalid,
    output wire                       s_axis_tready,

    // -------------------------------------------------------------------------
    // AXI-Stream Master (pixel passthrough)
    // -------------------------------------------------------------------------
    output wire [AXIS_DATA_WIDTH-1:0] m_axis_tdata,
    output wire [3:0]                 m_axis_tkeep,
    output wire [0:0]                 m_axis_tuser,
    output wire                       m_axis_tlast,
    output wire                       m_axis_tvalid,
    input  wire                       m_axis_tready,

    // -------------------------------------------------------------------------
    // Interrupt
    // -------------------------------------------------------------------------
    output wire         frame_done_irq_o
);

    localparam int BLOB_W = $clog2(MAX_BLOBS);
    localparam int ADDR_W = $clog2(MAX_CELLS);

    // =========================================================================
    // Register block
    // =========================================================================
    blob_detect_grid_regs__in_t  hwif_in;
    blob_detect_grid_regs__out_t hwif_out;

    blob_detect_grid_regs u_regs (
        .clk              (aclk),
        .rst_n            (aresetn),

        .s_axil_awaddr    (s_axi_awaddr),
        .s_axil_awprot    (s_axi_awprot),
        .s_axil_awvalid   (s_axi_awvalid),
        .s_axil_awready   (s_axi_awready),

        .s_axil_wdata     (s_axi_wdata),
        .s_axil_wstrb     (s_axi_wstrb),
        .s_axil_wvalid    (s_axi_wvalid),
        .s_axil_wready    (s_axi_wready),

        .s_axil_bresp     (s_axi_bresp),
        .s_axil_bvalid    (s_axi_bvalid),
        .s_axil_bready    (s_axi_bready),

        .s_axil_araddr    (s_axi_araddr),
        .s_axil_arprot    (s_axi_arprot),
        .s_axil_arvalid   (s_axi_arvalid),
        .s_axil_arready   (s_axi_arready),

        .s_axil_rdata     (s_axi_rdata),
        .s_axil_rresp     (s_axi_rresp),
        .s_axil_rvalid    (s_axi_rvalid),
        .s_axil_rready    (s_axi_rready),

        .hwif_in          (hwif_in),
        .hwif_out         (hwif_out)
    );

    // =========================================================================
    // Convenience aliases from register block
    // =========================================================================
    wire        rst_all   = hwif_out.CTRL.RESET.value;
    wire        cfg_start = hwif_out.CTRL.START.value;
    wire        irq_clear = hwif_out.CTRL.IRQ_CLEAR.value;
    wire [7:0]  threshold = hwif_out.CTRL.THRESHOLD.value;
    wire [15:0] hres      = hwif_out.HRES.HRES.value;
    wire [15:0] vres      = hwif_out.VRES.VRES.value;
    wire        autoinc   = hwif_out.CTRL.BLOB_ADDR_AUTOINC.value;
    wire [7:0]  blob_rd_addr = hwif_out.BLOB_ADDR.BLOB_ADDR.value;

    // =========================================================================
    // Input FIFO (pixel data + sideband)
    // Width: 32 data + 1 tuser + 1 tlast = 34 bits
    // =========================================================================
    localparam int FIFO_W = AXIS_DATA_WIDTH + 1 + 1;  // data + tuser + tlast

    logic              in_fifo_s_valid, in_fifo_s_ready;
    logic [FIFO_W-1:0] in_fifo_s_data;
    logic              in_fifo_m_valid, in_fifo_m_ready;
    logic [FIFO_W-1:0] in_fifo_m_data;
    logic              in_fifo_empty;

    assign in_fifo_s_valid = s_axis_tvalid;
    assign s_axis_tready   = in_fifo_s_ready;
    assign in_fifo_s_data  = {s_axis_tuser, s_axis_tlast, s_axis_tdata};

    stream_fifo #(
        .DATA_WIDTH (FIFO_W),
        .DEPTH      (16)
    ) u_in_fifo (
        .clk     (aclk),
        .rst_n   (aresetn),
        .s_valid (in_fifo_s_valid),
        .s_ready (in_fifo_s_ready),
        .s_data  (in_fifo_s_data),
        .m_valid (in_fifo_m_valid),
        .m_ready (in_fifo_m_ready),
        .m_data  (in_fifo_m_data),
        .empty   (in_fifo_empty)
    );

    wire [AXIS_DATA_WIDTH-1:0] in_pix_data  = in_fifo_m_data[AXIS_DATA_WIDTH-1:0];
    wire                       in_pix_tlast = in_fifo_m_data[AXIS_DATA_WIDTH];
    wire [0:0]                 in_pix_tuser = in_fifo_m_data[AXIS_DATA_WIDTH+1];

    // =========================================================================
    // Output FIFO (pixel passthrough)
    // =========================================================================
    logic              out_fifo_s_valid, out_fifo_s_ready;
    logic [FIFO_W-1:0] out_fifo_s_data;
    logic              out_fifo_m_valid, out_fifo_m_ready;
    logic [FIFO_W-1:0] out_fifo_m_data;
    logic              out_fifo_empty;

    stream_fifo #(
        .DATA_WIDTH (FIFO_W),
        .DEPTH      (16)
    ) u_out_fifo (
        .clk     (aclk),
        .rst_n   (aresetn),
        .s_valid (out_fifo_s_valid),
        .s_ready (out_fifo_s_ready),
        .s_data  (out_fifo_s_data),
        .m_valid (out_fifo_m_valid),
        .m_ready (out_fifo_m_ready),
        .m_data  (out_fifo_m_data),
        .empty   (out_fifo_empty)
    );

    assign m_axis_tdata    = out_fifo_m_data[AXIS_DATA_WIDTH-1:0];
    assign m_axis_tlast    = out_fifo_m_data[AXIS_DATA_WIDTH];
    assign m_axis_tuser    = out_fifo_m_data[AXIS_DATA_WIDTH+1];
    assign m_axis_tkeep    = 4'hF;
    assign m_axis_tvalid   = out_fifo_m_valid;
    assign out_fifo_m_ready = m_axis_tready;

    // =========================================================================
    // Cell accumulator
    // =========================================================================
    logic        acc_enable, acc_clear;
    logic        acc_frame_done;
    logic        acc_overflow;
    logic [10:0] grid_cols, grid_rows;

    logic [ADDR_W-1:0] scan_bram_addr;
    logic [79:0]       scan_bram_data;

    // Accumulator consumes from input FIFO
    logic acc_s_ready;

    cell_accumulator #(
        .CELL_W    (CELL_W),
        .CELL_H    (CELL_H),
        .MAX_CELLS (MAX_CELLS)
    ) u_cell_acc (
        .clk        (aclk),
        .rst_n      (aresetn),
        .enable     (acc_enable),
        .clear      (acc_clear),
        .threshold  (threshold),
        .hres       (hres),
        .vres       (vres),
        .s_data     (in_pix_data),
        .s_valid    (in_fifo_m_valid),
        .s_ready    (acc_s_ready),
        .frame_done (acc_frame_done),
        .overflow   (acc_overflow),
        .grid_cols  (grid_cols),
        .grid_rows  (grid_rows),
        .portb_addr (scan_bram_addr),
        .portb_data (scan_bram_data)
    );

    // The input FIFO is popped when the accumulator accepts a beat.
    // Simultaneously, that beat is written to the output FIFO.
    assign in_fifo_m_ready  = acc_s_ready && out_fifo_s_ready;
    assign out_fifo_s_valid = in_fifo_m_valid && acc_s_ready && out_fifo_s_ready;
    assign out_fifo_s_data  = in_fifo_m_data;

    // =========================================================================
    // Grid scanner
    // =========================================================================
    logic       scan_start, scan_done;
    logic       scan_out_valid, scan_out_ready;
    logic [ADDR_W-1:0] scan_out_cell_idx;
    logic [10:0]       scan_out_cell_col, scan_out_cell_row;
    logic [BLOB_W-1:0] scan_out_blob_id;
    logic [19:0]       scan_out_cell_count;
    logic [31:0]       scan_out_cell_sum_x;
    logic [27:0]       scan_out_cell_sum_y;
    logic [BLOB_W-1:0] scan_blob_count;
    logic              scan_overflow;

    grid_scanner #(
        .MAX_CELLS   (MAX_CELLS),
        .MAX_BLOBS   (MAX_BLOBS),
        .STACK_DEPTH (64)
    ) u_grid_scanner (
        .clk            (aclk),
        .rst_n          (aresetn),
        .start          (scan_start),
        .done           (scan_done),
        .grid_cols      (grid_cols),
        .grid_rows      (grid_rows),
        .cell_addr      (scan_bram_addr),
        .cell_data      (scan_bram_data),
        .out_valid      (scan_out_valid),
        .out_ready      (scan_out_ready),
        .out_cell_idx   (scan_out_cell_idx),
        .out_cell_col   (scan_out_cell_col),
        .out_cell_row   (scan_out_cell_row),
        .out_blob_id    (scan_out_blob_id),
        .out_cell_count (scan_out_cell_count),
        .out_cell_sum_x (scan_out_cell_sum_x),
        .out_cell_sum_y (scan_out_cell_sum_y),
        .blob_count     (scan_blob_count),
        .overflow       (scan_overflow)
    );

    // =========================================================================
    // Blob emitter
    // =========================================================================
    logic       emit_clear, emit_busy;
    logic [BLOB_W-1:0] result_rd_addr_w;
    logic [159:0]      result_rd_data_w;

    assign result_rd_addr_w = BLOB_W'(blob_rd_addr);

    blob_emitter #(
        .MAX_BLOBS (MAX_BLOBS),
        .CELL_W    (CELL_W),
        .CELL_H    (CELL_H),
        .MAX_CELLS (MAX_CELLS)
    ) u_blob_emitter (
        .clk            (aclk),
        .rst_n          (aresetn),
        .clear          (emit_clear),
        .busy           (emit_busy),
        .hres           (hres),
        .vres           (vres),
        .in_valid       (scan_out_valid),
        .in_ready       (scan_out_ready),
        .in_cell_idx    (scan_out_cell_idx),
        .in_cell_col    (scan_out_cell_col),
        .in_cell_row    (scan_out_cell_row),
        .in_blob_id     (scan_out_blob_id),
        .in_cell_count  (scan_out_cell_count),
        .in_cell_sum_x  (scan_out_cell_sum_x),
        .in_cell_sum_y  (scan_out_cell_sum_y),
        .result_rd_addr (result_rd_addr_w),
        .result_rd_data (result_rd_data_w)
    );

    // =========================================================================
    // Top-level FSM
    // =========================================================================
    typedef enum logic [2:0] {
        ST_IDLE     = 3'd0,
        ST_CLEAR    = 3'd1,   // clear cell BRAM + result BRAM
        ST_WAIT_SOF = 3'd2,   // wait for TUSER (start-of-frame)
        ST_PROCESS  = 3'd3,   // accumulate pixels
        ST_SCAN     = 3'd4,   // grid_scanner running
        ST_DONE     = 3'd5    // latch results, assert IRQ
    } state_t;

    state_t state, next_state;

    // Status / output registers
    logic        frame_done_reg;
    logic        overflow_sticky;
    logic        frame_done_irq_q;
    logic [31:0] frame_cnt;
    logic [7:0]  blob_count_reg;

    assign frame_done_irq_o = frame_done_irq_q;

    // Guard: only start if hres/vres are non-zero
    wire dims_ok = (hres != 16'h0) && (vres != 16'h0);

    // -------------------------------------------------------------------------
    // FSM next-state (combinational)
    // -------------------------------------------------------------------------
    always_comb begin
        next_state = state;
        case (state)
            ST_IDLE:     if (cfg_start && dims_ok)  next_state = ST_CLEAR;
            ST_CLEAR:    if (!emit_busy)             next_state = ST_WAIT_SOF;
            ST_WAIT_SOF: if (in_fifo_m_valid && in_pix_tuser) next_state = ST_PROCESS;
            ST_PROCESS:  if (acc_frame_done)         next_state = ST_SCAN;
            ST_SCAN:     if (scan_done)              next_state = ST_DONE;
            ST_DONE:                                 next_state = ST_IDLE;
            default:                                 next_state = ST_IDLE;
        endcase
    end

    // -------------------------------------------------------------------------
    // FSM state register
    // -------------------------------------------------------------------------
    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn)     state <= ST_IDLE;
        else if (rst_all) state <= ST_IDLE;
        else              state <= next_state;
    end

    // -------------------------------------------------------------------------
    // Control signal derivation
    // -------------------------------------------------------------------------
    // acc_enable: high during WAIT_SOF (to track x/y from first beat) and PROCESS
    assign acc_enable = (state == ST_WAIT_SOF) || (state == ST_PROCESS);

    // acc_clear: single-cycle pulse when entering ST_CLEAR
    assign acc_clear  = (state == ST_IDLE && next_state == ST_CLEAR);

    // emit_clear: pulse at entry of ST_CLEAR
    assign emit_clear = acc_clear;

    // scan_start: pulse when transitioning from PROCESS to SCAN
    assign scan_start = (state == ST_PROCESS) && (next_state == ST_SCAN);

    // -------------------------------------------------------------------------
    // frame_done / overflow sticky / frame_cnt / blob_count
    // -------------------------------------------------------------------------
    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            frame_done_reg  <= 1'b0;
            overflow_sticky <= 1'b0;
            frame_cnt       <= 32'h0;
            blob_count_reg  <= 8'h0;
        end else if (rst_all) begin
            frame_done_reg  <= 1'b0;
            overflow_sticky <= 1'b0;
            // frame_cnt not reset by CTRL.RESET in this design (keep running)
            blob_count_reg  <= 8'h0;
        end else begin
            // frame_done asserted for 1 cycle when entering ST_DONE
            frame_done_reg <= (state == ST_SCAN && next_state == ST_DONE);

            // overflow: sticky from accumulator or scanner
            if (acc_overflow || scan_overflow)
                overflow_sticky <= 1'b1;
            else if (state == ST_IDLE && next_state == ST_CLEAR)
                overflow_sticky <= 1'b0;

            // frame counter: increment on each frame completion
            if (state == ST_SCAN && next_state == ST_DONE)
                frame_cnt <= frame_cnt + 32'h1;

            // blob count snapshot
            if (state == ST_SCAN && next_state == ST_DONE)
                blob_count_reg <= 8'(scan_blob_count);
        end
    end

    // -------------------------------------------------------------------------
    // IRQ latch
    // -------------------------------------------------------------------------
    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn)
            frame_done_irq_q <= 1'b0;
        else if (rst_all || irq_clear)
            frame_done_irq_q <= 1'b0;
        else if (frame_done_reg)
            frame_done_irq_q <= 1'b1;
    end

    // =========================================================================
    // Register block hwif_in (all fields in one always_comb)
    // =========================================================================
    always_comb begin
        // STATUS
        hwif_in.STATUS.READY.next          = (state == ST_IDLE);
        hwif_in.STATUS.FRAME_DONE.next     = frame_done_reg;
        hwif_in.STATUS.OVERFLOW.next       = overflow_sticky;
        hwif_in.STATUS.FRAME_DONE_IRQ.next = frame_done_irq_q;
        hwif_in.STATUS.BLOB_COUNT.next     = blob_count_reg;

        // Frame counter and max blobs config
        hwif_in.FRAME_CNT.FRAME_CNT.next             = frame_cnt;
        hwif_in.MAX_BLOBS_CFG.MAX_BLOBS_CFG.next     = 16'(MAX_BLOBS);

        // BLOB_ADDR auto-increment: fire on AXI read of BLOB_COUNT_RD (0x14)
        hwif_in.BLOB_ADDR.BLOB_ADDR.next = blob_rd_addr + 8'h1;
        hwif_in.BLOB_ADDR.BLOB_ADDR.we   =
            (s_axi_arvalid & s_axi_arready & (s_axi_araddr == 6'h14)) & autoinc;

        // Blob descriptor fields: from result BRAM (1-cycle latency)
        hwif_in.BLOB_COUNT_RD.BLOB_COUNT_RD.next = result_rd_data_w[159:128];
        hwif_in.BLOB_SX.BLOB_SX.next             = result_rd_data_w[127: 96];
        hwif_in.BLOB_SY.BLOB_SY.next             = result_rd_data_w[ 95: 64];
        hwif_in.BLOB_XMIN.BLOB_XMIN.next         = result_rd_data_w[ 63: 48];
        hwif_in.BLOB_XMAX.BLOB_XMAX.next         = result_rd_data_w[ 47: 32];
        hwif_in.BLOB_YMIN.BLOB_YMIN.next         = result_rd_data_w[ 31: 16];
        hwif_in.BLOB_YMAX.BLOB_YMAX.next         = result_rd_data_w[ 15:  0];
    end

endmodule
