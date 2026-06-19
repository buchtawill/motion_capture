`timescale 1ns / 1ps

import blob_detect_rle_regs_pkg::*;

module blob_detect_rle_top #(
    parameter int MAX_BLOBS        = 128,
    parameter int MAX_RUNS_PER_ROW = 640,
    parameter int AXIS_DATA_WIDTH  = 32
)(
    input  wire         aclk,
    input  wire         aresetn,

    // AXI4-Lite Slave
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

    // AXI-Stream Slave (pixel input)
    input  wire [AXIS_DATA_WIDTH-1:0] s_axis_tdata,
    input  wire [0:0]                 s_axis_tuser,
    input  wire                       s_axis_tlast,
    input  wire                       s_axis_tvalid,
    output wire                       s_axis_tready,

    // AXI-Stream Master (pixel passthrough)
    output wire [AXIS_DATA_WIDTH-1:0] m_axis_tdata,
    output wire [3:0]                 m_axis_tkeep,
    output wire [0:0]                 m_axis_tuser,
    output wire                       m_axis_tlast,
    output wire                       m_axis_tvalid,
    input  wire                       m_axis_tready,

    output wire         frame_done_irq_o
);

    localparam int BLOB_W = $clog2(MAX_BLOBS);

    // =========================================================================
    // Register block
    // =========================================================================
    blob_detect_rle_regs__in_t  hwif_in;
    blob_detect_rle_regs__out_t hwif_out;

    blob_detect_rle_regs u_regs (
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
    // Register aliases
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
    // Input FIFO
    // =========================================================================
    localparam int FIFO_W = AXIS_DATA_WIDTH + 1 + 1;

    logic              in_fifo_s_valid, in_fifo_s_ready;
    logic [FIFO_W-1:0] in_fifo_s_data;
    logic              in_fifo_m_valid, in_fifo_m_ready;
    logic [FIFO_W-1:0] in_fifo_m_data;
    logic              in_fifo_empty;

    assign in_fifo_s_valid = s_axis_tvalid;
    assign s_axis_tready   = in_fifo_s_ready;
    assign in_fifo_s_data  = {s_axis_tuser, s_axis_tlast, s_axis_tdata};

    stream_fifo #(.DATA_WIDTH(FIFO_W), .DEPTH(16)) u_in_fifo (
        .clk(aclk), .rst_n(aresetn),
        .s_valid(in_fifo_s_valid), .s_ready(in_fifo_s_ready), .s_data(in_fifo_s_data),
        .m_valid(in_fifo_m_valid), .m_ready(in_fifo_m_ready), .m_data(in_fifo_m_data),
        .empty(in_fifo_empty)
    );

    wire [AXIS_DATA_WIDTH-1:0] in_pix_data  = in_fifo_m_data[AXIS_DATA_WIDTH-1:0];
    wire                       in_pix_tlast = in_fifo_m_data[AXIS_DATA_WIDTH];
    wire [0:0]                 in_pix_tuser = in_fifo_m_data[AXIS_DATA_WIDTH+1];

    // =========================================================================
    // Output FIFO
    // =========================================================================
    logic              out_fifo_s_valid, out_fifo_s_ready;
    logic [FIFO_W-1:0] out_fifo_s_data;
    logic              out_fifo_m_valid, out_fifo_m_ready;
    logic [FIFO_W-1:0] out_fifo_m_data;
    logic              out_fifo_empty;

    stream_fifo #(.DATA_WIDTH(FIFO_W), .DEPTH(16)) u_out_fifo (
        .clk(aclk), .rst_n(aresetn),
        .s_valid(out_fifo_s_valid), .s_ready(out_fifo_s_ready), .s_data(out_fifo_s_data),
        .m_valid(out_fifo_m_valid), .m_ready(out_fifo_m_ready), .m_data(out_fifo_m_data),
        .empty(out_fifo_empty)
    );

    assign m_axis_tdata     = out_fifo_m_data[AXIS_DATA_WIDTH-1:0];
    assign m_axis_tlast     = out_fifo_m_data[AXIS_DATA_WIDTH];
    assign m_axis_tuser     = out_fifo_m_data[AXIS_DATA_WIDTH+1];
    assign m_axis_tkeep     = 4'hF;
    assign m_axis_tvalid    = out_fifo_m_valid;
    assign out_fifo_m_ready = m_axis_tready;

    // =========================================================================
    // Run extractor
    // =========================================================================
    logic        re_enable, re_clear;
    logic        re_s_ready;
    logic        re_frame_done;

    logic [15:0] re_run_xs, re_run_xe, re_run_row;
    logic        re_run_last, re_run_valid, re_run_ready;

    run_extractor u_run_ext (
        .clk       (aclk),
        .rst_n     (aresetn),
        .enable    (re_enable),
        .clear     (re_clear),
        .threshold (threshold),
        .hres      (hres),
        .vres      (vres),
        .s_data    (in_pix_data),
        .s_valid   (in_fifo_m_valid && out_fifo_s_ready),
        .s_ready   (re_s_ready),
        .run_xs    (re_run_xs),
        .run_xe    (re_run_xe),
        .run_row   (re_run_row),
        .run_last_in_row (re_run_last),
        .run_valid (re_run_valid),
        .run_ready (re_run_ready),
        .frame_done(re_frame_done)
    );

    // Passthrough: pop input FIFO when run_extractor accepts AND output FIFO has space
    assign in_fifo_m_ready  = re_s_ready && out_fifo_s_ready;
    assign out_fifo_s_valid = in_fifo_m_valid && re_s_ready && out_fifo_s_ready;
    assign out_fifo_s_data  = in_fifo_m_data;

    // =========================================================================
    // Row merger
    // =========================================================================
    logic        rm_enable, rm_clear;
    logic [15:0] rm_out_xs, rm_out_xe, rm_out_row;
    logic [6:0]  rm_out_blob_id;
    logic        rm_out_is_new, rm_out_valid, rm_out_ready;
    logic        rm_merge_valid, rm_merge_ready;
    logic [6:0]  rm_merge_a, rm_merge_b;
    logic        rm_overflow;

    row_merger #(
        .MAX_BLOBS        (MAX_BLOBS),
        .MAX_RUNS_PER_ROW (MAX_RUNS_PER_ROW)
    ) u_row_merger (
        .clk       (aclk),
        .rst_n     (aresetn),
        .enable    (rm_enable),
        .clear     (rm_clear),
        .in_xs     (re_run_xs),
        .in_xe     (re_run_xe),
        .in_row    (re_run_row),
        .in_last_in_row (re_run_last),
        .in_valid  (re_run_valid),
        .in_ready  (re_run_ready),
        .out_xs    (rm_out_xs),
        .out_xe    (rm_out_xe),
        .out_row   (rm_out_row),
        .out_blob_id (rm_out_blob_id),
        .out_is_new(rm_out_is_new),
        .out_valid (rm_out_valid),
        .out_ready (rm_out_ready),
        .merge_valid (rm_merge_valid),
        .merge_a   (rm_merge_a),
        .merge_b   (rm_merge_b),
        .merge_ready (rm_merge_ready),
        .overflow  (rm_overflow)
    );

    // =========================================================================
    // Blob table
    // =========================================================================
    logic        bt_clear, bt_flatten_start, bt_flatten_done;
    logic [6:0]  bt_blob_count;
    logic [159:0] result_rd_data_w;

    blob_table #(.MAX_BLOBS(MAX_BLOBS)) u_blob_table (
        .clk            (aclk),
        .rst_n          (aresetn),
        .clear          (bt_clear),
        .flatten_start  (bt_flatten_start),
        .in_xs          (rm_out_xs),
        .in_xe          (rm_out_xe),
        .in_row         (rm_out_row),
        .in_blob_id     (rm_out_blob_id),
        .in_is_new      (rm_out_is_new),
        .in_valid       (rm_out_valid),
        .in_ready       (rm_out_ready),
        .merge_valid    (rm_merge_valid),
        .merge_a        (rm_merge_a),
        .merge_b        (rm_merge_b),
        .merge_ready    (rm_merge_ready),
        .flatten_done   (bt_flatten_done),
        .blob_count     (bt_blob_count),
        .result_rd_addr (blob_rd_addr[6:0]),
        .result_rd_data (result_rd_data_w)
    );

    // =========================================================================
    // Top-level FSM
    // =========================================================================
    typedef enum logic [2:0] {
        ST_IDLE     = 3'd0,
        ST_CLEAR    = 3'd1,
        ST_WAIT_SOF = 3'd2,
        ST_PROCESS  = 3'd3,
        ST_FINALIZE = 3'd4,
        ST_DONE     = 3'd5
    } state_t;

    state_t state, next_state;

    logic        frame_done_reg;
    logic        overflow_sticky;
    logic        frame_done_irq_q;
    logic [31:0] frame_cnt;
    logic [7:0]  blob_count_reg;

    assign frame_done_irq_o = frame_done_irq_q;

    wire dims_ok = (hres != 16'h0) && (vres != 16'h0);

    // -------------------------------------------------------------------------
    // Next-state logic
    // -------------------------------------------------------------------------
    always_comb begin
        next_state = state;
        case (state)
            ST_IDLE:     if (cfg_start && dims_ok)                    next_state = ST_CLEAR;
            ST_CLEAR:    next_state = ST_WAIT_SOF;
            ST_WAIT_SOF: if (in_fifo_m_valid && in_pix_tuser)        next_state = ST_PROCESS;
            ST_PROCESS:  if (re_frame_done)                           next_state = ST_FINALIZE;
            ST_FINALIZE: if (bt_flatten_done)                         next_state = ST_DONE;
            ST_DONE:     next_state = ST_IDLE;
            default:     next_state = ST_IDLE;
        endcase
    end

    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn)     state <= ST_IDLE;
        else if (rst_all) state <= ST_IDLE;
        else              state <= next_state;
    end

    // -------------------------------------------------------------------------
    // Control signals
    // -------------------------------------------------------------------------
    assign re_enable = (state == ST_WAIT_SOF) || (state == ST_PROCESS);
    assign re_clear  = (state == ST_IDLE && next_state == ST_CLEAR);

    assign rm_enable = (state == ST_PROCESS) || (state == ST_FINALIZE);
    assign rm_clear  = re_clear;

    assign bt_clear         = re_clear;
    assign bt_flatten_start = (state == ST_PROCESS && next_state == ST_FINALIZE);

    // -------------------------------------------------------------------------
    // Status registers
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
            blob_count_reg  <= 8'h0;
        end else begin
            if (state == ST_FINALIZE && next_state == ST_DONE)
                frame_done_reg <= 1'b1;
            else if (state == ST_IDLE && next_state == ST_CLEAR)
                frame_done_reg <= 1'b0;

            if (rm_overflow)
                overflow_sticky <= 1'b1;
            else if (state == ST_IDLE && next_state == ST_CLEAR)
                overflow_sticky <= 1'b0;

            if (state == ST_FINALIZE && next_state == ST_DONE)
                frame_cnt <= frame_cnt + 32'h1;

            if (state == ST_FINALIZE && next_state == ST_DONE)
                blob_count_reg <= 8'(bt_blob_count);
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
        else if (state == ST_FINALIZE && next_state == ST_DONE)
            frame_done_irq_q <= 1'b1;
    end

    // =========================================================================
    // Register hwif_in
    // =========================================================================
    always_comb begin
        hwif_in.STATUS.READY.next          = (state == ST_IDLE);
        hwif_in.STATUS.FRAME_DONE.next     = frame_done_reg;
        hwif_in.STATUS.OVERFLOW.next       = overflow_sticky;
        hwif_in.STATUS.FRAME_DONE_IRQ.next = frame_done_irq_q;
        hwif_in.STATUS.BLOB_COUNT.next     = blob_count_reg;

        hwif_in.FRAME_CNT.FRAME_CNT.next         = frame_cnt;
        hwif_in.MAX_BLOBS_CFG.MAX_BLOBS_CFG.next  = 16'(MAX_BLOBS);

        hwif_in.BLOB_ADDR.BLOB_ADDR.next = blob_rd_addr + 8'h1;
        hwif_in.BLOB_ADDR.BLOB_ADDR.we   =
            (s_axi_arvalid & s_axi_arready & (s_axi_araddr == 6'h14)) & autoinc;

        hwif_in.BLOB_COUNT_RD.BLOB_COUNT_RD.next = result_rd_data_w[159:128];
        hwif_in.BLOB_SX.BLOB_SX.next             = result_rd_data_w[127: 96];
        hwif_in.BLOB_SY.BLOB_SY.next             = result_rd_data_w[ 95: 64];
        hwif_in.BLOB_XMIN.BLOB_XMIN.next         = result_rd_data_w[ 63: 48];
        hwif_in.BLOB_XMAX.BLOB_XMAX.next         = result_rd_data_w[ 47: 32];
        hwif_in.BLOB_YMIN.BLOB_YMIN.next         = result_rd_data_w[ 31: 16];
        hwif_in.BLOB_YMAX.BLOB_YMAX.next         = result_rd_data_w[ 15:  0];
    end

endmodule
