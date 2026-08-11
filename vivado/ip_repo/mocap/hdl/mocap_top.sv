// mocap_top.sv
// Motion-capture fused-pipeline core (SystemVerilog).
// Wrapped for IP Integrator by mocap_wrapper.v.
//
// Fuses the 256-bin ISP histogram and the RLE blob detector behind one AXI4-Lite
// register file with a race-free, hardware-owned double-buffered result store.
// See motion_capture/agents/PLAN_new_hw_pipeline.md (architecture) and the
// implementation plan for the ownership protocol.
//
// Datapath:
//   s_axis --+--> (snoop) 2x isp_histogram (ping-pong, one hist_en at a time)
//            |
//            +--> FIFO -> blob core (run_extractor/row_merger/blob_table,
//                 reused unmodified from blob_detect_rle) -> FIFO -> m_axis
//
//   Result store (all banking owned HERE; engines are reused unmodified):
//     - histogram: the two isp_histogram instances ARE the two banks
//     - blob:      wrapper_blob_buf[2] filled by a copy-FSM on flatten_done
//     - one shared bank_select / sw_owns / RESULTS_ACK / DROPPED_FRAMES
//
// A single frame-control (FC) FSM drives BOTH engines for one frame at a time,
// then a race-free ownership FSM publishes the results into a hardware-owned
// double buffer that SW reads back through HIST_ADDR/BLOB_ADDR autoincrement
// windows.
//
// Deviations from the blob_detect_rle_top / isp_math_top reference patterns:
//   - The blob core's own top FSM (ST_IDLE/ST_CLEAR/...) is reproduced here as
//     the FC FSM, but SOF detection for hist_en purposes uses the *input-side*
//     accepted beat (s_axis_tvalid & s_axis_tready & tuser) rather than the
//     FIFO-output-side check blob_detect_rle_top uses for its own ST_WAIT_SOF,
//     because the two isp_histogram instances snoop the input side per spec.
//     The FC FSM's WAIT_SOF->PROCESS transition mirrors blob_detect_rle_top's
//     FIFO-output-side check (in_fifo_m_valid && tuser) so the blob core's row/
//     column tracking stays exactly aligned with what run_extractor actually
//     consumes.
// -----------------------------------------------------------------------------

`timescale 1ns / 1ps

`include "mocap_regs_defines.svh"

import mocap_regs_pkg::*;

module mocap_top #(
    parameter integer MAX_BLOBS        = 64,
    parameter integer MAX_RUNS_PER_ROW = 64, // worst case = HRES/2, every other pixel
    parameter integer AXIS_DATA_WIDTH  = 32,
    parameter integer AXIS_TUSER_WIDTH = 1,
    parameter integer AXIS_TKEEP_WIDTH = AXIS_DATA_WIDTH / 8
)(
    // -------------------------------------------------------------------------
    // Clock / reset
    // -------------------------------------------------------------------------
    input  wire                        aclk,
    input  wire                        aresetn,

    // -------------------------------------------------------------------------
    // AXI4-Lite Slave (7-bit byte address; region size 0x6C)
    // -------------------------------------------------------------------------
    input  wire [6:0]                  s_axi_awaddr,
    input  wire [2:0]                  s_axi_awprot,
    input  wire                        s_axi_awvalid,
    output wire                        s_axi_awready,

    input  wire [31:0]                 s_axi_wdata,
    input  wire [3:0]                  s_axi_wstrb,
    input  wire                        s_axi_wvalid,
    output wire                        s_axi_wready,

    output wire [1:0]                  s_axi_bresp,
    output wire                        s_axi_bvalid,
    input  wire                        s_axi_bready,

    input  wire [6:0]                  s_axi_araddr,
    input  wire [2:0]                  s_axi_arprot,
    input  wire                        s_axi_arvalid,
    output wire                        s_axi_arready,

    output wire [31:0]                 s_axi_rdata,
    output wire [1:0]                  s_axi_rresp,
    output wire                        s_axi_rvalid,
    input  wire                        s_axi_rready,

    // -------------------------------------------------------------------------
    // AXI-Stream Slave (from MIPI CSI-2 Rx subsystem)
    // -------------------------------------------------------------------------
    input  wire [AXIS_DATA_WIDTH-1:0]  s_axis_tdata,
    input  wire [9:0]                  s_axis_tdest,
    input  wire [AXIS_TUSER_WIDTH-1:0] s_axis_tuser,
    input  wire                        s_axis_tlast,
    input  wire                        s_axis_tvalid,
    output wire                        s_axis_tready,

    // -------------------------------------------------------------------------
    // AXI-Stream Master (to AXI VDMA)
    // -------------------------------------------------------------------------
    output wire [AXIS_DATA_WIDTH-1:0]  m_axis_tdata,
    output wire [AXIS_TKEEP_WIDTH-1:0] m_axis_tkeep,
    output wire [AXIS_TUSER_WIDTH-1:0] m_axis_tuser,
    output wire                        m_axis_tlast,
    output wire                        m_axis_tvalid,
    input  wire                        m_axis_tready,

    // -------------------------------------------------------------------------
    // Interrupt (frame_done_irq = isp_done & blob_done, latched)
    // -------------------------------------------------------------------------
    output wire                        frame_done_irq_o
);

    // =========================================================================
    // Register-block hwif
    // =========================================================================
    mocap_regs__in_t  hwif_in;
    mocap_regs__out_t hwif_out;

    // CTRL / config aliases
    wire        rst_all    = hwif_out.CMD.RESET.value;
    wire        ctrl_enable= hwif_out.CTRL.ENABLE.value;
    wire        ack_pulse  = hwif_out.CMD.RESULTS_ACK.value;
    wire [7:0]  threshold  = hwif_out.CTRL.THRESHOLD.value;
    wire [15:0] hres       = hwif_out.HRES.HRES.value;
    wire [15:0] vres       = hwif_out.VRES.VRES.value;
    wire        dims_ok    = (hres != 16'h0) && (vres != 16'h0) && (hres[1:0] == 2'b00);

    // =========================================================================
    // Datapath fork (race-free passthrough).
    //
    // The MIPI CSI-2 Rx is an UN-STALLABLE source: any backpressure asserted
    // toward it accumulates in its line buffer until it overflows ("Stream Line
    // Buffer Full!") and frames corrupt/drop. So the VIDEO path to VDMA must
    // NEVER depend on the blob engine or on CTRL.ENABLE -- it is a pure
    // passthrough whose ONLY backpressure source is the downstream VDMA
    // (m_axis_tready). The blob core and the histogram snoop the *accepted*
    // s_axis beats through their OWN elastic FIFO; if the blob core falls behind
    // its FIFO drops beats (best-effort) and sets STATUS.BLOB_FIFO_OVFL. The
    // blob core can never stall the video.
    //
    //   s_axis --accepted beat--+--> u_out_fifo ---------------> m_axis  (VIDEO,
    //   (tready = out_fifo         |                              never stalls)
    //    space ONLY)              +--> 2x isp_histogram          (snoop)
    //                             |
    //                             +--> u_in_fifo -> blob core    (best-effort)
    // =========================================================================
    // Packed AXIS sideband payload carried through both stream_fifos. Using a
    // packed struct instead of a hand-counted concat/slice makes the field
    // access name-checked: the tuser/tlast off-by-one that shredded VDMA frame
    // sync is structurally impossible here. First field = MSB, so the bit layout
    // is {tuser, tlast, tdata} -- identical to the previous concatenation.
    typedef struct packed {
        logic [AXIS_TUSER_WIDTH-1:0] tuser;  // [MSB] start-of-frame
        logic                        tlast;  //       end-of-line
        logic [AXIS_DATA_WIDTH-1:0]  tdata;  // [LSB] 4x8b pixels
    } axis_payload_t;
    localparam int FIFO_W = $bits(axis_payload_t);

    // The accepted s_axis beat, packed once and fed to BOTH FIFOs (video + blob).
    axis_payload_t s_axis_pl;
    assign s_axis_pl = '{tuser: s_axis_tuser, tlast: s_axis_tlast, tdata: s_axis_tdata};

    // ---- VIDEO passthrough FIFO: s_axis -> u_out_fifo -> m_axis -------------
    // s_axis_tready depends ONLY on this FIFO's space -- the camera is throttled
    // by VDMA alone, never by the blob engine or ENABLE.
    logic          out_fifo_s_ready;
    logic          out_fifo_m_valid, out_fifo_m_ready;
    axis_payload_t out_fifo_m_pl;
    logic          out_fifo_empty;

    assign s_axis_tready = out_fifo_s_ready;

    stream_fifo #(.DATA_WIDTH(FIFO_W), .DEPTH(16)) u_out_fifo (
        .clk(aclk), .rst_n(aresetn),
        .s_valid(s_axis_tvalid), .s_ready(out_fifo_s_ready),
        .s_data (s_axis_pl),
        .m_valid(out_fifo_m_valid), .m_ready(out_fifo_m_ready), .m_data(out_fifo_m_pl),
        .empty(out_fifo_empty)
    );

    assign m_axis_tdata     = out_fifo_m_pl.tdata;
    assign m_axis_tlast     = out_fifo_m_pl.tlast;
    assign m_axis_tuser     = out_fifo_m_pl.tuser;
    assign m_axis_tkeep     = {AXIS_TKEEP_WIDTH{1'b1}};
    assign m_axis_tvalid    = out_fifo_m_valid;
    assign out_fifo_m_ready = m_axis_tready;

    // The accepted-beat tap: one beat is "accepted" exactly when the video path
    // takes it. This drives BOTH the histogram snoop and the blob input FIFO.
    wire s_axis_accept = s_axis_tvalid & s_axis_tready;

    // ---- BLOB input FIFO: SOF-gated, non-blocking tap off the accepted beat --
    // run_extractor has no SOF input: it treats its first consumed beat after
    // `clear` as pixel (0,0) and counts hres*vres internally. So the blob core's
    // first captured beat MUST be a frame SOF and the FIFO must be empty when a
    // capture begins, or every subsequent pixel is misplaced. Because the video
    // path no longer backpressures the camera, we can no longer rely on the FIFO
    // filling/stalling to hold frame alignment; instead we explicitly gate
    // capture to the [SOF .. frame_done] window (blob_capture_beat / capturing_q,
    // driven from the FC FSM below) and DRAIN the FIFO outside that window so it
    // is always empty when the next capture's SOF arrives.
    //
    // Within a capture the enqueue is still non-blocking: if the blob core falls
    // behind and the FIFO fills, the beat is DROPPED (in_fifo_s_ready never feeds
    // back to s_axis_tready) and blob_fifo_ovfl pulses. On sparse marker frames
    // (and in sim, with the histogram's >=4-cycle inter-beat gap) the core keeps
    // up and this never trips; it is a hard guarantee that a busy frame degrades
    // the blob result, not the video.
    logic          in_fifo_s_ready;
    logic          in_fifo_m_valid, in_fifo_m_ready;
    axis_payload_t in_fifo_m_pl;
    logic          in_fifo_empty;

    logic capturing_q;        // high across one frame's [SOF .. frame_done]
    logic blob_capture_beat;  // an accepted beat we want the blob core to see
                              // (driven in the FC-FSM section, below)

    wire blob_fifo_ovfl = blob_capture_beat & ~in_fifo_s_ready;

    stream_fifo #(.DATA_WIDTH(FIFO_W), .DEPTH(64)) u_in_fifo (
        .clk(aclk), .rst_n(aresetn),
        .s_valid(blob_capture_beat), .s_ready(in_fifo_s_ready),
        .s_data (s_axis_pl),
        .m_valid(in_fifo_m_valid), .m_ready(in_fifo_m_ready), .m_data(in_fifo_m_pl),
        .empty(in_fifo_empty)
    );

    wire [AXIS_DATA_WIDTH-1:0]  in_pix_data  = in_fifo_m_pl.tdata;
    wire                        in_pix_tlast = in_fifo_m_pl.tlast;
    wire [AXIS_TUSER_WIDTH-1:0] in_pix_tuser = in_fifo_m_pl.tuser;

    // =========================================================================
    // Blob core: run_extractor -> row_merger -> blob_table (reused unmodified,
    // control signals driven by the FC FSM below instead of blob's own regs)
    // =========================================================================
    logic        re_enable, re_clear, re_s_ready, re_frame_done;
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
        .s_valid   (in_fifo_m_valid),
        .s_ready   (re_s_ready),
        .run_xs    (re_run_xs),
        .run_xe    (re_run_xe),
        .run_row   (re_run_row),
        .run_last_in_row (re_run_last),
        .run_valid (re_run_valid),
        .run_ready (re_run_ready),
        .frame_done(re_frame_done)
    );

    // Blob core drains its OWN input FIFO at its own pace during a capture; this
    // is fully independent of the video path -- run_extractor stalling (dropping
    // re_s_ready while it emits a run) can never backpressure s_axis or the video
    // FIFO, only its own snoop FIFO. OUTSIDE a capture we force-drain (discard)
    // so the FIFO returns to empty before the next capture's SOF.
    assign in_fifo_m_ready = capturing_q ? re_s_ready : 1'b1;

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

    logic        bt_clear, bt_flatten_start, bt_flatten_done, bt_clearing;
    logic [6:0]  bt_blob_count;
    logic [159:0] result_rd_data_w;
    logic [7:0]  copy_idx_q, copy_count_q, copy_addr_prev_q;
    logic        copy_prev_valid_q;
    wire  [6:0]  copy_addr_to_bt = copy_idx_q[6:0];

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
        .busy           (bt_clearing),
        .blob_count     (bt_blob_count),
        .result_rd_addr (copy_addr_to_bt),
        .result_rd_data (result_rd_data_w)
    );

    // =========================================================================
    // Histogram snoop: two isp_histogram instances, one hist_en at a time
    // (ping-pong on write_bank_q). Snoop point is the accepted INPUT beat.
    // =========================================================================
    logic [1:0]  hist_en, hist_scrub, hist_rdy, hist_err;
    logic [19:0] hist_ram_data     [0:1];
    logic        hist_ram_data_vld [0:1];

    genvar gi;
    generate
        for (gi = 0; gi < 2; gi++) begin : g_hist
            isp_histogram #(
                .STREAM_WIDTH (AXIS_DATA_WIDTH),
                .RAM_WIDTH    (20)
            ) u_hist (
                .clk_i          (aclk),
                .rst_n          (aresetn),
                .hist_en_i      (hist_en[gi]),
                .ram_scrub_i    (hist_scrub[gi]),
                .hist_rdy_o     (hist_rdy[gi]),
                .err_o          (hist_err[gi]),
                .ram_addr_i     (hwif_out.HIST_ADDR.HIST_ADDR.value),
                .ram_data_o     (hist_ram_data[gi]),
                .ram_data_o_vld (hist_ram_data_vld[gi]),
                // Snoop the input-FIFO POP point (the beat run_extractor consumes),
                // NOT s_axis, so histogram framing matches the blob engine exactly.
                // Enqueue = (in_fifo_m_valid & in_fifo_m_ready) & hist_en_i. Pops
                // are gated off during FINALIZE (re_enable low), so no next-frame
                // beats bleed into this frame's bank in continuous mode.
                .pix_data_i     (in_pix_data),
                .pix_data_vld_i (in_fifo_m_valid),
                .pix_data_rdy_i (in_fifo_m_ready)
            );
        end
    endgenerate

    // =========================================================================
    // Ownership / double-buffer state
    // =========================================================================
    logic        write_bank_q, read_bank_q, sw_owns_q, results_valid_q, frame_done_irq_q;
    logic [31:0] frame_id_q, dropped_frames_q;
    logic        overrun_sticky_q;
    logic [7:0]  published_blob_count_q [0:1];
    logic [31:0] published_pixel_sum_q  [0:1];
    // Blob double-buffer, flattened to a 1D memory so it infers a simple
    // dual-port BRAM. A 3D/[bank][idx] unpacked array with different bank
    // indices on the write vs read port is NOT inferred as BRAM by Vivado
    // (Synth 8-11357 -> 40960 registers). Address = {bank, idx[6:0]}; requires
    // MAX_BLOBS to be a power of two (128 here).
    localparam int BLOB_IDX_W = $clog2(MAX_BLOBS);
    logic [159:0] wrapper_blob_buf [0:2*MAX_BLOBS-1];

    assign frame_done_irq_o = frame_done_irq_q;

    // =========================================================================
    // Frame-control (FC) FSM: drives both engines for one frame, then hands
    // off to the ownership/publish logic. Mandatory FSM style: separate
    // always_comb (next-state) + always_ff (state register).
    // =========================================================================
    typedef enum logic [3:0] {
        FC_POST_RESET_SCRUB      = 4'd0,
        FC_WAIT_POST_RESET_SCRUB = 4'd1,
        FC_IDLE                  = 4'd2,
        FC_SCRUB                 = 4'd3,
        FC_WAIT_SOF              = 4'd4,
        FC_PROCESS               = 4'd5,
        FC_FINALIZE              = 4'd6,
        FC_COPY                  = 4'd7,
        FC_PUBLISH               = 4'd8,
        FC_LATCH                 = 4'd9   // 1-cycle: let blob_table.blob_count settle
    } fc_state_t;

    fc_state_t fc_state, fc_next;

    // Per-bank hist scrub-done detection (busy-seen & rdy, like isp_math_top)
    wire hist_scrub_done0 = hist_rdy[0];
    wire hist_scrub_done1 = hist_rdy[1];
    logic [1:0] hist_scrub_busy_seen_q;
    wire        hist_scrub_done_wb = hist_scrub_busy_seen_q[write_bank_q] & hist_rdy[write_bank_q];
    wire        hist_scrub_done_both = (hist_scrub_busy_seen_q[0] & hist_rdy[0]) &
                                        (hist_scrub_busy_seen_q[1] & hist_rdy[1]);

    logic [7:0] flush_cnt_q;
    wire        hist_flush_done = (flush_cnt_q >= 8'd50);

    wire copy_done = (fc_state == FC_COPY) && (copy_idx_q >= copy_count_q) && !copy_prev_valid_q;

    always_comb begin
        fc_next = fc_state;
        case (fc_state)
            FC_POST_RESET_SCRUB:      fc_next = FC_WAIT_POST_RESET_SCRUB;
            FC_WAIT_POST_RESET_SCRUB: if (hist_scrub_done_both) fc_next = FC_IDLE;
            FC_IDLE:                  if (ctrl_enable && dims_ok) fc_next = FC_SCRUB;
            FC_SCRUB:                 if (hist_scrub_done_wb && !bt_clearing) fc_next = FC_WAIT_SOF;
            FC_WAIT_SOF:              if (in_fifo_m_valid && in_pix_tuser[0]) fc_next = FC_PROCESS;
            FC_PROCESS:               if (re_frame_done) fc_next = FC_FINALIZE;
            FC_FINALIZE:              if (bt_flatten_done && hist_flush_done) fc_next = FC_LATCH;
            FC_LATCH:                 fc_next = FC_COPY;
            FC_COPY:                  if (copy_done) fc_next = FC_PUBLISH;
            FC_PUBLISH:               fc_next = ctrl_enable ? FC_SCRUB : FC_IDLE;
            default:                  fc_next = FC_POST_RESET_SCRUB;
        endcase
    end

    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn)      fc_state <= FC_POST_RESET_SCRUB;
        else if (rst_all)  fc_state <= FC_POST_RESET_SCRUB;
        else               fc_state <= fc_next;
    end

    // =========================================================================
    // Blob-core control signals (mirrors blob_detect_rle_top, driven by FC)
    // =========================================================================
    assign re_enable = (fc_state == FC_WAIT_SOF) || (fc_state == FC_PROCESS);
    assign re_clear  = (fc_state != FC_SCRUB) && (fc_next == FC_SCRUB);

    assign rm_enable = (fc_state == FC_PROCESS) || (fc_state == FC_FINALIZE);
    assign rm_clear  = re_clear;

    assign bt_clear         = re_clear;
    assign bt_flatten_start = (fc_state == FC_PROCESS) && (fc_next == FC_FINALIZE);

    // =========================================================================
    // Blob capture window (SOF-aligned, exactly one frame).
    //
    // The blob core snoops the video stream but must never stall it, so it can
    // only ever see a subset of frames -- the ones whose SOF arrives while the
    // FC FSM is READY (FC_WAIT_SOF). A frame whose SOF lands while the core is
    // still processing the previous one is simply not captured (real cameras
    // have vertical blanking >> the ~50-cycle finalize/scrub gap, so in practice
    // every frame is caught). capturing_q spans [SOF .. frame_done]; the very
    // first enqueued beat is that SOF, so run_extractor's pixel (0,0) is aligned.
    // =========================================================================
    wire blob_sof            = s_axis_accept & s_axis_tuser[0];
    wire blob_capture_start  = (fc_state == FC_WAIT_SOF) & blob_sof;
    assign blob_capture_beat = s_axis_accept & (capturing_q | blob_capture_start);

    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn)                capturing_q <= 1'b0;
        else if (rst_all)            capturing_q <= 1'b0;
        else if (re_clear)           capturing_q <= 1'b0; // FC scrub between frames
        else if (blob_capture_start) capturing_q <= 1'b1; // frame SOF, FC ready
        else if (re_frame_done)      capturing_q <= 1'b0; // frame fully consumed
    end

    // =========================================================================
    // Histogram control: scrub pulses + hist_en, gated on the active write bank
    // =========================================================================
    wire hist_active_period = (fc_state == FC_WAIT_SOF) || (fc_state == FC_PROCESS) ||
                               (fc_state == FC_FINALIZE);

    // Scrub the write bank at FC_SCRUB *entry*, one cycle AFTER write_bank_q has
    // flipped at PUBLISH. Deriving the scrub from re_clear (which pulses during
    // PUBLISH, before the flip) would scrub the bank just published to SW.
    logic fc_in_scrub_q;
    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn)     fc_in_scrub_q <= 1'b0;
        else if (rst_all) fc_in_scrub_q <= 1'b0;
        else              fc_in_scrub_q <= (fc_state == FC_SCRUB);
    end
    wire scrub_entry = (fc_state == FC_SCRUB) && !fc_in_scrub_q;

    assign hist_en[0]    = hist_active_period && (write_bank_q == 1'b0);
    assign hist_en[1]    = hist_active_period && (write_bank_q == 1'b1);
    assign hist_scrub[0] = (fc_state == FC_POST_RESET_SCRUB) || (scrub_entry && (write_bank_q == 1'b0));
    assign hist_scrub[1] = (fc_state == FC_POST_RESET_SCRUB) || (scrub_entry && (write_bank_q == 1'b1));

    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            hist_scrub_busy_seen_q <= 2'b00;
        end else if (rst_all) begin
            hist_scrub_busy_seen_q <= 2'b00;
        end else begin
            for (int i = 0; i < 2; i++) begin
                if (hist_scrub[i])
                    hist_scrub_busy_seen_q[i] <= 1'b0;
                else if (!hist_rdy[i])
                    hist_scrub_busy_seen_q[i] <= 1'b1;
            end
        end
    end

    // FLUSH drain timer (active-bank hist FIFO drain during FINALIZE)
    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn)                     flush_cnt_q <= 8'h0;
        else if (fc_state != FC_FINALIZE) flush_cnt_q <= 8'h0;
        else if (flush_cnt_q != 8'hFF)    flush_cnt_q <= flush_cnt_q + 8'h1;
    end

    // =========================================================================
    // Pixel-sum accumulation for the current frame (snapshot published at FC_PUBLISH)
    // =========================================================================
    // Accumulate over exactly the beats the histogram counts: the input-FIFO pop
    // while the active bank's hist_en is high. Guarantees PIXEL_SUM == sum(bin*i).
    logic [31:0] fc_pixel_sum_q;
    wire         beat_consumed = in_fifo_m_valid && in_fifo_m_ready;
    wire         capture_beat  = hist_active_period && beat_consumed;

    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn)             fc_pixel_sum_q <= 32'h0;
        else if (rst_all)         fc_pixel_sum_q <= 32'h0;
        else if (re_clear)        fc_pixel_sum_q <= 32'h0;
        else if (capture_beat)    fc_pixel_sum_q <= fc_pixel_sum_q
                                                   + {24'h0, in_pix_data[ 7: 0]}
                                                   + {24'h0, in_pix_data[15: 8]}
                                                   + {24'h0, in_pix_data[23:16]}
                                                   + {24'h0, in_pix_data[31:24]};
    end

    // =========================================================================
    // Blob copy-FSM: on flatten_done, walk result_rd_addr 0..blob_count-1 and
    // latch each 160-bit record into wrapper_blob_buf[write_bank]. Pipelined
    // one-address-per-cycle scheme that respects blob_table's 1-cycle
    // registered read latency (address presented this cycle; data valid next).
    // =========================================================================
    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            copy_idx_q        <= 8'h0;
            copy_count_q       <= 8'h0;
            copy_addr_prev_q  <= 8'h0;
            copy_prev_valid_q <= 1'b0;
        end else if (rst_all) begin
            copy_idx_q        <= 8'h0;
            copy_count_q       <= 8'h0;
            copy_addr_prev_q  <= 8'h0;
            copy_prev_valid_q <= 1'b0;
        end else if (fc_state == FC_LATCH) begin
            // blob_table updates its blob_count register on the edge LEAVING
            // BT_DONE (i.e. the FINALIZE->LATCH edge), so capture the CURRENT
            // frame's count here in FC_LATCH, one cycle later, not at the
            // FINALIZE->COPY edge (which would grab the previous frame's count).
            copy_idx_q        <= 8'h0;
            copy_count_q       <= {1'b0, bt_blob_count};
            copy_prev_valid_q <= 1'b0;
        end else if (fc_state == FC_COPY) begin
            copy_addr_prev_q  <= copy_idx_q;
            copy_prev_valid_q <= (copy_idx_q < copy_count_q);
            if (copy_idx_q < copy_count_q)
                copy_idx_q <= copy_idx_q + 8'h1;
        end
    end

    always_ff @(posedge aclk) begin
        if (fc_state == FC_COPY && copy_prev_valid_q)
            wrapper_blob_buf[{write_bank_q, copy_addr_prev_q[BLOB_IDX_W-1:0]}] <= result_rd_data_w;
    end

    // =========================================================================
    // Ownership / publish FSM (race-free double buffer)
    // =========================================================================
    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            write_bank_q            <= 1'b0;
            read_bank_q              <= 1'b0;
            sw_owns_q                <= 1'b0;
            results_valid_q          <= 1'b0;
            frame_done_irq_q         <= 1'b0;
            frame_id_q               <= 32'h0;
            dropped_frames_q         <= 32'h0;
            overrun_sticky_q         <= 1'b0;
            published_blob_count_q[0] <= 8'h0;
            published_blob_count_q[1] <= 8'h0;
            published_pixel_sum_q[0]  <= 32'h0;
            published_pixel_sum_q[1]  <= 32'h0;
        end else if (rst_all) begin
            write_bank_q            <= 1'b0;
            read_bank_q              <= 1'b0;
            sw_owns_q                <= 1'b0;
            results_valid_q          <= 1'b0;
            frame_done_irq_q         <= 1'b0;
            frame_id_q               <= 32'h0;
            dropped_frames_q         <= 32'h0;
            overrun_sticky_q         <= 1'b0;
            published_blob_count_q[0] <= 8'h0;
            published_blob_count_q[1] <= 8'h0;
            published_pixel_sum_q[0]  <= 32'h0;
            published_pixel_sum_q[1]  <= 32'h0;
        end else if (ack_pulse) begin
            // RESULTS_ACK: release the bank SW was holding
            sw_owns_q        <= 1'b0;
            results_valid_q  <= 1'b0;
            frame_done_irq_q <= 1'b0;
        end else if (fc_state == FC_PUBLISH) begin
            if (!sw_owns_q) begin
                // Free bank: publish write_bank, flip to the other bank
                published_blob_count_q[write_bank_q] <= copy_count_q[7:0];
                published_pixel_sum_q[write_bank_q]  <= fc_pixel_sum_q;
                read_bank_q       <= write_bank_q;
                sw_owns_q         <= 1'b1;
                frame_id_q        <= frame_id_q + 32'h1;
                results_valid_q   <= 1'b1;
                frame_done_irq_q  <= 1'b1;
                write_bank_q      <= ~write_bank_q;
            end else begin
                // Overrun: SW still holds read_bank. Keep-latest: overwrite the
                // same write_bank next frame instead of flipping into read_bank.
                published_blob_count_q[write_bank_q] <= copy_count_q[7:0];
                published_pixel_sum_q[write_bank_q]  <= fc_pixel_sum_q;
                dropped_frames_q <= dropped_frames_q + 32'h1;
                overrun_sticky_q <= 1'b1;
            end
        end
    end

    // =========================================================================
    // Sticky error/overflow bits (cleared only by RTL reset or CMD.RESET)
    // =========================================================================
    logic hist_fifo_err_sticky, blob_overflow_sticky, blob_fifo_ovfl_sticky;

    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn)                       hist_fifo_err_sticky <= 1'b0;
        else if (rst_all)                   hist_fifo_err_sticky <= 1'b0;
        else if (hist_err[0] || hist_err[1]) hist_fifo_err_sticky <= 1'b1;
    end

    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn)          blob_overflow_sticky <= 1'b0;
        else if (rst_all)      blob_overflow_sticky <= 1'b0;
        else if (rm_overflow)  blob_overflow_sticky <= 1'b1;
    end

    // Blob snoop FIFO dropped a beat (blob core couldn't keep up). Best-effort:
    // the video was unaffected, but this frame's blob result may be incomplete.
    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn)           blob_fifo_ovfl_sticky <= 1'b0;
        else if (rst_all)       blob_fifo_ovfl_sticky <= 1'b0;
        else if (blob_fifo_ovfl) blob_fifo_ovfl_sticky <= 1'b1;
    end

    // =========================================================================
    // Free-running 64-bit cycle counter + SW-triggered snapshot
    //   - cycle_counter_q counts every aclk from hardware reset (aresetn). It is
    //     intentionally NOT cleared by CMD.RESET so it stays a monotonic
    //     timebase across soft resets.
    //   - A CMD.CYCLE_SNAPSHOT pulse atomically captures it into cycle_snap_q,
    //     which SW reads back via CYCLE_SNAP_LO / CYCLE_SNAP_HI. Because the snap
    //     regs only change on the pulse, the two 32-bit reads never tear.
    // =========================================================================
    logic [63:0] cycle_counter_q;
    logic [63:0] cycle_snap_q;

    always_ff @(posedge aclk) begin
        if (!aresetn) cycle_counter_q <= 64'h0;
        else          cycle_counter_q <= cycle_counter_q + 64'h1;
    end

    always_ff @(posedge aclk) begin
        if (!aresetn)                              cycle_snap_q <= 64'h0;
        else if (hwif_out.CMD.CYCLE_SNAPSHOT.value) cycle_snap_q <= cycle_counter_q;
    end

    // =========================================================================
    // Register glue (hwif_in)
    // =========================================================================
    // Registered (synchronous) read of the blob double-buffer. A combinational
    // read here would force the whole 2x128x160b array into flip-flops plus a
    // 256:1 x160b mux (~40k FFs + tens of thousands of F7/F8 mux LUTs). Register
    // the read so it maps to a simple-dual-port BRAM (write port below at
    // {write_bank_q, copy_addr}, this read port at {read_bank_q, BLOB_ADDR}).
    // The one-cycle read latency is absorbed by AXI-Lite's AR->R phase gap and by
    // BLOB_ADDR being held stable across a blob's field reads -- the exact scheme
    // the histogram BRAM readback (hist_ram_data) already relies on.
    logic [159:0] blob_rd_rec;
    always_ff @(posedge aclk) begin
        blob_rd_rec <= wrapper_blob_buf[{read_bank_q, hwif_out.BLOB_ADDR.BLOB_ADDR.value[BLOB_IDX_W-1:0]}];
    end

    always_comb begin
        // ---- STATUS ----
        hwif_in.STATUS.READY.next          = (fc_state == FC_IDLE);
        hwif_in.STATUS.RESULTS_VALID.next  = results_valid_q;
        hwif_in.STATUS.FRAME_DONE_IRQ.next = frame_done_irq_q;
        hwif_in.STATUS.READ_BANK.next      = read_bank_q;
        hwif_in.STATUS.HIST_FIFO_ERR.next  = hist_fifo_err_sticky;
        hwif_in.STATUS.BLOB_OVERFLOW.next  = blob_overflow_sticky;
        hwif_in.STATUS.OVERRUN.next        = overrun_sticky_q;
        hwif_in.STATUS.BLOB_FIFO_OVFL.next = blob_fifo_ovfl_sticky;
        hwif_in.STATUS.BLOB_COUNT.next     = published_blob_count_q[read_bank_q];

        hwif_in.FRAME_ID.FRAME_ID.next             = frame_id_q;
        hwif_in.DROPPED_FRAMES.DROPPED_FRAMES.next = dropped_frames_q;
        hwif_in.PIXEL_SUM.PIXEL_SUM.next           = published_pixel_sum_q[read_bank_q];

        // ---- HIST_ADDR / HIST_DATA (autoinc on read of HIST_DATA) ----
        hwif_in.HIST_ADDR.HIST_ADDR.next = hwif_out.HIST_ADDR.HIST_ADDR.value + 8'h1;
        hwif_in.HIST_ADDR.HIST_ADDR.we   =
            (s_axi_arvalid & s_axi_arready & (s_axi_araddr == `MOCAP_REG_HIST_DATA))
          & hwif_out.CTRL.HIST_ADDR_AUTOINC.value;

        hwif_in.HIST_DATA.HIST_DATA.next = hist_ram_data[read_bank_q];

        // ---- BLOB_ADDR (autoinc + wrap on read of BLOB_COUNT_RD) ----
        hwif_in.BLOB_ADDR.BLOB_ADDR.next =
            (hwif_out.BLOB_ADDR.BLOB_ADDR.value >= 8'(MAX_BLOBS - 1)) ?
                8'h0 : (hwif_out.BLOB_ADDR.BLOB_ADDR.value + 8'h1);
        hwif_in.BLOB_ADDR.BLOB_ADDR.we   =
            (s_axi_arvalid & s_axi_arready & (s_axi_araddr == `MOCAP_REG_BLOB_COUNT_RD))
          & hwif_out.CTRL.BLOB_ADDR_AUTOINC.value;

        // ---- Blob result fields (from the wrapper-owned double buffer) ----
        hwif_in.BLOB_COUNT_RD.BLOB_COUNT_RD.next = blob_rd_rec[159:128];
        hwif_in.BLOB_SX.BLOB_SX.next             = blob_rd_rec[127: 96];
        hwif_in.BLOB_SY.BLOB_SY.next             = blob_rd_rec[ 95: 64];
        hwif_in.BLOB_XMIN.BLOB_XMIN.next         = blob_rd_rec[ 63: 48];
        hwif_in.BLOB_XMAX.BLOB_XMAX.next         = blob_rd_rec[ 47: 32];
        hwif_in.BLOB_YMIN.BLOB_YMIN.next         = blob_rd_rec[ 31: 16];
        hwif_in.BLOB_YMAX.BLOB_YMAX.next         = blob_rd_rec[ 15:  0];

        hwif_in.MAX_BLOBS_CFG.MAX_BLOBS_CFG.next = 16'(MAX_BLOBS);

        // ---- Cycle-counter snapshot readback ----
        hwif_in.CYCLE_SNAP_LO.CYCLE_SNAP_LO.next = cycle_snap_q[31:0];
        hwif_in.CYCLE_SNAP_HI.CYCLE_SNAP_HI.next = cycle_snap_q[63:32];
    end

    // =========================================================================
    // Register block (frozen contract)
    // =========================================================================
    mocap_regs u_mocap_regs (
        .clk            (aclk),
        .rst_n          (aresetn),

        .s_axil_awready (s_axi_awready),
        .s_axil_awvalid (s_axi_awvalid),
        .s_axil_awaddr  (s_axi_awaddr),
        .s_axil_awprot  (s_axi_awprot),

        .s_axil_wready  (s_axi_wready),
        .s_axil_wvalid  (s_axi_wvalid),
        .s_axil_wdata   (s_axi_wdata),
        .s_axil_wstrb   (s_axi_wstrb),

        .s_axil_bready  (s_axi_bready),
        .s_axil_bvalid  (s_axi_bvalid),
        .s_axil_bresp   (s_axi_bresp),

        .s_axil_arready (s_axi_arready),
        .s_axil_arvalid (s_axi_arvalid),
        .s_axil_araddr  (s_axi_araddr),
        .s_axil_arprot  (s_axi_arprot),

        .s_axil_rready  (s_axi_rready),
        .s_axil_rvalid  (s_axi_rvalid),
        .s_axil_rdata   (s_axi_rdata),
        .s_axil_rresp   (s_axi_rresp),

        .hwif_in        (hwif_in),
        .hwif_out       (hwif_out)
    );

    // Signals intentionally unused in this design (DMA path is SW-visible only
    // at this milestone; tdest is not needed for the snoop/series datapath).
    wire _unused = &{1'b0, s_axis_tdest,
                     hwif_out.DMA_BASE_LO.DMA_BASE_LO.value,
                     hwif_out.DMA_BASE_HI.DMA_BASE_HI.value,
                     hwif_out.DMA_LEN.DMA_LEN.value,
                     hwif_out.DMA_CTRL.DMA_CTRL.value,
                     hist_ram_data_vld[0], hist_ram_data_vld[1],
                     in_fifo_empty, out_fifo_empty,
                     hist_scrub_done0, hist_scrub_done1,
                     in_pix_tlast,
                     1'b0};

endmodule
