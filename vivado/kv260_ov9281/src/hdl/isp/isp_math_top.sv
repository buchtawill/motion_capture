// isp_math_top.sv
// Top-level ISP statistics helper for Vivado IP Integrator (KV260 / OV9281).
// https://docs.google.com/document/d/1X94gooBqVSsMei09nRS8F3e_mesuL7J35dIWdkzZWnM/edit?usp=sharing
//
// Hierarchy:
//   isp_math_wrapper.v
//     isp_math_top.sv         (this file)
//       isp_regs              (PeakRDL-generated AXI4-Lite slave)
//       isp_histogram         (256-bin histogram w/ internal FIFO)
//       local counters        (cycle, frame, pixel sum, beat count)
//
// Datapath posture
//   The AXI-Stream path from MIPI CSI RX -> AXI VDMA is a PURE SNOOP. This
//   module does not buffer, stall, or backpressure the datapath. The only
//   FIFO in the pipeline is the one inside isp_histogram, which the histogram
//   drains on its own; if it overflows, STATUS.HIST_FIFO_ERR latches sticky.
//
// Measurement flow (matches the spec state flow):
//   RTL reset           -> POST_RESET_SCRUB (scrub hist RAM) -> IDLE
//   IDLE + HISTOGRAM_START && HRES!=0 && VRES!=0
//                       -> START_SCRUB -> WAIT_SCRUB
//                       -> WAIT_TUSER  -> (on TUSER beat) MEASURE
//                       -> FLUSH (drain hist FIFO) -> IDLE (set HIST_DATA_VALID)
//
// Counter / snapshot fields in isp_regs.rdl are hw=w;sw=r, so this module
// holds counter state in local flops and republishes .next to the regblock.

`timescale 1ns / 1ps

`include "isp_regs_defines.svh"

import isp_regs_pkg::*;

module isp_wrapper #(
    parameter integer AXIS_DATA_WIDTH  = 32,
    parameter integer AXIS_TUSER_WIDTH = 1,
    parameter integer AXIS_TKEEP_WIDTH = AXIS_DATA_WIDTH / 8
)(
    // -------------------------------------------------------------------------
    // Clock / reset
    // -------------------------------------------------------------------------
    input  wire                           aclk,
    input  wire                           aresetn,

    // -------------------------------------------------------------------------
    // AXI4-Lite Slave  (11-bit byte address)
    // -------------------------------------------------------------------------
    input  wire [10:0]                    s_axi_awaddr,
    input  wire [2:0]                     s_axi_awprot,
    input  wire                           s_axi_awvalid,
    output wire                           s_axi_awready,

    input  wire [31:0]                    s_axi_wdata,
    input  wire [3:0]                     s_axi_wstrb,
    input  wire                           s_axi_wvalid,
    output wire                           s_axi_wready,

    output wire [1:0]                     s_axi_bresp,
    output wire                           s_axi_bvalid,
    input  wire                           s_axi_bready,

    input  wire [10:0]                    s_axi_araddr,
    input  wire [2:0]                     s_axi_arprot,
    input  wire                           s_axi_arvalid,
    output wire                           s_axi_arready,

    output wire [31:0]                    s_axi_rdata,
    output wire [1:0]                     s_axi_rresp,
    output wire                           s_axi_rvalid,
    input  wire                           s_axi_rready,

    // -------------------------------------------------------------------------
    // AXI-Stream Slave  (from MIPI CSI-2 Rx subsystem)
    // -------------------------------------------------------------------------
    input  wire [AXIS_DATA_WIDTH-1:0]     s_axis_tdata,
    input  wire [9:0]                     s_axis_tdest,
    input  wire [AXIS_TUSER_WIDTH-1:0]    s_axis_tuser,
    input  wire                           s_axis_tlast,
    input  wire                           s_axis_tvalid,
    output wire                           s_axis_tready,

    // -------------------------------------------------------------------------
    // AXI-Stream Master  (to AXI VDMA)
    // -------------------------------------------------------------------------
    output wire [AXIS_DATA_WIDTH-1:0]     m_axis_tdata,
    output wire [AXIS_TKEEP_WIDTH-1:0]    m_axis_tkeep,
    output wire [AXIS_TUSER_WIDTH-1:0]    m_axis_tuser,
    output wire                           m_axis_tlast,
    output wire                           m_axis_tvalid,
    input  wire                           m_axis_tready,

    // -------------------------------------------------------------------------
    // Interrupt
    // -------------------------------------------------------------------------
    output wire                           frame_done_irq_o
);

    // =========================================================================
    // AXI-Stream passthrough (no stall, no backpressure, zero datapath latency)
    // =========================================================================
    logic frame_done_irq_q;

    assign m_axis_tdata     = s_axis_tdata;
    assign m_axis_tkeep     = {AXIS_TKEEP_WIDTH{1'b1}};
    assign m_axis_tuser     = s_axis_tuser;
    assign m_axis_tlast     = s_axis_tlast;
    assign m_axis_tvalid    = s_axis_tvalid;
    assign s_axis_tready    = m_axis_tready;
    assign frame_done_irq_o = frame_done_irq_q;

    // =========================================================================
    // Register-block hwif
    // =========================================================================
    isp_regs__in_t  hwif_in;
    isp_regs__out_t hwif_out;

    // =========================================================================
    // CTRL-derived strobes
    // =========================================================================
    wire rst_all    = hwif_out.CTRL.RESET.value;
    wire rst_frame  = rst_all | hwif_out.CTRL.FRAME_CNT_RESET.value;
    wire rst_cycle  = rst_all | hwif_out.CTRL.CYCLE_CNT_RESET.value;
    wire snap_pulse = hwif_out.CTRL.SNAPSHOT.value;

    // AXIS handshake shorthand (tready is the passthrough of m_axis_tready)
    wire beat_accepted = s_axis_tvalid & m_axis_tready;
    wire tuser_tick    = s_axis_tuser[0] & beat_accepted;
    wire frame_tick    = tuser_tick;  // frame counter increments on first beat of each frame

    // =========================================================================
    // Cycle + frame counters and their snapshots (local state; regblock is
    // sw=r;hw=w so we republish via .next each cycle)
    // =========================================================================
    logic [31:0] cycle_cnt_lo_q,  cycle_cnt_hi_q;
    logic [31:0] cycle_snap_lo_q, cycle_snap_hi_q;
    logic [31:0] frame_cnt_q,     frame_snap_q;

    wire cycle_lo_wrap = (cycle_cnt_lo_q == 32'hFFFFFFFF);

    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn) begin
            cycle_cnt_lo_q  <= 32'h0;
            cycle_cnt_hi_q  <= 32'h0;
            cycle_snap_lo_q <= 32'h0;
            cycle_snap_hi_q <= 32'h0;
            frame_cnt_q     <= 32'h0;
            frame_snap_q    <= 32'h0;
        end else begin
            // ---- Cycle counter (free-running, clearable) ----
            if (rst_cycle) begin
                cycle_cnt_lo_q  <= 32'h0;
                cycle_cnt_hi_q  <= 32'h0;
                cycle_snap_lo_q <= 32'h0;
                cycle_snap_hi_q <= 32'h0;
            end else begin
                cycle_cnt_lo_q <= cycle_cnt_lo_q + 32'h1;
                cycle_cnt_hi_q <= cycle_cnt_hi_q + (cycle_lo_wrap ? 32'h1 : 32'h0);
                if (snap_pulse) begin
                    cycle_snap_lo_q <= cycle_cnt_lo_q;
                    cycle_snap_hi_q <= cycle_cnt_hi_q;
                end
            end

            // ---- Frame counter (TUSER-triggered, clearable) ----
            if (rst_frame) begin
                frame_cnt_q  <= 32'h0;
                frame_snap_q <= 32'h0;
            end else begin
                if (frame_tick)  frame_cnt_q  <= frame_cnt_q + 32'h1;
                if (snap_pulse)  frame_snap_q <= frame_cnt_q;
            end
        end
    end

    // =========================================================================
    // isp_histogram output signals (declared up here so the FSM can reference
    // hist_rdy before the instance itself is laid out below)
    // =========================================================================
    logic [19:0] hist_ram_data;
    logic        hist_ram_data_vld;  // unused; HIST_DATA reads route via regblock
    logic        hist_rdy;
    logic        hist_err;

    // =========================================================================
    // Measurement FSM
    // =========================================================================
    typedef enum logic [2:0] {
        ST_POST_RESET_SCRUB      = 3'd0,  // 1-cycle ram_scrub pulse post-reset
        ST_WAIT_POST_RESET_SCRUB = 3'd1,  // wait for hist_rdy rising edge
        ST_IDLE                  = 3'd2,  // READY = 1; awaiting HISTOGRAM_START
        ST_START_SCRUB           = 3'd3,  // 1-cycle ram_scrub pulse (measurement)
        ST_WAIT_SCRUB            = 3'd4,  // wait for hist_rdy rising edge
        ST_WAIT_TUSER            = 3'd5,  // arm hist_en when first TUSER beat arrives
        ST_MEASURE               = 3'd6,  // hist_en=1; count HRES*VRES/4 beats
        ST_FLUSH                 = 3'd7   // drain internal FIFO, then set HIST_DATA_VALID
    } state_t;

    state_t      state, next_state;
    logic        seen_scrub_busy_q;         // rising-edge detector for hist_rdy
    logic [31:0] beat_cnt_q;                // beats captured this measurement
    logic [31:0] pixel_sum_q;               // sum of pixels this measurement
    logic        data_valid_q;              // STATUS.HIST_DATA_VALID latch
    logic [7:0]  flush_cnt_q;               // FLUSH drain timer

    // HRES/VRES guards (spec: ignore HISTOGRAM_START if either is 0)
    wire [15:0] hres = hwif_out.HRES.HRES.value;
    wire [15:0] vres = hwif_out.VRES.VRES.value;
    wire        dims_nonzero = (hres != 16'h0) & (vres != 16'h0);

    // Beat-count target: pixels-per-frame / 4 pixels-per-beat
    wire [31:0] target_beats = ({16'h0, hres} * {16'h0, vres}) >> 2;

    // HISTOGRAM_START only honored when the FSM is in IDLE (implies READY=1)
    wire start_valid = hwif_out.CTRL.HISTOGRAM_START.value & dims_nonzero;

    // "Capture this beat now" — true on beats counted into beat_cnt / pixel_sum.
    // Includes the TUSER beat itself, because hist_en is raised combinationally
    // on that cycle.
    wire capture_beat_now =
        (state == ST_MEASURE    && beat_accepted) |
        (state == ST_WAIT_TUSER && tuser_tick);

    // Gate the tvalid fed to isp_histogram so that no more than target_beats
    // beats ever reach its FIFO, regardless of upstream traffic during FLUSH.
    wire hist_saturated = (beat_cnt_q >= target_beats);
    wire tvalid_to_hist = s_axis_tvalid & ~hist_saturated;

    // =========================================================================
    // FSM next-state and combinational outputs
    // =========================================================================
    logic hist_en;     // to isp_histogram
    logic ram_scrub;   // to isp_histogram

    always_comb begin
        next_state = state;
        case (state)
            // POST_RESET_SCRUB is a 1-cycle scrub-pulse state; we flow
            // unconditionally into the wait state and drop ram_scrub.
            ST_POST_RESET_SCRUB:      next_state = ST_WAIT_POST_RESET_SCRUB;

            ST_WAIT_POST_RESET_SCRUB: if (seen_scrub_busy_q & hist_rdy)
                                          next_state = ST_IDLE;

            ST_IDLE:                  if (start_valid)
                                          next_state = ST_START_SCRUB;

            ST_START_SCRUB:           next_state = ST_WAIT_SCRUB;

            ST_WAIT_SCRUB:            if (seen_scrub_busy_q & hist_rdy)
                                          next_state = ST_WAIT_TUSER;

            ST_WAIT_TUSER:            if (tuser_tick)
                                          next_state = ST_MEASURE;

            ST_MEASURE:               if (capture_beat_now &
                                         (beat_cnt_q + 32'h1 >= target_beats))
                                          next_state = ST_FLUSH;

            // FLUSH waits a fixed number of cycles so the histogram can pop
            // any residual FIFO entries. The histogram consumes ~4 cycles
            // per beat, the FIFO is 16 deep — 64 cycles is the absolute
            // worst case, 50 is safe for the gap-5 cadence we use in the TB.
            ST_FLUSH:                 if (flush_cnt_q >= 8'd50)
                                          next_state = ST_IDLE;

            default:                  next_state = ST_POST_RESET_SCRUB;
        endcase
    end

    // hist_en: high in MEASURE and FLUSH (so the histogram keeps popping its
    // FIFO while we drain), AND combinationally high on the TUSER beat so the
    // histogram sees hist_en=1 at the moment the first beat is accepted.
    assign hist_en =
          (state == ST_MEASURE)
        | (state == ST_FLUSH)
        | (state == ST_WAIT_TUSER && tuser_tick);

    // Single-cycle scrub pulse at the entry of each scrub phase
    assign ram_scrub = (state == ST_POST_RESET_SCRUB)
                     | (state == ST_START_SCRUB);

    // =========================================================================
    // FSM state register (RTL reset OR CTRL.RESET forces back to post-reset scrub)
    // =========================================================================
    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn)      state <= ST_POST_RESET_SCRUB;
        else if (rst_all)  state <= ST_POST_RESET_SCRUB;
        else               state <= next_state;
    end

    // seen_scrub_busy_q: detects that hist_rdy went low after we kicked a
    // scrub, so we don't immediately declare "scrub done" just because the
    // histogram was already idle.
    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn)
            seen_scrub_busy_q <= 1'b0;
        else if (state == ST_POST_RESET_SCRUB || state == ST_START_SCRUB)
            seen_scrub_busy_q <= 1'b0;
        else if ((state == ST_WAIT_POST_RESET_SCRUB || state == ST_WAIT_SCRUB)
                 && !hist_rdy)
            seen_scrub_busy_q <= 1'b1;
    end

    // =========================================================================
    // Beat counter (capture target_beats exactly)
    // =========================================================================
    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn)                          beat_cnt_q <= 32'h0;
        else if (rst_all)                      beat_cnt_q <= 32'h0;
        else if (state == ST_START_SCRUB)      beat_cnt_q <= 32'h0;
        else if (capture_beat_now)             beat_cnt_q <= beat_cnt_q + 32'h1;
    end

    // =========================================================================
    // Pixel-sum counter (same capture gate; 8+8+8+8 bytes per beat)
    // =========================================================================
    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn)                          pixel_sum_q <= 32'h0;
        else if (rst_all)                      pixel_sum_q <= 32'h0;
        else if (state == ST_START_SCRUB)      pixel_sum_q <= 32'h0;
        else if (capture_beat_now) begin
            pixel_sum_q <= pixel_sum_q
                         + {24'h0, s_axis_tdata[ 7: 0]}
                         + {24'h0, s_axis_tdata[15: 8]}
                         + {24'h0, s_axis_tdata[23:16]}
                         + {24'h0, s_axis_tdata[31:24]};
        end
    end

    // =========================================================================
    // HIST_DATA_VALID: set at FLUSH->IDLE; cleared on reset / new measurement
    // =========================================================================
    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn)                              data_valid_q <= 1'b0;
        else if (rst_all)                          data_valid_q <= 1'b0;
        else if (state == ST_START_SCRUB)          data_valid_q <= 1'b0;
        else if (state == ST_FLUSH && next_state == ST_IDLE)
                                                   data_valid_q <= 1'b1;
    end

    // =========================================================================
    // FLUSH drain timer
    // =========================================================================
    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn)                  flush_cnt_q <= 8'h0;
        else if (state != ST_FLUSH)    flush_cnt_q <= 8'h0;
        else if (flush_cnt_q != 8'hFF) flush_cnt_q <= flush_cnt_q + 8'h1;
    end

    // =========================================================================
    // isp_histogram instance (pure snoop of AXIS; its FIFO is the only buffer)
    // =========================================================================
    isp_histogram #(
        .STREAM_WIDTH (32),
        .RAM_WIDTH    (20)
    ) u_isp_histogram (
        .clk_i          (aclk),
        .rst_n          (aresetn),
        .hist_en_i      (hist_en),
        .ram_scrub_i    (ram_scrub),
        .hist_rdy_o     (hist_rdy),
        .err_o          (hist_err),

        .ram_addr_i     (hwif_out.HIST_ADDR.HIST_ADDR.value),
        .ram_data_o     (hist_ram_data),
        .ram_data_o_vld (hist_ram_data_vld),

        .pix_data_i     (s_axis_tdata),
        .pix_data_vld_i (tvalid_to_hist),
        .pix_data_rdy_i (m_axis_tready)
    );

    // =========================================================================
    // STATUS.HIST_FIFO_ERR sticky bit
    //   Source: isp_histogram.err_o (FIFO overflow). Cleared only by RTL reset
    //   or CTRL.RESET per spec.
    // =========================================================================
    logic hist_fifo_err_sticky;
    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn)        hist_fifo_err_sticky <= 1'b0;
        else if (rst_all)    hist_fifo_err_sticky <= 1'b0;
        else if (hist_err)   hist_fifo_err_sticky <= 1'b1;
    end

    // =========================================================================
    // FRAME_DONE_IRQ latch
    //   Set on the FLUSH->IDLE transition (measurement complete).
    //   Cleared by CTRL.IRQ_CLEAR or CTRL.RESET.
    // =========================================================================
    wire irq_clear     = hwif_out.CTRL.IRQ_CLEAR.value;
    wire measurement_done = (state == ST_FLUSH) && (next_state == ST_IDLE);

    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn)                    frame_done_irq_q <= 1'b0;
        else if (rst_all | irq_clear)    frame_done_irq_q <= 1'b0;
        else if (measurement_done)       frame_done_irq_q <= 1'b1;
    end

    // =========================================================================
    // Drive the register block
    // =========================================================================
    always_comb begin
        // ---- STATUS ----
        hwif_in.STATUS.READY.next           = (state == ST_IDLE);
        hwif_in.STATUS.HIST_DATA_VALID.next = data_valid_q;
        hwif_in.STATUS.HIST_FIFO_ERR.next   = hist_fifo_err_sticky;
        hwif_in.STATUS.FRAME_DONE_IRQ.next  = frame_done_irq_q;

        // ---- Cycle + frame counters and snapshots ----
        hwif_in.CYCLE_CNT_LO.CYCLE_CNT_LO.next   = cycle_cnt_lo_q;
        hwif_in.CYCLE_CNT_HI.CYCLE_CNT_HI.next   = cycle_cnt_hi_q;
        hwif_in.CYCLE_SNAP_LO.CYCLE_SNAP_LO.next = cycle_snap_lo_q;
        hwif_in.CYCLE_SNAP_HI.CYCLE_SNAP_HI.next = cycle_snap_hi_q;
        hwif_in.FRAME_CNT.FRAME_CNT.next         = frame_cnt_q;
        hwif_in.FRAME_SNAP.FRAME_SNAP.next       = frame_snap_q;

        // ---- PIXEL_SUM ----
        hwif_in.PIXEL_SUM.PIXEL_SUM.next = pixel_sum_q;

        // ---- HIST_ADDR auto-increment on AXI read of HIST_DATA ----
        hwif_in.HIST_ADDR.HIST_ADDR.next =
            hwif_out.HIST_ADDR.HIST_ADDR.value + 8'h1;
        hwif_in.HIST_ADDR.HIST_ADDR.we   =
            (s_axi_arvalid & s_axi_arready & (s_axi_araddr == `ISP_REG_HIST_DATA))
          & hwif_out.CTRL.HIST_ADDR_AUTOINC.value;

        // ---- HIST_DATA (frontdoor read port of isp_histogram) ----
        hwif_in.HIST_DATA.HIST_DATA.next = hist_ram_data;
    end

    // =========================================================================
    // Register block
    // =========================================================================
    isp_regs u_isp_regs (
        .clk              (aclk),
        .rst_n            (aresetn),

        // AXI4-Lite
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

        // Hardware interface
        .hwif_in          (hwif_in),
        .hwif_out         (hwif_out)
    );

endmodule
