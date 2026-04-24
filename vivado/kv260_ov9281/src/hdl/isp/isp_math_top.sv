// isp_math_top.sv
// Top-level ISP wrapper for Vivado IP Integrator (KV260 / OV9281).
// https://docs.google.com/document/d/1X94gooBqVSsMei09nRS8F3e_mesuL7J35dIWdkzZWnM/edit?usp=sharing
// Instantiates the PeakRDL-generated register block (isp_regs.sv) and wires
// the AXI4-Lite slave port through to it. The AXI-Stream path is a pure
// passthrough for now -- zero latency, no backpressure added.
//
// The register-level plumbing implemented here:
//   * Free-running cycle counter (64b, LO+HI) with per-counter SW reset
//     -- local flops; value is published to the regblock as .next each cycle.
//   * Frame counter (32b), incremented on TUSER & TVALID & TREADY.
//   * Cycle + frame snapshot registers, latched on CTRL.SNAPSHOT.
//   * Global CTRL.RESET clears all counters and snapshots.
//   * HIST_ADDR auto-increment on AXI reads of HIST_DATA (gated by
//     CTRL.HIST_ADDR_AUTOINC).
//   * STATUS.HIST_FIFO_ERR sticky-bit flop.
//
// Stubs (to be wired once isp_histogram.sv and the histogram FSM land):
//   * STATUS.READY is tied to 1 (no scrub FSM yet).
//   * STATUS.HIST_DATA_VALID is tied to 0.
//   * PIXEL_SUM is tied to 0.
//   * HIST_DATA is tied to 0.
//   * HIST_FIFO_ERR raw input is tied to 0.
//
// The counter / snapshot fields in isp_regs.rdl are declared hw=w; sw=r;
// meaning the regblock is the SW-visible storage and HW cannot read back the
// register's contents. All counter state is kept in local flops in this
// module; we only drive .next to publish values into the regblock.

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
    input  wire                           m_axis_tready
);

    // =========================================================================
    // AXI-Stream passthrough
    // =========================================================================
    assign m_axis_tdata  = s_axis_tdata;
    assign m_axis_tkeep  = {AXIS_TKEEP_WIDTH{1'b1}};
    assign m_axis_tuser  = s_axis_tuser;
    assign m_axis_tlast  = s_axis_tlast;
    assign m_axis_tvalid = s_axis_tvalid;
    assign s_axis_tready = m_axis_tready;

    // =========================================================================
    // Hardware interface structs
    // =========================================================================
    isp_regs__in_t  hwif_in;
    isp_regs__out_t hwif_out;

    // =========================================================================
    // Derived strobes
    // =========================================================================
    wire rst_all    = hwif_out.CTRL.RESET.value;
    wire rst_frame  = rst_all | hwif_out.CTRL.FRAME_CNT_RESET.value;
    wire rst_cycle  = rst_all | hwif_out.CTRL.CYCLE_CNT_RESET.value;
    wire snap_pulse = hwif_out.CTRL.SNAPSHOT.value;

    // "Frame tick" = first accepted beat of a frame from MIPI CSI RX:
    // TUSER marks that beat, TVALID/TREADY qualify it.
    wire frame_tick = s_axis_tuser[0] & s_axis_tvalid & m_axis_tready;

    // Snoop AXI read handshake targeting HIST_DATA to drive HIST_ADDR autoinc.
    wire hist_data_ar_handshake =
        s_axi_arvalid & s_axi_arready & (s_axi_araddr == `ISP_REG_HIST_DATA);

    // =========================================================================
    // Local counter / snapshot flops
    //
    // The regblock does not expose these fields back to HW (hw=w;sw=r), so we
    // hold the counter state here and republish it as .next every cycle. The
    // regblock latches .next on each clock; SW sees the same value.
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
    // STATUS.HIST_FIFO_ERR sticky bit
    //
    // Driven by isp_histogram once that module is instantiated; tied to 0 for
    // now. Cleared only by RTL reset or CTRL.RESET, per spec.
    // =========================================================================
    wire  hist_fifo_err_raw = 1'b0;
    logic hist_fifo_err_sticky;

    always_ff @(posedge aclk or negedge aresetn) begin
        if (!aresetn)                hist_fifo_err_sticky <= 1'b0;
        else if (rst_all)            hist_fifo_err_sticky <= 1'b0;
        else if (hist_fifo_err_raw)  hist_fifo_err_sticky <= 1'b1;
    end

    // =========================================================================
    // Register-map hwif_in drivers
    // =========================================================================
    always_comb begin
        // ---- STATUS (stubbed until histogram FSM lands) ----
        hwif_in.STATUS.READY.next           = 1'b1;
        hwif_in.STATUS.HIST_DATA_VALID.next = 1'b0;
        hwif_in.STATUS.HIST_FIFO_ERR.next   = hist_fifo_err_sticky;

        // ---- Cycle + frame counters and their snapshots ----
        hwif_in.CYCLE_CNT_LO.CYCLE_CNT_LO.next   = cycle_cnt_lo_q;
        hwif_in.CYCLE_CNT_HI.CYCLE_CNT_HI.next   = cycle_cnt_hi_q;
        hwif_in.CYCLE_SNAP_LO.CYCLE_SNAP_LO.next = cycle_snap_lo_q;
        hwif_in.CYCLE_SNAP_HI.CYCLE_SNAP_HI.next = cycle_snap_hi_q;
        hwif_in.FRAME_CNT.FRAME_CNT.next         = frame_cnt_q;
        hwif_in.FRAME_SNAP.FRAME_SNAP.next       = frame_snap_q;

        // ---- PIXEL_SUM (stub until pixel-sum counter lands) ----
        hwif_in.PIXEL_SUM.PIXEL_SUM.next = 32'h0;

        // ---- HIST_ADDR auto-increment (natural 8b wrap at 0xFF) ----
        hwif_in.HIST_ADDR.HIST_ADDR.next =
            hwif_out.HIST_ADDR.HIST_ADDR.value + 8'h1;
        hwif_in.HIST_ADDR.HIST_ADDR.we   =
            hist_data_ar_handshake & hwif_out.CTRL.HIST_ADDR_AUTOINC.value;

        // ---- HIST_DATA (stub until isp_histogram is instantiated) ----
        hwif_in.HIST_DATA.HIST_DATA.next = 20'h0;
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