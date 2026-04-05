// isp_math_top.sv
// Top-level ISP wrapper for Vivado IP Integrator (KV260 / OV9281).
//
// Instantiates the PeakRDL-generated register block (isp_regs.sv) and
// wires the AXI4-Lite slave port through to it.  AXI-Stream is a pure
// passthrough — zero latency, no backpressure added.
//
// NOTE: This file uses SystemVerilog (package import for isp_regs_pkg).
//       Set the file type to "SystemVerilog" in Vivado source properties
//       (or rename to .sv) if Vivado does not pick it up automatically.
//
// Reset polarity: IP Integrator supplies active-low aresetn; isp_regs
// expects active-high rst.  The inversion is done here.
//
// hwif connections: all hwif_in fields are tied to 0 / '0 for now.
// Wire stats_engine outputs here once that module is written.

`timescale 1ns / 1ps

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
    // AXI4-Lite Slave  (11-bit byte address, covers 0x000–0x42F)
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
    // Reset polarity conversion  (IP Integrator: active-low → isp_regs: active-high)
    // =========================================================================
    wire rst = ~aresetn;

    // =========================================================================
    // AXI-Stream passthrough  (pure wire — zero latency, zero backpressure)
    // =========================================================================
    assign m_axis_tdata  = s_axis_tdata;
    assign m_axis_tkeep  = {AXIS_TKEEP_WIDTH{1'b1}};
    assign m_axis_tuser  = s_axis_tuser;
    assign m_axis_tlast  = s_axis_tlast;
    assign m_axis_tvalid = s_axis_tvalid;
    assign s_axis_tready = m_axis_tready;

    // =========================================================================
    // Hardware interface structs
    // TODO: connect hwif_in fields from stats_engine once written.
    // =========================================================================
    isp_regs__in_t  hwif_in;
    isp_regs__out_t hwif_out;

    // always_comb begin
    //     hwif_in = '0;
    // end

    // =========================================================================
    // Register block
    // =========================================================================
    isp_regs u_isp_regs (
        .clk              (aclk),
        .rst              (rst),

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
