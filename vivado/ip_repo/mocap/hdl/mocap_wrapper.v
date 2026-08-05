// mocap_wrapper.v
// Plain Verilog 2001 shell for Vivado IP Integrator block diagram.
// Instantiates mocap_top (SystemVerilog) and passes all signals through 1:1.
// No logic here. The frame-done interrupt is annotated so IP Integrator
// recognizes it as an interrupt pin (rising-edge sensitive).

`timescale 1ns / 1ps

module mocap_wrapper #(
    parameter integer MAX_BLOBS        = 128,
    parameter integer MAX_RUNS_PER_ROW = 640
) (
    // -------------------------------------------------------------------------
    // Clock / reset
    // -------------------------------------------------------------------------
    input  wire         aclk,
    input  wire         aresetn,

    // -------------------------------------------------------------------------
    // AXI4-Lite Slave (7-bit byte address; region size 0x60)
    // -------------------------------------------------------------------------
    input  wire [6:0]   s_axi_awaddr,
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

    input  wire [6:0]   s_axi_araddr,
    input  wire [2:0]   s_axi_arprot,
    input  wire         s_axi_arvalid,
    output wire         s_axi_arready,

    output wire [31:0]  s_axi_rdata,
    output wire [1:0]   s_axi_rresp,
    output wire         s_axi_rvalid,
    input  wire         s_axi_rready,

    // -------------------------------------------------------------------------
    // AXI-Stream Slave  (from MIPI CSI-2 Rx subsystem)
    // -------------------------------------------------------------------------
    input  wire [31:0]  s_axis_tdata,
    input  wire [9:0]   s_axis_tdest,
    input  wire [0:0]   s_axis_tuser,
    input  wire         s_axis_tlast,
    input  wire         s_axis_tvalid,
    output wire         s_axis_tready,

    // -------------------------------------------------------------------------
    // AXI-Stream Master  (to AXI VDMA)
    // -------------------------------------------------------------------------
    output wire [31:0]  m_axis_tdata,
    output wire [3:0]   m_axis_tkeep,
    output wire [0:0]   m_axis_tuser,
    output wire         m_axis_tlast,
    output wire         m_axis_tvalid,
    input  wire         m_axis_tready,

    // -------------------------------------------------------------------------
    // Interrupt  (frame_done_irq = isp_done & blob_done, latched)
    // -------------------------------------------------------------------------
    (* X_INTERFACE_INFO = "xilinx.com:signal:interrupt:1.0 frame_done_irq_o INTERRUPT" *)
    (* X_INTERFACE_PARAMETER = "SENSITIVITY EDGE_RISING" *)
    output wire         frame_done_irq_o
);

    mocap_top #(
        .MAX_BLOBS        (MAX_BLOBS),
        .MAX_RUNS_PER_ROW (MAX_RUNS_PER_ROW),
        .AXIS_DATA_WIDTH  (32),
        .AXIS_TUSER_WIDTH (1)
    ) u_mocap_top (
        .aclk             (aclk),
        .aresetn          (aresetn),

        .s_axi_awaddr     (s_axi_awaddr),
        .s_axi_awprot     (s_axi_awprot),
        .s_axi_awvalid    (s_axi_awvalid),
        .s_axi_awready    (s_axi_awready),

        .s_axi_wdata      (s_axi_wdata),
        .s_axi_wstrb      (s_axi_wstrb),
        .s_axi_wvalid     (s_axi_wvalid),
        .s_axi_wready     (s_axi_wready),

        .s_axi_bresp      (s_axi_bresp),
        .s_axi_bvalid     (s_axi_bvalid),
        .s_axi_bready     (s_axi_bready),

        .s_axi_araddr     (s_axi_araddr),
        .s_axi_arprot     (s_axi_arprot),
        .s_axi_arvalid    (s_axi_arvalid),
        .s_axi_arready    (s_axi_arready),

        .s_axi_rdata      (s_axi_rdata),
        .s_axi_rresp      (s_axi_rresp),
        .s_axi_rvalid     (s_axi_rvalid),
        .s_axi_rready     (s_axi_rready),

        .s_axis_tdata     (s_axis_tdata),
        .s_axis_tdest     (s_axis_tdest),
        .s_axis_tuser     (s_axis_tuser),
        .s_axis_tlast     (s_axis_tlast),
        .s_axis_tvalid    (s_axis_tvalid),
        .s_axis_tready    (s_axis_tready),

        .m_axis_tdata     (m_axis_tdata),
        .m_axis_tkeep     (m_axis_tkeep),
        .m_axis_tuser     (m_axis_tuser),
        .m_axis_tlast     (m_axis_tlast),
        .m_axis_tvalid    (m_axis_tvalid),
        .m_axis_tready    (m_axis_tready),

        .frame_done_irq_o (frame_done_irq_o)
    );

endmodule
