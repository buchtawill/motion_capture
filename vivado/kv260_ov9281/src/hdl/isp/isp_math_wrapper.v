// isp_math_wrapper.v
// Plain Verilog 2001 shell for Vivado IP Integrator block diagram.
// Instantiates isp_math_top (SystemVerilog) and passes all signals through.
// No logic here — all ports are wired 1:1.

`timescale 1ns / 1ps

module isp_math_wrapper (
    // -------------------------------------------------------------------------
    // Clock / reset
    // -------------------------------------------------------------------------
    input  wire         aclk,
    input  wire         aresetn,

    // -------------------------------------------------------------------------
    // AXI4-Lite Slave
    // -------------------------------------------------------------------------
    input  wire [10:0]  s_axi_awaddr,
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

    input  wire [10:0]  s_axi_araddr,
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
    // Interrupt
    // -------------------------------------------------------------------------
    (* X_INTERFACE_INFO = "xilinx.com:signal:interrupt:1.0 frame_done_irq_o INTERRUPT" *)
    (* X_INTERFACE_PARAMETER = "SENSITIVITY EDGE_RISING" *)
    output wire         frame_done_irq_o
);

    isp_wrapper #(
        .AXIS_DATA_WIDTH  (32),
        .AXIS_TUSER_WIDTH (1)
    ) u_isp_math_top (
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

        .m_axis_tdata      (m_axis_tdata),
        .m_axis_tkeep      (m_axis_tkeep),
        .m_axis_tuser      (m_axis_tuser),
        .m_axis_tlast      (m_axis_tlast),
        .m_axis_tvalid     (m_axis_tvalid),
        .m_axis_tready     (m_axis_tready),

        .frame_done_irq_o  (frame_done_irq_o)
    );

endmodule
