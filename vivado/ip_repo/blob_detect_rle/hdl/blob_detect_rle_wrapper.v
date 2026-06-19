`timescale 1ns / 1ps

module blob_detect_rle_wrapper (
    input  wire         aclk,
    input  wire         aresetn,

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

    input  wire [31:0]  s_axis_tdata,
    input  wire [0:0]   s_axis_tuser,
    input  wire         s_axis_tlast,
    input  wire         s_axis_tvalid,
    output wire         s_axis_tready,

    output wire [31:0]  m_axis_tdata,
    output wire [3:0]   m_axis_tkeep,
    output wire [0:0]   m_axis_tuser,
    output wire         m_axis_tlast,
    output wire         m_axis_tvalid,
    input  wire         m_axis_tready,

    (* X_INTERFACE_INFO = "xilinx.com:signal:interrupt:1.0 frame_done_irq_o INTERRUPT" *)
    (* X_INTERFACE_PARAMETER = "SENSITIVITY EDGE_RISING" *)
    output wire         frame_done_irq_o
);

    blob_detect_rle_top #(
        .MAX_BLOBS        (128),
        .MAX_RUNS_PER_ROW (640),
        .AXIS_DATA_WIDTH  (32)
    ) u_blob_detect_rle_top (
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
