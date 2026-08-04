// mocap_wrapper.sv
// KV260 / OV9281 motion-capture wrapper.
//
// Fuses the 256-bin ISP histogram and the RLE blob detector behind one AXI4-Lite
// register file with a race-free, hardware-owned double-buffered result store.
// See motion_capture/agents/PLAN_new_hw_pipeline.md (architecture) and the
// implementation plan for the ownership protocol.
//
// Datapath (settled):
//   s_axis --+--> (snoop) 2x isp_histogram (ping-pong, one hist_en at a time)
//            |
//            +--> FIFO -> blob_detect_rle (series) -> FIFO -> m_axis (to VDMA)
//
//   Result store (all banking owned HERE; engines are reused unmodified):
//     - histogram: the two isp_histogram instances ARE the two banks
//     - blob:      wrapper_blob_buf[2] filled by a copy-FSM on flatten_done
//     - one shared bank_select / sw_owns / RESULTS_ACK / DROPPED_FRAMES
//
// -----------------------------------------------------------------------------
// STATUS: M0 skeleton. Ports + register block are the frozen contract. The
// datapath, the two histogram instances, the blob engine + copy-FSM, and the
// shared frame-control / ownership FSM are implemented in Milestone 2. Until
// then this file compiles and passes the stream through so the testbench can
// bind and run in a red state.
// -----------------------------------------------------------------------------

`timescale 1ns / 1ps

`include "mocap_regs_defines.svh"

import mocap_regs_pkg::*;

module mocap_wrapper #(
    parameter integer MAX_BLOBS        = 128,
    parameter integer MAX_RUNS_PER_ROW = 640,
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
    // AXI4-Lite Slave (7-bit byte address; region size 0x60)
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

    // =========================================================================
    // M0 SKELETON BEHAVIOUR (replaced in Milestone 2)
    //   - AXIS: pure passthrough so downstream sees the stream.
    //   - hwif_in: safe constant defaults so reads are well-defined.
    //   - IRQ: deasserted.
    // =========================================================================
    assign m_axis_tdata  = s_axis_tdata;
    assign m_axis_tkeep  = {AXIS_TKEEP_WIDTH{1'b1}};
    assign m_axis_tuser  = s_axis_tuser;
    assign m_axis_tlast  = s_axis_tlast;
    assign m_axis_tvalid = s_axis_tvalid;
    assign s_axis_tready = m_axis_tready;

    assign frame_done_irq_o = 1'b0;

    always_comb begin
        // STATUS
        hwif_in.STATUS.READY.next          = 1'b0;
        hwif_in.STATUS.RESULTS_VALID.next  = 1'b0;
        hwif_in.STATUS.FRAME_DONE_IRQ.next = 1'b0;
        hwif_in.STATUS.READ_BANK.next      = 1'b0;
        hwif_in.STATUS.HIST_FIFO_ERR.next  = 1'b0;
        hwif_in.STATUS.BLOB_OVERFLOW.next  = 1'b0;
        hwif_in.STATUS.OVERRUN.next        = 1'b0;
        hwif_in.STATUS.BLOB_COUNT.next     = 8'h0;

        hwif_in.FRAME_ID.FRAME_ID.next             = 32'h0;
        hwif_in.DROPPED_FRAMES.DROPPED_FRAMES.next = 32'h0;
        hwif_in.PIXEL_SUM.PIXEL_SUM.next           = 32'h0;

        // HIST_ADDR / BLOB_ADDR: no HW autoinc write in skeleton
        hwif_in.HIST_ADDR.HIST_ADDR.next = 8'h0;
        hwif_in.HIST_ADDR.HIST_ADDR.we   = 1'b0;
        hwif_in.BLOB_ADDR.BLOB_ADDR.next = 8'h0;
        hwif_in.BLOB_ADDR.BLOB_ADDR.we   = 1'b0;

        // Result fields
        hwif_in.HIST_DATA.HIST_DATA.next     = 20'h0;
        hwif_in.BLOB_COUNT_RD.BLOB_COUNT_RD.next = 32'h0;
        hwif_in.BLOB_SX.BLOB_SX.next         = 32'h0;
        hwif_in.BLOB_SY.BLOB_SY.next         = 32'h0;
        hwif_in.BLOB_XMIN.BLOB_XMIN.next     = 16'h0;
        hwif_in.BLOB_XMAX.BLOB_XMAX.next     = 16'h0;
        hwif_in.BLOB_YMIN.BLOB_YMIN.next     = 16'h0;
        hwif_in.BLOB_YMAX.BLOB_YMAX.next     = 16'h0;

        hwif_in.MAX_BLOBS_CFG.MAX_BLOBS_CFG.next = 16'(MAX_BLOBS);
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

    // Prevent "unused" elaboration noise in the skeleton.
    wire _unused = &{1'b0, hwif_out.CTRL.RESET.value, s_axis_tdest, s_axis_tvalid,
                     hwif_out.DMA_CTRL.DMA_CTRL.value, 1'b0};

endmodule
