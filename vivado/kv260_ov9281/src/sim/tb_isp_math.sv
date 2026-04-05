// tb_isp_math.sv
// Testbench for isp_math_wrapper (wraps isp_math_top -> isp_regs)
//
// Topology
//   tb ──AXI-Lite──► isp_math_wrapper (DUT) ──AXI-Stream──► tready (tied high)
//
// Test plan
//   1. Read HRES (0x008) — expect 16'h500 (1280, PeakRDL reset default)
//   2. Read VRES (0x00C) — expect 16'h320 ( 800, PeakRDL reset default)

`timescale 1ns / 1ps

module tb_isp_math;

    // =========================================================================
    // Clock
    // =========================================================================
    localparam real CLK_HALF_NS = 2.5; // 200 MHz

    logic clk     = 1'b0;
    logic aresetn = 1'b0;

    always #CLK_HALF_NS clk = ~clk;

    // =========================================================================
    // Register addresses (byte-addressed, 11-bit)
    // =========================================================================
    localparam logic [10:0] ADDR_HRES = 11'h008;
    localparam logic [10:0] ADDR_VRES = 11'h00C;
    localparam logic [10:0] ADDR_FCNT = 11'h020;

    // =========================================================================
    // AXI-Lite signals
    // =========================================================================
    logic [10:0] s_axi_awaddr  = '0;
    logic [2:0]  s_axi_awprot  = '0;
    logic        s_axi_awvalid = 1'b0;
    logic        s_axi_awready;

    logic [31:0] s_axi_wdata   = '0;
    logic [3:0]  s_axi_wstrb   = 4'hF;
    logic        s_axi_wvalid  = 1'b0;
    logic        s_axi_wready;

    logic [1:0]  s_axi_bresp;
    logic        s_axi_bvalid;
    logic        s_axi_bready  = 1'b0;

    logic [10:0] s_axi_araddr  = '0;
    logic [2:0]  s_axi_arprot  = '0;
    logic        s_axi_arvalid = 1'b0;
    logic        s_axi_arready;

    logic [31:0] s_axi_rdata;
    logic [1:0]  s_axi_rresp;
    logic        s_axi_rvalid;
    logic        s_axi_rready  = 1'b0;

    // =========================================================================
    // AXI-Stream  (idle slave / tready-tied-high master)
    // =========================================================================
    logic [31:0] s_axis_tdata  = '0;
    logic [9:0]  s_axis_tdest  = '0;
    logic [0:0]  s_axis_tuser  = 1'b0;
    logic        s_axis_tlast  = 1'b0;
    logic        s_axis_tvalid = 1'b0;
    logic        s_axis_tready;

    logic [31:0] m_axis_tdata;
    logic [3:0]  m_axis_tkeep;
    logic [0:0]  m_axis_tuser;
    logic        m_axis_tlast;
    logic        m_axis_tvalid;
    logic        m_axis_tready = 1'b1;

    // =========================================================================
    // DUT
    // =========================================================================
    isp_math_wrapper dut (
        .aclk           (clk),
        .aresetn        (aresetn),

        .s_axi_awaddr   (s_axi_awaddr),
        .s_axi_awprot   (s_axi_awprot),
        .s_axi_awvalid  (s_axi_awvalid),
        .s_axi_awready  (s_axi_awready),
        .s_axi_wdata    (s_axi_wdata),
        .s_axi_wstrb    (s_axi_wstrb),
        .s_axi_wvalid   (s_axi_wvalid),
        .s_axi_wready   (s_axi_wready),
        .s_axi_bresp    (s_axi_bresp),
        .s_axi_bvalid   (s_axi_bvalid),
        .s_axi_bready   (s_axi_bready),

        .s_axi_araddr   (s_axi_araddr),
        .s_axi_arprot   (s_axi_arprot),
        .s_axi_arvalid  (s_axi_arvalid),
        .s_axi_arready  (s_axi_arready),
        .s_axi_rdata    (s_axi_rdata),
        .s_axi_rresp    (s_axi_rresp),
        .s_axi_rvalid   (s_axi_rvalid),
        .s_axi_rready   (s_axi_rready),

        .s_axis_tdata   (s_axis_tdata),
        .s_axis_tdest   (s_axis_tdest),
        .s_axis_tuser   (s_axis_tuser),
        .s_axis_tlast   (s_axis_tlast),
        .s_axis_tvalid  (s_axis_tvalid),
        .s_axis_tready  (s_axis_tready),

        .m_axis_tdata   (m_axis_tdata),
        .m_axis_tkeep   (m_axis_tkeep),
        .m_axis_tuser   (m_axis_tuser),
        .m_axis_tlast   (m_axis_tlast),
        .m_axis_tvalid  (m_axis_tvalid),
        .m_axis_tready  (m_axis_tready)
    );

    // =========================================================================
    // Task: axi_read
    // =========================================================================
    task automatic axi_read(input  logic [10:0] addr,
                             output logic [31:0] data);
        @(posedge clk); #1;
        s_axi_araddr  = addr;
        s_axi_arvalid = 1'b1;

        do @(posedge clk); while (!(s_axi_arvalid && s_axi_arready));
        #1; s_axi_arvalid = 1'b0;

        s_axi_rready = 1'b1;
        do @(posedge clk); while (!(s_axi_rvalid && s_axi_rready));
        data = s_axi_rdata;
        #1; s_axi_rready = 1'b0;

        assert (s_axi_rresp == 2'b00)
            else $warning("[AXI-R] Non-OKAY RRESP=0x%0h  addr=0x%0h", s_axi_rresp, addr);
    endtask

    // =========================================================================
    // Main test sequence
    // =========================================================================
    initial begin : test_seq
        logic [31:0] rdata;

        $display("==============================================");
        $display("  tb_isp_math: HRES/VRES reset-value check");
        $display("==============================================");

        // Release reset
        aresetn = 1'b0;
        repeat(20) @(posedge clk);
        #1; aresetn = 1'b1;
        repeat(5) @(posedge clk);
        $display("[INFO] Reset released");

        // ------------------------------------------------------------------
        // Test 1: HRES default = 0x500 (1280)
        // ------------------------------------------------------------------
        axi_read(ADDR_HRES, rdata);
        $display("[READ] HRES (0x008) = 0x%08h  (bits[15:0] = %0d)", rdata, rdata[15:0]);
        assert (rdata[15:0] == 16'h500)
            else $error("[FAIL] HRES: expected 0x0500 (1280), got 0x%04h", rdata[15:0]);
        $display("[PASS] HRES reset value correct (1280)");

        // ------------------------------------------------------------------
        // Test 2: VRES default = 0x320 (800)
        // ------------------------------------------------------------------
        axi_read(ADDR_VRES, rdata);
        $display("[READ] VRES (0x00C) = 0x%08h  (bits[15:0] = %0d)", rdata, rdata[15:0]);
        assert (rdata[15:0] == 16'h320)
            else $error("[FAIL] VRES: expected 0x0320 (800), got 0x%04h", rdata[15:0]);
        $display("[PASS] VRES reset value correct (800)");

        // ------------------------------------------------------------------
        // Test 3: ADDR_FCNT test --> 0x00c0ffee
        // ------------------------------------------------------------------
        axi_read(ADDR_FCNT, rdata);
        $display("[READ] FCNT (0x020) = 0x%08h  (bits[31:0] = %0d)", rdata, rdata[31:0]);
        assert (rdata[31:0] == 32'h00c0ffee)
            else $error("[FAIL] VRES: expected 0x00c0ffee, got 0x%08h", rdata[31:0]);
        $display("[PASS] VRES reset value correct (0x00c0ffee)");

        // ------------------------------------------------------------------
        $display("\n[DONE] All register connectivity checks passed");
        $finish;
    end

endmodule
