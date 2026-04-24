// tb_isp_math.sv
// Register-level sanity testbench for isp_math_wrapper (wraps isp_math_top ->
// PeakRDL-generated isp_regs).
//
// Topology
//   tb --AXI-Lite--> isp_math_wrapper (DUT) --AXI-Stream--> tready (tied high)
//
// Scope: connectivity + behavior of each register in the map. The histogram
// FSM, isp_histogram, and pixel-sum counter are not exercised here (stubbed in
// isp_math_top); those get their own testbenches.
//
// Register addresses and bit positions are sourced from the single
// spec-of-truth sidecar file isp_regs_defines.svh (which mirrors isp_regs.rdl).

`timescale 1ns / 1ps

`include "isp_regs_defines.svh"

module tb_isp_math;

    // =========================================================================
    // Clock
    // =========================================================================
    localparam real CLK_HALF_NS = 2.5; // 200 MHz

    logic clk     = 1'b0;
    logic aresetn = 1'b0;

    always #CLK_HALF_NS clk = ~clk;

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
    // Pass / fail bookkeeping
    // =========================================================================
    int n_pass = 0;
    int n_fail = 0;

    task automatic report_check(input string label, input bit ok);
        if (ok) begin
            $display("[PASS] %s", label);
            n_pass++;
        end else begin
            $display("[FAIL] %s", label);
            n_fail++;
        end
    endtask

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
    // Task: axi_write
    // =========================================================================
    task automatic axi_write(input logic [10:0] addr,
                             input logic [31:0] data);
        @(posedge clk); #1;
        s_axi_awaddr  = addr;
        s_axi_awvalid = 1'b1;
        s_axi_wdata   = data;
        s_axi_wstrb   = 4'hF;
        s_axi_wvalid  = 1'b1;

        // Wait for both AW and W handshakes (may occur on different cycles).
        fork
            begin
                do @(posedge clk); while (!(s_axi_awvalid && s_axi_awready));
                #1; s_axi_awvalid = 1'b0;
            end
            begin
                do @(posedge clk); while (!(s_axi_wvalid && s_axi_wready));
                #1; s_axi_wvalid = 1'b0;
            end
        join

        s_axi_bready = 1'b1;
        do @(posedge clk); while (!(s_axi_bvalid && s_axi_bready));
        #1; s_axi_bready = 1'b0;

        assert (s_axi_bresp == 2'b00)
            else $warning("[AXI-W] Non-OKAY BRESP=0x%0h  addr=0x%0h data=0x%0h",
                          s_axi_bresp, addr, data);
    endtask

    // =========================================================================
    // Main test sequence
    // =========================================================================
    initial begin : test_seq
        logic [31:0] rdata;
        logic [31:0] rdata2;
        logic [31:0] snap_lo_val;
        logic [31:0] snap_hi_val;

        $display("=========================================================");
        $display("  tb_isp_math: register-level sanity checks");
        $display("=========================================================");

        // Release reset
        aresetn = 1'b0;
        repeat (20) @(posedge clk);
        #1; aresetn = 1'b1;
        repeat (5)  @(posedge clk);
        $display("[INFO] Reset released");

        // ------------------------------------------------------------------
        // 0x08 HRES -- default + write/readback
        // ------------------------------------------------------------------
        axi_read(`ISP_REG_HRES, rdata);
        $display("[READ] HRES          (0x008) = 0x%08h", rdata);
        report_check("HRES reset value = 1280 (0x500)",
                     rdata[`ISP_HRES_WIDTH-1:0] === `ISP_HRES_RESET);

        axi_write(`ISP_REG_HRES, 32'h0000_0ABC);
        axi_read (`ISP_REG_HRES, rdata);
        report_check("HRES readback after write 0xABC",
                     rdata[`ISP_HRES_WIDTH-1:0] === 16'h0ABC);

        // ------------------------------------------------------------------
        // 0x0C VRES -- default + write/readback
        // ------------------------------------------------------------------
        axi_read(`ISP_REG_VRES, rdata);
        $display("[READ] VRES          (0x00C) = 0x%08h", rdata);
        report_check("VRES reset value = 800 (0x320)",
                     rdata[`ISP_VRES_WIDTH-1:0] === `ISP_VRES_RESET);

        axi_write(`ISP_REG_VRES, 32'h0000_0DEF);
        axi_read (`ISP_REG_VRES, rdata);
        report_check("VRES readback after write 0xDEF",
                     rdata[`ISP_VRES_WIDTH-1:0] === 16'h0DEF);

        // ------------------------------------------------------------------
        // 0x00 CTRL -- default reads as HIST_ADDR_AUTOINC only
        // ------------------------------------------------------------------
        axi_read(`ISP_REG_CTRL, rdata);
        $display("[READ] CTRL          (0x000) = 0x%08h", rdata);
        report_check("CTRL reset value = HIST_ADDR_AUTOINC only (W-pulse bits read 0)",
                     rdata === `ISP_CTRL_RESET_VALUE);

        // Clear HIST_ADDR_AUTOINC
        axi_write(`ISP_REG_CTRL, 32'h0);
        axi_read (`ISP_REG_CTRL, rdata);
        report_check("CTRL = 0 clears HIST_ADDR_AUTOINC", rdata === 32'h0);

        // Restore HIST_ADDR_AUTOINC = 1
        axi_write(`ISP_REG_CTRL, `ISP_CTRL_HIST_ADDR_AUTOINC);
        axi_read (`ISP_REG_CTRL, rdata);
        report_check("CTRL restore sets HIST_ADDR_AUTOINC back to 1",
                     rdata === `ISP_CTRL_HIST_ADDR_AUTOINC);

        // ------------------------------------------------------------------
        // 0x04 STATUS -- readable. Stub drives READY=1, others = 0.
        // ------------------------------------------------------------------
        axi_read(`ISP_REG_STATUS, rdata);
        $display("[READ] STATUS        (0x004) = 0x%08h", rdata);
        report_check("STATUS = READY only (stubbed: HIST_DATA_VALID=0, HIST_FIFO_ERR=0)",
                     (rdata & (`ISP_STATUS_READY
                             | `ISP_STATUS_HIST_DATA_VALID
                             | `ISP_STATUS_HIST_FIFO_ERR)) === `ISP_STATUS_READY);

        // ------------------------------------------------------------------
        // 0x10 CYCLE_CNT_LO -- free-running between reads
        // ------------------------------------------------------------------
        axi_read(`ISP_REG_CYCLE_CNT_LO, rdata);
        axi_read(`ISP_REG_CYCLE_CNT_LO, rdata2);
        $display("[READ] CYCLE_CNT_LO: first=0x%08h  second=0x%08h", rdata, rdata2);
        report_check("CYCLE_CNT_LO advances between reads", rdata2 > rdata);

        // ------------------------------------------------------------------
        // 0x14 CYCLE_CNT_HI -- 0 (no 32b wrap in this short test)
        // ------------------------------------------------------------------
        axi_read(`ISP_REG_CYCLE_CNT_HI, rdata);
        $display("[READ] CYCLE_CNT_HI  (0x014) = 0x%08h", rdata);
        report_check("CYCLE_CNT_HI = 0 (no wrap in short test)", rdata === 32'h0);

        // ------------------------------------------------------------------
        // 0x18 / 0x1C CYCLE_SNAP_* -- 0 before any SNAPSHOT pulse
        // ------------------------------------------------------------------
        axi_read(`ISP_REG_CYCLE_SNAP_LO, rdata);
        report_check("CYCLE_SNAP_LO = 0 before SNAPSHOT", rdata === 32'h0);
        axi_read(`ISP_REG_CYCLE_SNAP_HI, rdata);
        report_check("CYCLE_SNAP_HI = 0 before SNAPSHOT", rdata === 32'h0);

        // ------------------------------------------------------------------
        // CTRL.SNAPSHOT latches CYCLE_SNAP_* and FRAME_SNAP
        // ------------------------------------------------------------------
        axi_write(`ISP_REG_CTRL, `ISP_CTRL_HIST_ADDR_AUTOINC | `ISP_CTRL_SNAPSHOT);
        repeat (2) @(posedge clk);

        axi_read(`ISP_REG_CYCLE_CNT_LO,      rdata2);
        axi_read(`ISP_REG_CYCLE_SNAP_LO,     snap_lo_val);
        axi_read(`ISP_REG_CYCLE_SNAP_HI,     snap_hi_val);
        $display("[READ] CYCLE_SNAP_LO (0x018) = 0x%08h (CYCLE_CNT_LO now 0x%08h)",
                 snap_lo_val, rdata2);
        report_check("CYCLE_SNAP_LO latched a non-zero value",  snap_lo_val !== 32'h0);
        report_check("CYCLE_SNAP_LO <= current CYCLE_CNT_LO",   snap_lo_val <= rdata2);
        report_check("CYCLE_SNAP_HI = 0 after first snapshot",  snap_hi_val === 32'h0);

        // ------------------------------------------------------------------
        // 0x20 FRAME_CNT -- 0 (no TUSER activity)
        // ------------------------------------------------------------------
        axi_read(`ISP_REG_FRAME_CNT, rdata);
        $display("[READ] FRAME_CNT     (0x020) = 0x%08h", rdata);
        report_check("FRAME_CNT = 0 without TUSER activity", rdata === 32'h0);

        // ------------------------------------------------------------------
        // 0x24 FRAME_SNAP -- latched 0 after SNAPSHOT above
        // ------------------------------------------------------------------
        axi_read(`ISP_REG_FRAME_SNAP, rdata);
        report_check("FRAME_SNAP = 0 (FRAME_CNT was 0 at snapshot)", rdata === 32'h0);

        // ------------------------------------------------------------------
        // 0x28 PIXEL_SUM -- stubbed at 0
        // ------------------------------------------------------------------
        axi_read(`ISP_REG_PIXEL_SUM, rdata);
        $display("[READ] PIXEL_SUM     (0x028) = 0x%08h", rdata);
        report_check("PIXEL_SUM = 0 before any measurement", rdata === 32'h0);

        // ------------------------------------------------------------------
        // 0x2C HIST_ADDR -- write / readback, upper bits reserved
        // ------------------------------------------------------------------
        axi_write(`ISP_REG_HIST_ADDR, 32'hDEAD_BE42);
        axi_read (`ISP_REG_HIST_ADDR, rdata);
        $display("[READ] HIST_ADDR     (0x02C) = 0x%08h", rdata);
        report_check("HIST_ADDR stores low 8 bits only (bits [31:8] reserved)",
                     rdata === 32'h0000_0042);

        // ------------------------------------------------------------------
        // 0x30 HIST_DATA -- RO, stubbed at 0; read auto-increments HIST_ADDR
        // ------------------------------------------------------------------
        // NOTE: AUTOINC is enabled here, so this read bumps HIST_ADDR 0x42 -> 0x43.
        axi_read(`ISP_REG_HIST_DATA, rdata);
        $display("[READ] HIST_DATA     (0x030) = 0x%08h", rdata);
        report_check("HIST_DATA = 0 (stubbed); bits [31:20] reserved/zero",
                     rdata === 32'h0);

        axi_read(`ISP_REG_HIST_ADDR, rdata);
        report_check("HIST_ADDR auto-incremented 0x42 -> 0x43 after HIST_DATA read",
                     rdata[`ISP_HIST_ADDR_WIDTH-1:0] === 8'h43);

        // ------------------------------------------------------------------
        // HIST_ADDR auto-increment disabled: HIST_DATA read leaves HIST_ADDR alone
        // ------------------------------------------------------------------
        axi_write(`ISP_REG_CTRL,      32'h0);
        axi_write(`ISP_REG_HIST_ADDR, 32'h0000_0080);
        axi_read (`ISP_REG_HIST_DATA, rdata);
        axi_read (`ISP_REG_HIST_ADDR, rdata);
        report_check("HIST_ADDR unchanged after HIST_DATA read with AUTOINC=0",
                     rdata[`ISP_HIST_ADDR_WIDTH-1:0] === 8'h80);

        // Restore AUTOINC = 1
        axi_write(`ISP_REG_CTRL, `ISP_CTRL_HIST_ADDR_AUTOINC);

        // ------------------------------------------------------------------
        // HIST_ADDR wraps at 0xFF -> 0x00 on next HIST_DATA read
        // ------------------------------------------------------------------
        axi_write(`ISP_REG_HIST_ADDR, 32'h0000_00FF);
        axi_read (`ISP_REG_HIST_DATA, rdata);
        axi_read (`ISP_REG_HIST_ADDR, rdata);
        report_check("HIST_ADDR wraps 0xFF -> 0x00 under autoinc",
                     rdata[`ISP_HIST_ADDR_WIDTH-1:0] === 8'h00);

        // ------------------------------------------------------------------
        // CTRL.CYCLE_CNT_RESET -- drops CYCLE_CNT_LO to ~0
        // ------------------------------------------------------------------
        axi_read(`ISP_REG_CYCLE_CNT_LO, rdata);
        $display("[READ] CYCLE_CNT_LO before CYCLE_CNT_RESET = 0x%08h", rdata);
        axi_write(`ISP_REG_CTRL,
                  `ISP_CTRL_HIST_ADDR_AUTOINC | `ISP_CTRL_CYCLE_CNT_RESET);
        repeat (2) @(posedge clk);
        axi_read(`ISP_REG_CYCLE_CNT_LO, rdata2);
        $display("[READ] CYCLE_CNT_LO after  CYCLE_CNT_RESET = 0x%08h", rdata2);
        report_check("CYCLE_CNT_RESET drops CYCLE_CNT_LO well below prior reading",
                     rdata2 < rdata);

        // ------------------------------------------------------------------
        // CTRL.FRAME_CNT_RESET -- FRAME_CNT stays 0 (no TUSER traffic);
        // the write must complete cleanly
        // ------------------------------------------------------------------
        axi_write(`ISP_REG_CTRL,
                  `ISP_CTRL_HIST_ADDR_AUTOINC | `ISP_CTRL_FRAME_CNT_RESET);
        repeat (2) @(posedge clk);
        axi_read(`ISP_REG_FRAME_CNT, rdata);
        report_check("FRAME_CNT = 0 after FRAME_CNT_RESET (no TUSER traffic)",
                     rdata === 32'h0);

        // ------------------------------------------------------------------
        // CTRL.RESET -- preserves HRES/VRES, clears snapshots
        // ------------------------------------------------------------------
        axi_write(`ISP_REG_HRES, 32'h0000_0ABC);
        axi_write(`ISP_REG_VRES, 32'h0000_0DEF);
        axi_write(`ISP_REG_CTRL,
                  `ISP_CTRL_HIST_ADDR_AUTOINC | `ISP_CTRL_RESET);
        repeat (2) @(posedge clk);

        axi_read(`ISP_REG_HRES, rdata);
        report_check("SW RESET preserves HRES (spec: HRES not reset)",
                     rdata[`ISP_HRES_WIDTH-1:0] === 16'h0ABC);
        axi_read(`ISP_REG_VRES, rdata);
        report_check("SW RESET preserves VRES (spec: VRES not reset)",
                     rdata[`ISP_VRES_WIDTH-1:0] === 16'h0DEF);

        axi_read(`ISP_REG_CYCLE_SNAP_LO, rdata);
        report_check("SW RESET clears CYCLE_SNAP_LO", rdata === 32'h0);
        axi_read(`ISP_REG_FRAME_SNAP, rdata);
        report_check("SW RESET clears FRAME_SNAP", rdata === 32'h0);

        // ------------------------------------------------------------------
        $display("=========================================================");
        $display("  tb_isp_math: %0d passed, %0d failed", n_pass, n_fail);
        $display("=========================================================");
        if (n_fail == 0)
            $display("[DONE] All register sanity checks passed");
        else
            $display("[DONE] FAILURES PRESENT -- see [FAIL] lines above");
        $finish;
    end

endmodule
