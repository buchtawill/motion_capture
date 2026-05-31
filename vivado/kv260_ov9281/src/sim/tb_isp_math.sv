// tb_isp_math.sv
// Three-phase testbench for isp_math_wrapper (wraps isp_math_top ->
// PeakRDL-generated isp_regs + isp_histogram).
//
// Topology
//   tb --AXI-Lite--> isp_math_wrapper (DUT) --AXI-Stream--> tready (tied high)
//
// PHASE 1 — register peek/poke
//   Walks every register in the map: reset defaults, writes+readback,
//   snapshot latch, HIST_ADDR autoinc on/off + wrap, per-counter SW resets,
//   SW-reset preserves HRES/VRES.
//
// PHASE 2 — multi-resolution functional tests
//   For each supported resolution (1280x800, 640x400, 1280x720):
//   RTL reset -> program HRES/VRES -> HISTOGRAM_START -> stream a full frame
//   with random pixel data -> wait for HIST_DATA_VALID -> verify all 256 bins
//   and PIXEL_SUM against a golden reference -> verify frame_done_irq_o
//   latches high -> clear via CTRL.IRQ_CLEAR -> verify it deasserts.
//
// PHASE 3 — interrupt clear via CTRL.RESET
//   Verifies the alternate IRQ clear path: run a frame, confirm IRQ latches,
//   then clear via CTRL.RESET instead of IRQ_CLEAR.
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

    logic        frame_done_irq_o;

    // =========================================================================
    // DUT
    // =========================================================================
    isp_math_wrapper dut (
        .aclk              (clk),
        .aresetn           (aresetn),

        .s_axi_awaddr      (s_axi_awaddr),
        .s_axi_awprot      (s_axi_awprot),
        .s_axi_awvalid     (s_axi_awvalid),
        .s_axi_awready     (s_axi_awready),
        .s_axi_wdata       (s_axi_wdata),
        .s_axi_wstrb       (s_axi_wstrb),
        .s_axi_wvalid      (s_axi_wvalid),
        .s_axi_wready      (s_axi_wready),
        .s_axi_bresp       (s_axi_bresp),
        .s_axi_bvalid      (s_axi_bvalid),
        .s_axi_bready      (s_axi_bready),

        .s_axi_araddr      (s_axi_araddr),
        .s_axi_arprot      (s_axi_arprot),
        .s_axi_arvalid     (s_axi_arvalid),
        .s_axi_arready     (s_axi_arready),
        .s_axi_rdata       (s_axi_rdata),
        .s_axi_rresp       (s_axi_rresp),
        .s_axi_rvalid      (s_axi_rvalid),
        .s_axi_rready      (s_axi_rready),

        .s_axis_tdata      (s_axis_tdata),
        .s_axis_tdest      (s_axis_tdest),
        .s_axis_tuser      (s_axis_tuser),
        .s_axis_tlast      (s_axis_tlast),
        .s_axis_tvalid     (s_axis_tvalid),
        .s_axis_tready     (s_axis_tready),

        .m_axis_tdata      (m_axis_tdata),
        .m_axis_tkeep      (m_axis_tkeep),
        .m_axis_tuser      (m_axis_tuser),
        .m_axis_tlast      (m_axis_tlast),
        .m_axis_tvalid     (m_axis_tvalid),
        .m_axis_tready     (m_axis_tready),

        .frame_done_irq_o  (frame_done_irq_o)
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
    // Phase 2 — frame geometry, golden-reference state, AXIS driver helpers
    // =========================================================================
    localparam int FRAME_PIX_PER_BEAT = 4;

    // Beat cadence: 1 cycle tvalid high, BEAT_GAP cycles low. Gives the
    // histogram (throughput ~4 cycles/beat in S_ACTIVE) enough slack to
    // drain its internal FIFO continuously -- keeps fifo_err quiet.
    localparam int BEAT_GAP = 4;  // -> 5 cycles/beat total

    // Active frame geometry (set by run_frame_test before each measurement)
    int frame_hres;
    int frame_vres;
    int frame_total_beats;

    // Golden reference accumulators (reset at each send_frame call)
    int      exp_bin [256];
    longint  exp_sum;

    // ---------------------------------------------------------------------
    // send_beat — drive one 32b beat for exactly one clock, then gap.
    //   Updates golden bins + pixel sum for the four bytes.
    //   Caller must have the FSM armed (WAIT_TUSER or MEASURE) before the
    //   first beat, so the DUT captures every beat this task emits.
    // ---------------------------------------------------------------------
    task automatic send_beat(input logic [31:0] data,
                             input logic        tuser_in);
        logic [7:0] p0, p1, p2, p3;

        @(posedge clk); #1;
        s_axis_tdata  = data;
        s_axis_tuser  = tuser_in;
        s_axis_tvalid = 1'b1;

        @(posedge clk); #1;
        s_axis_tvalid = 1'b0;
        s_axis_tuser  = 1'b0;

        // inter-beat gap
        repeat (BEAT_GAP - 1) @(posedge clk);

        // Golden accumulators (4 bytes per beat, LSB first to match the
        // shift direction in isp_histogram)
        p0 = data[ 7: 0];
        p1 = data[15: 8];
        p2 = data[23:16];
        p3 = data[31:24];
        exp_bin[p0]++;
        exp_bin[p1]++;
        exp_bin[p2]++;
        exp_bin[p3]++;
        exp_sum += {24'h0, p0} + {24'h0, p1} + {24'h0, p2} + {24'h0, p3};
    endtask

    // ---------------------------------------------------------------------
    // send_frame — reset golden, then push FRAME_TOTAL_BEATS beats.
    //   First beat carries TUSER=1 (spec: MIPI CSI marks start-of-frame on
    //   the first valid beat of each frame).
    //   seed_val seeds the thread-local RNG so repeated runs are
    //   reproducible across simulations.
    // ---------------------------------------------------------------------
    task automatic send_frame(input int unsigned seed_val);
        int unsigned  _discard;
        logic [31:0]  beat_data;
        int           beats_per_line;

        beats_per_line = frame_hres / FRAME_PIX_PER_BEAT;

        $display("[INFO] send_frame(seed=%0d): %0d beats = %0d lines * %0d beats/line",
                 seed_val, frame_total_beats, frame_vres, beats_per_line);

        // Reset golden accumulators
        for (int i = 0; i < 256; i++) exp_bin[i] = 0;
        exp_sum = 0;

        _discard = $urandom(seed_val);

        for (int beat_ix = 0; beat_ix < frame_total_beats; beat_ix++) begin
            beat_data = $urandom();
            send_beat(beat_data, (beat_ix == 0));
        end
        $display("[INFO] send_frame done; exp_sum = %0d (0x%08h lower 32b)",
                 exp_sum, exp_sum[31:0]);
    endtask

    // ---------------------------------------------------------------------
    // wait_for_data_valid — poll STATUS.HIST_DATA_VALID with a timeout.
    //   Returns 1 on success, 0 on timeout.
    // ---------------------------------------------------------------------
    function automatic bit status_has_data_valid(input logic [31:0] status);
        return (status & `ISP_STATUS_HIST_DATA_VALID) != 0;
    endfunction

    task automatic wait_for_data_valid(input int max_polls,
                                        output bit ok);
        logic [31:0] status;
        int          poll_ix;

        ok = 1'b0;
        for (poll_ix = 0; poll_ix < max_polls; poll_ix++) begin
            axi_read(`ISP_REG_STATUS, status);
            if (status_has_data_valid(status)) begin
                ok = 1'b1;
                $display("[INFO] HIST_DATA_VALID asserted after %0d polls", poll_ix + 1);
                return;
            end
            repeat (4) @(posedge clk);
        end
        $display("[FAIL] HIST_DATA_VALID did not assert within %0d polls", max_polls);
    endtask

    // ---------------------------------------------------------------------
    // check_all_bins_and_sum — iterate 0..255 via HIST_ADDR / HIST_DATA
    //   frontdoor (using the autoinc), compare each bin to exp_bin[], and
    //   the PIXEL_SUM register to exp_sum. Reports pass/fail per region.
    // ---------------------------------------------------------------------
    task automatic check_all_bins_and_sum(input string frame_label);
        logic [31:0] status, rdata;
        int          mismatches;

        // Set HIST_ADDR = 0 and confirm AUTOINC is enabled
        axi_write(`ISP_REG_HIST_ADDR, 32'h0);
        axi_write(`ISP_REG_CTRL,      `ISP_CTRL_HIST_ADDR_AUTOINC);

        mismatches = 0;
        for (int i = 0; i < 256; i++) begin
            axi_read(`ISP_REG_HIST_DATA, rdata);
            if (rdata[19:0] !== exp_bin[i][19:0]) begin
                if (mismatches < 8) begin
                    $display("[FAIL]   bin[%0d]: got 0x%05h, expected 0x%05h",
                             i, rdata[19:0], exp_bin[i][19:0]);
                end
                mismatches++;
            end
        end
        report_check($sformatf("%s: all 256 histogram bins match golden reference",
                               frame_label),
                     mismatches == 0);

        // PIXEL_SUM: hardware holds the low 32 bits of the true sum
        axi_read(`ISP_REG_PIXEL_SUM, rdata);
        $display("[READ] PIXEL_SUM     (0x028) = 0x%08h  (expected 0x%08h)",
                 rdata, exp_sum[31:0]);
        report_check($sformatf("%s: PIXEL_SUM matches golden exp_sum[31:0]",
                               frame_label),
                     rdata === exp_sum[31:0]);

        // Status should still report HIST_DATA_VALID=1 through the read phase
        axi_read(`ISP_REG_STATUS, status);
        report_check($sformatf("%s: STATUS.HIST_DATA_VALID stays asserted during readback",
                               frame_label),
                     status_has_data_valid(status));
    endtask

    // ---------------------------------------------------------------------
    // arm_measurement — issue HISTOGRAM_START, then wait 300 cycles for the
    //   measurement-phase scrub to complete.
    // ---------------------------------------------------------------------
    task automatic arm_measurement();
        axi_write(`ISP_REG_CTRL,
                  `ISP_CTRL_HIST_ADDR_AUTOINC | `ISP_CTRL_HISTOGRAM_START);
        $display("[INFO] HISTOGRAM_START asserted; waiting 300 cycles for scrub");
        repeat (300) @(posedge clk);
    endtask

    // ---------------------------------------------------------------------
    // snap_and_read — pulse CTRL.SNAPSHOT, then return the three snapshot
    //   values coherent with each other.
    // ---------------------------------------------------------------------
    task automatic snap_and_read(output logic [31:0] frame_snap,
                                  output logic [31:0] cycle_snap_lo,
                                  output logic [31:0] cycle_snap_hi);
        axi_write(`ISP_REG_CTRL,
                  `ISP_CTRL_HIST_ADDR_AUTOINC | `ISP_CTRL_SNAPSHOT);
        repeat (2) @(posedge clk);
        axi_read(`ISP_REG_FRAME_SNAP,     frame_snap);
        axi_read(`ISP_REG_CYCLE_SNAP_LO,  cycle_snap_lo);
        axi_read(`ISP_REG_CYCLE_SNAP_HI,  cycle_snap_hi);
    endtask

    // ---------------------------------------------------------------------
    // run_frame_test — configure resolution, arm, stream, verify bins +
    //   pixel sum + interrupt latch/clear. Self-contained: resets DUT,
    //   programs HRES/VRES, runs one frame, checks everything.
    // ---------------------------------------------------------------------
    task automatic run_frame_test(input int hres,
                                  input int vres,
                                  input int unsigned seed);
        bit          dv_ok;
        logic [31:0] rdata;
        string       label;

        label = $sformatf("%0dx%0d", hres, vres);
        $display("\n----- run_frame_test %s (seed=%0d) -----", label, seed);

        // Set active geometry for send_frame
        frame_hres        = hres;
        frame_vres        = vres;
        frame_total_beats = (hres / FRAME_PIX_PER_BEAT) * vres;

        // RTL reset
        aresetn = 1'b0;
        repeat (20) @(posedge clk);
        #1; aresetn = 1'b1;
        repeat (5)  @(posedge clk);
        repeat (300) @(posedge clk);

        // Program resolution
        axi_write(`ISP_REG_HRES, hres);
        axi_write(`ISP_REG_VRES, vres);

        // IRQ should be low after reset
        report_check($sformatf("%s: frame_done_irq_o low after reset", label),
                     frame_done_irq_o === 1'b0);
        axi_read(`ISP_REG_STATUS, rdata);
        report_check($sformatf("%s: STATUS.FRAME_DONE_IRQ clear after reset", label),
                     (rdata & `ISP_STATUS_FRAME_DONE_IRQ) === 32'h0);

        // Arm and stream
        arm_measurement();
        send_frame(seed);

        // Wait for data valid
        wait_for_data_valid(.max_polls(1000), .ok(dv_ok));
        report_check($sformatf("%s: HIST_DATA_VALID observed", label), dv_ok);

        // Verify histogram bins and pixel sum
        check_all_bins_and_sum(label);

        // IRQ should be latched high now
        report_check($sformatf("%s: frame_done_irq_o latched high", label),
                     frame_done_irq_o === 1'b1);
        axi_read(`ISP_REG_STATUS, rdata);
        report_check($sformatf("%s: STATUS.FRAME_DONE_IRQ set", label),
                     (rdata & `ISP_STATUS_FRAME_DONE_IRQ) !== 32'h0);

        // Clear IRQ via CTRL.IRQ_CLEAR
        axi_write(`ISP_REG_CTRL, `ISP_CTRL_HIST_ADDR_AUTOINC | `ISP_CTRL_IRQ_CLEAR);
        repeat (2) @(posedge clk);

        report_check($sformatf("%s: frame_done_irq_o low after IRQ_CLEAR", label),
                     frame_done_irq_o === 1'b0);
        axi_read(`ISP_REG_STATUS, rdata);
        report_check($sformatf("%s: STATUS.FRAME_DONE_IRQ clear after IRQ_CLEAR", label),
                     (rdata & `ISP_STATUS_FRAME_DONE_IRQ) === 32'h0);

        // No FIFO overflow
        axi_read(`ISP_REG_STATUS, rdata);
        report_check($sformatf("%s: no HIST_FIFO_ERR", label),
                     (rdata & `ISP_STATUS_HIST_FIFO_ERR) === 32'h0);
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
        $display("  tb_isp_math: PHASE 1 — register peek/poke");
        $display("=========================================================");

        // Release reset
        aresetn = 1'b0;
        repeat (20) @(posedge clk);
        #1; aresetn = 1'b1;
        repeat (5)  @(posedge clk);
        $display("[INFO] Reset released");

        // Wait for the FSM's post-reset histogram scrub to complete so that
        // STATUS.READY is 1 and every register is in its quiescent state
        // before the peek/poke walk. Scrub is 256 cycles + a few cycles of
        // wrapper overhead; 300 covers it.
        repeat (300) @(posedge clk);
        $display("[INFO] Post-reset scrub complete, FSM in IDLE");

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
        report_check("STATUS = READY only (HIST_DATA_VALID=0, HIST_FIFO_ERR=0, FRAME_DONE_IRQ=0)",
                     (rdata & (`ISP_STATUS_READY
                             | `ISP_STATUS_HIST_DATA_VALID
                             | `ISP_STATUS_HIST_FIFO_ERR
                             | `ISP_STATUS_FRAME_DONE_IRQ)) === `ISP_STATUS_READY);

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

        // ==================================================================
        //                PHASE 2 — functional (all resolutions)
        // ==================================================================
        $display("");
        $display("=========================================================");
        $display("  tb_isp_math: PHASE 2 — multi-resolution frame tests");
        $display("=========================================================");

        run_frame_test(1280, 800, 32'h1);
        run_frame_test( 640, 400, 32'h2);
        run_frame_test(1280, 720, 32'h3);

        // ==================================================================
        //                PHASE 3 — interrupt clear via CTRL.RESET
        // ==================================================================
        $display("");
        $display("=========================================================");
        $display("  tb_isp_math: PHASE 3 — IRQ clear via CTRL.RESET");
        $display("=========================================================");

        // Run a frame to latch the IRQ, then clear it via SW reset instead
        // of IRQ_CLEAR, verifying a different clear path.
        begin : irq_reset_test
            bit          dv_ok;

            frame_hres        = 640;
            frame_vres        = 400;
            frame_total_beats = (640 / FRAME_PIX_PER_BEAT) * 400;

            aresetn = 1'b0;
            repeat (20) @(posedge clk);
            #1; aresetn = 1'b1;
            repeat (5)  @(posedge clk);
            repeat (300) @(posedge clk);

            axi_write(`ISP_REG_HRES, 640);
            axi_write(`ISP_REG_VRES, 400);

            arm_measurement();
            send_frame(32'hAA);

            wait_for_data_valid(.max_polls(1000), .ok(dv_ok));
            report_check("IRQ-reset: HIST_DATA_VALID observed", dv_ok);

            // IRQ should be latched
            report_check("IRQ-reset: frame_done_irq_o latched high",
                         frame_done_irq_o === 1'b1);

            // Clear via CTRL.RESET (not IRQ_CLEAR)
            axi_write(`ISP_REG_CTRL, `ISP_CTRL_HIST_ADDR_AUTOINC | `ISP_CTRL_RESET);
            repeat (300) @(posedge clk);

            report_check("IRQ-reset: frame_done_irq_o low after CTRL.RESET",
                         frame_done_irq_o === 1'b0);
            axi_read(`ISP_REG_STATUS, rdata);
            report_check("IRQ-reset: STATUS.FRAME_DONE_IRQ clear after CTRL.RESET",
                         (rdata & `ISP_STATUS_FRAME_DONE_IRQ) === 32'h0);
        end

        // ------------------------------------------------------------------
        $display("");
        $display("=========================================================");
        $display("  tb_isp_math: %0d passed, %0d failed", n_pass, n_fail);
        $display("=========================================================");
        if (n_fail == 0)
            $display("[DONE] All checks passed (Phase 1 + Phase 2 + Phase 3)");
        else
            $display("[DONE] FAILURES PRESENT -- see [FAIL] lines above");
        $finish;
    end

endmodule
