// tb_frame_rate.sv
// Testbench for frame_rate_counter.sv
//
// Topology
//   stream_source ──► frame_rate_counter (DUT) ──► m_axis_tready (tied high)
//
// Clock      : 200 MHz  (5 ns period)
// Backpressure: none  — m_axis_tready is permanently asserted
// Stream     : TUSER pulse (one cycle per frame) at FRAME_RATE_FPS.
//              Change the module parameter to run at a different rate.
//
// Simulation time estimate (default 60 fps, 200 MHz):
//   ~337 M clock cycles  ≈ 1.68 s of simulation time.
//   Use a higher FRAME_RATE_FPS value to shorten simulation (e.g. 6000).
//
// Test plan
//   1. Basic measurement   — enable, run full 100-frame count, verify FPS ±1%
//   2. sw_reset mid-count  — assert reset while busy, re-run to completion
//   3. Disable mid-count   — clear enable bit while busy, verify idle

`timescale 1ns / 1ps

module tb_frame_rate #(
    parameter int FRAME_RATE_FPS = 6000
);

    // =========================================================================
    // Derived constants
    // =========================================================================
    localparam int  CLK_FREQ_HZ      = 200_000_000;
    localparam real CLK_HALF_NS      = 2.5;                           // 5 ns period
    localparam int  CYCLES_PER_FRAME = CLK_FREQ_HZ / FRAME_RATE_FPS; // integer truncation OK

    // Register byte offsets
    localparam logic [3:0] ADDR_CTRL        = 4'h0;
    localparam logic [3:0] ADDR_STATUS      = 4'h4;
    localparam logic [3:0] ADDR_CYCLE_COUNT = 4'h8;

    // Control bit positions
    localparam int CTRL_ENABLE_BIT = 0;
    localparam int CTRL_RESET_BIT  = 1;

    // Status bit positions
    localparam int STATUS_IDLE_BIT = 0;
    localparam int STATUS_BUSY_BIT = 1;
    localparam int STATUS_DONE_BIT = 2;

    // =========================================================================
    // Signal declarations
    // =========================================================================
    logic clk     = 1'b0;
    logic aresetn = 1'b0;

    // ── AXI-Lite (all driven from test tasks) ────────────────────────────────
    logic [3:0]  s_axi_awaddr  = '0;
    logic        s_axi_awvalid = 1'b0;
    logic        s_axi_awready;

    logic [31:0] s_axi_wdata   = '0;
    logic [3:0]  s_axi_wstrb   = 4'hF;
    logic        s_axi_wvalid  = 1'b0;
    logic        s_axi_wready;

    logic [1:0]  s_axi_bresp;
    logic        s_axi_bvalid;
    logic        s_axi_bready  = 1'b0;

    logic [3:0]  s_axi_araddr  = '0;
    logic        s_axi_arvalid = 1'b0;
    logic        s_axi_arready;

    logic [31:0] s_axi_rdata;
    logic [1:0]  s_axi_rresp;
    logic        s_axi_rvalid;
    logic        s_axi_rready  = 1'b0;

    // ── AXI-Stream slave (driven by stream source process) ───────────────────
    logic [31:0] s_axis_tdata  = '0;
    logic [3:0]  s_axis_tkeep  = 4'hF;
    logic        s_axis_tuser  = 1'b0;
    logic        s_axis_tlast  = 1'b0;
    logic        s_axis_tvalid = 1'b0;
    logic        s_axis_tready;          // output from DUT

    // ── AXI-Stream master (downstream consumer) ──────────────────────────────
    logic [31:0] m_axis_tdata;
    logic [3:0]  m_axis_tkeep;
    logic        m_axis_tuser;
    logic        m_axis_tlast;
    logic        m_axis_tvalid;
    logic        m_axis_tready = 1'b1;  // no backpressure — permanently tied high

    // =========================================================================
    // DUT instantiation
    // =========================================================================
    frame_rate_counter #(
        .AXIS_DATA_WIDTH  (32),
        .AXIS_TUSER_WIDTH (1),
        .AXIS_TKEEP_WIDTH (4)
    ) dut (
        .aclk           (clk),
        .aresetn        (aresetn),
        // AXI-Lite
        .s_axi_awaddr   (s_axi_awaddr),
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
        .s_axi_arvalid  (s_axi_arvalid),
        .s_axi_arready  (s_axi_arready),
        .s_axi_rdata    (s_axi_rdata),
        .s_axi_rresp    (s_axi_rresp),
        .s_axi_rvalid   (s_axi_rvalid),
        .s_axi_rready   (s_axi_rready),
        // AXI-Stream slave
        .s_axis_tdata   (s_axis_tdata),
        .s_axis_tkeep   (s_axis_tkeep),
        .s_axis_tuser   (s_axis_tuser),
        .s_axis_tlast   (s_axis_tlast),
        .s_axis_tvalid  (s_axis_tvalid),
        .s_axis_tready  (s_axis_tready),
        // AXI-Stream master
        .m_axis_tdata   (m_axis_tdata),
        .m_axis_tkeep   (m_axis_tkeep),
        .m_axis_tuser   (m_axis_tuser),
        .m_axis_tlast   (m_axis_tlast),
        .m_axis_tvalid  (m_axis_tvalid),
        .m_axis_tready  (m_axis_tready)
    );

    // =========================================================================
    // Clock generation
    // =========================================================================
    always #CLK_HALF_NS clk = ~clk;

    // =========================================================================
    // Task: axi_write
    //
    //   Performs a single AXI-Lite write transaction.
    //   AW and W channels are driven simultaneously and monitored concurrently
    //   via a fork so neither channel is missed if both fire in the same cycle.
    // =========================================================================
    task automatic axi_write(input logic [3:0]  addr,
                              input logic [31:0] data,
                              input logic [3:0]  strb = 4'hF);
        // Present address and data channels together after a clock edge
        @(posedge clk); #1;
        s_axi_awaddr  = addr;
        s_axi_awvalid = 1'b1;
        s_axi_wdata   = data;
        s_axi_wstrb   = strb;
        s_axi_wvalid  = 1'b1;

        // Monitor AW and W channels in parallel — the DUT accepts both in one
        // cycle when awready=wready=1 (the normal case); the fork handles any
        // ordering correctly without risking a missed handshake on either channel.
        fork
            begin : aw_ch
                do @(posedge clk); while (!(s_axi_awvalid && s_axi_awready));
                #1; s_axi_awvalid = 1'b0;
            end
            begin : w_ch
                do @(posedge clk); while (!(s_axi_wvalid && s_axi_wready));
                #1; s_axi_wvalid = 1'b0;
            end
        join

        // Accept write response
        s_axi_bready = 1'b1;
        do @(posedge clk); while (!(s_axi_bvalid && s_axi_bready));
        #1; s_axi_bready = 1'b0;

        assert (s_axi_bresp == 2'b00)
            else $warning("[AXI-W] Non-OKAY BRESP=0x%0h  addr=0x%0h  data=0x%0h",
                          s_axi_bresp, addr, data);
    endtask

    // =========================================================================
    // Task: axi_read
    //
    //   Performs a single AXI-Lite read transaction.
    //   The DUT registers rvalid one cycle after the AR handshake, so rvalid
    //   is checked on the cycle following AR acceptance.
    // =========================================================================
    task automatic axi_read(input  logic [3:0]  addr,
                             output logic [31:0] data);
        @(posedge clk); #1;
        s_axi_araddr  = addr;
        s_axi_arvalid = 1'b1;

        do @(posedge clk); while (!(s_axi_arvalid && s_axi_arready));
        #1; s_axi_arvalid = 1'b0;

        s_axi_rready = 1'b1;
        do @(posedge clk); while (!(s_axi_rvalid && s_axi_rready));
        data = s_axi_rdata;   // capture on the handshake cycle
        #1; s_axi_rready = 1'b0;

        assert (s_axi_rresp == 2'b00)
            else $warning("[AXI-R] Non-OKAY RRESP=0x%0h  addr=0x%0h", s_axi_rresp, addr);
    endtask

    // =========================================================================
    // Task: apply_reset
    //
    //   Asserts aresetn low for `cycles` clocks then releases it.
    // =========================================================================
    task automatic apply_reset(input int cycles = 20);
        aresetn = 1'b0;
        repeat(cycles) @(posedge clk);
        #1; aresetn = 1'b1;
        @(posedge clk);
    endtask

    // =========================================================================
    // Task: poll_until_done
    //
    //   Polls the status register every half-frame until the DONE bit is set
    //   or the timeout (max_polls half-frame periods) is exceeded.
    //   Prints a one-line update on every poll.
    // =========================================================================
    task automatic poll_until_done(input int max_polls = 220);
        logic [31:0] status;
        int          poll_num;
        int          half_frame;

        half_frame = CYCLES_PER_FRAME / 2;
        poll_num   = 0;

        do begin
            repeat(half_frame) @(posedge clk);
            axi_read(ADDR_STATUS, status);
            poll_num++;
            $display("  [POLL %0d]  status=0x%0h  (%s%s%s)",
                     poll_num, status,
                     status[STATUS_IDLE_BIT] ? "IDLE " : "",
                     status[STATUS_BUSY_BIT] ? "BUSY " : "",
                     status[STATUS_DONE_BIT] ? "DONE"  : "");
        end while (!status[STATUS_DONE_BIT] && poll_num < max_polls);

        if (!status[STATUS_DONE_BIT])
            $error("[FAIL] poll_until_done: DONE not seen after %0d half-frame polls", max_polls);
    endtask

    // =========================================================================
    // Stream source (separate initial block — runs independently of tests)
    //
    //   Generates a one-cycle TUSER pulse (TVALID=1, TUSER=1) at FRAME_RATE_FPS.
    //   Starts after reset is released; runs continuously for the entire sim.
    // =========================================================================
    initial begin : stream_source
        // Hold stream idle until reset is released
        @(posedge aresetn);
        @(posedge clk);

        forever begin : frame_loop
            // Start-of-frame: one clock with TVALID and TUSER high
            @(posedge clk); #1;
            s_axis_tvalid = 1'b1;
            s_axis_tuser  = 1'b1;
            s_axis_tdata  = $urandom();
            s_axis_tkeep  = 4'hF;
            s_axis_tlast  = 1'b0;

            @(posedge clk); #1;
            s_axis_tvalid = 1'b0;
            s_axis_tuser  = 1'b0;

            // Wait the remaining frame period (subtract the two cycles just used)
            if (CYCLES_PER_FRAME > 2)
                repeat(CYCLES_PER_FRAME - 2) @(posedge clk);
        end
    end

    // =========================================================================
    // Main test sequence
    // =========================================================================
    initial begin : test_seq
        logic [31:0] ctrl_rd, status, cycle_count;
        real         measured_fps;

        $display("========================================================");
        $display("  frame_rate_counter testbench");
        $display("  CLK  = %0d MHz", CLK_FREQ_HZ / 1_000_000);
        $display("  FPS  = %0d", FRAME_RATE_FPS);
        $display("  CPF  = %0d cycles/frame", CYCLES_PER_FRAME);
        $display("========================================================");

        // -----------------------------------------------------------------------
        // Hardware reset
        // -----------------------------------------------------------------------
        apply_reset(20);
        $display("\n[INFO] Reset released");

        // -----------------------------------------------------------------------
        // Test 1: Basic 100-frame measurement
        // -----------------------------------------------------------------------
        $display("\n--- Test 1: Basic measurement (%0d fps, 100 frames) ---",
                 FRAME_RATE_FPS);

        // Status should be IDLE immediately after reset
        axi_read(ADDR_STATUS, status);
        assert (status[STATUS_IDLE_BIT] && !status[STATUS_BUSY_BIT] && !status[STATUS_DONE_BIT])
            else $error("[FAIL] T1: expected IDLE after reset, got status=0x%0h", status);
        $display("[PASS] T1: status=IDLE after reset");

        // Verify sw_reset bit reads 0 in control register
        axi_read(ADDR_CTRL, ctrl_rd);
        assert (!ctrl_rd[CTRL_RESET_BIT])
            else $error("[FAIL] T1: sw_reset should always read 0, got ctrl=0x%0h", ctrl_rd);
        assert (!ctrl_rd[CTRL_ENABLE_BIT])
            else $error("[FAIL] T1: enable should be 0 after reset");
        $display("[PASS] T1: ctrl register correct after reset (0x%0h)", ctrl_rd);

        // Enable DUT
        axi_write(ADDR_CTRL, 32'h1);  // enable=1
        axi_read(ADDR_CTRL, ctrl_rd);
        assert (ctrl_rd[CTRL_ENABLE_BIT])
            else $error("[FAIL] T1: enable bit did not stick, ctrl=0x%0h", ctrl_rd);
        $display("[INFO] T1: DUT enabled");

        // Poll until DONE
        poll_until_done(220);

        // Read and validate result
        axi_read(ADDR_CYCLE_COUNT, cycle_count);
        axi_read(ADDR_STATUS, status);

        assert (status[STATUS_DONE_BIT])
            else $error("[FAIL] T1: DONE bit not set after poll");
        assert (!status[STATUS_BUSY_BIT])
            else $error("[FAIL] T1: BUSY still set after DONE");

        measured_fps = (100.0 * CLK_FREQ_HZ) / cycle_count;
        $display("[RESULT] T1: cycle_count=%0d  measured_fps=%.3f  expected=%0d fps",
                 cycle_count, measured_fps, FRAME_RATE_FPS);

        // Allow ±1% tolerance (integer CYCLES_PER_FRAME truncation causes sub-0.1% error)
        assert (measured_fps > FRAME_RATE_FPS * 0.99 &&
                measured_fps < FRAME_RATE_FPS * 1.01)
            else $error("[FAIL] T1: measured FPS %.3f outside ±1%% of %0d",
                        measured_fps, FRAME_RATE_FPS);
        $display("[PASS] T1: FPS within ±1%% tolerance");

        // Verify sw_reset bit still reads 0
        axi_read(ADDR_CTRL, ctrl_rd);
        assert (!ctrl_rd[CTRL_RESET_BIT])
            else $error("[FAIL] T1: sw_reset should always read 0, got ctrl=0x%0h", ctrl_rd);

        // -----------------------------------------------------------------------
        // Test 2: sw_reset mid-measurement
        // -----------------------------------------------------------------------
        $display("\n--- Test 2: sw_reset mid-measurement ---");

        // Disable first to clear DONE state, then re-enable for a fresh run
        axi_write(ADDR_CTRL, 32'h0);
        axi_write(ADDR_CTRL, 32'h1);

        // Wait approximately 50 frames — DUT should be busy counting
        repeat(CYCLES_PER_FRAME * 50) @(posedge clk);

        axi_read(ADDR_STATUS, status);
        $display("[INFO] T2: status after ~50 frames: 0x%0h  (%s%s%s)",
                 status,
                 status[STATUS_IDLE_BIT] ? "IDLE " : "",
                 status[STATUS_BUSY_BIT] ? "BUSY " : "",
                 status[STATUS_DONE_BIT] ? "DONE"  : "");
        assert (status[STATUS_BUSY_BIT])
            else $warning("[WARN] T2: expected BUSY at 50-frame mark, got 0x%0h (stream may not have started yet)", status);

        // Assert sw_reset (bit 1) while keeping enable asserted
        axi_write(ADDR_CTRL, 32'h3);  // enable=1, sw_reset=1

        // sw_reset is self-clearing — must always read 0
        axi_read(ADDR_CTRL, ctrl_rd);
        assert (!ctrl_rd[CTRL_RESET_BIT])
            else $error("[FAIL] T2: sw_reset did not read back 0, ctrl=0x%0h", ctrl_rd);
        $display("[PASS] T2: sw_reset bit reads 0 after write");

        // State machine must have returned to IDLE (or WAIT_SOF since enable=0 after reset)
        repeat(5) @(posedge clk);
        axi_read(ADDR_STATUS, status);
        assert (status[STATUS_IDLE_BIT] && !status[STATUS_DONE_BIT] && !status[STATUS_BUSY_BIT])
            else $error("[FAIL] T2: expected IDLE after sw_reset, got status=0x%0h", status);
        $display("[PASS] T2: state returned to IDLE after sw_reset");

        // Re-enable and run a full measurement to confirm recovery
        $display("[INFO] T2: re-enabling after sw_reset for full re-run...");
        axi_write(ADDR_CTRL, 32'h1);
        poll_until_done(220);
        axi_read(ADDR_CYCLE_COUNT, cycle_count);
        axi_read(ADDR_STATUS, status);
        assert (status[STATUS_DONE_BIT])
            else $error("[FAIL] T2: re-run after sw_reset did not complete");
        measured_fps = (100.0 * CLK_FREQ_HZ) / cycle_count;
        $display("[PASS] T2: re-run complete  cycle_count=%0d  fps=%.3f",
                 cycle_count, measured_fps);

        // -----------------------------------------------------------------------
        // Test 3: Disable mid-measurement
        // -----------------------------------------------------------------------
        $display("\n--- Test 3: Disable mid-measurement ---");

        // Start fresh
        axi_write(ADDR_CTRL, 32'h0);
        axi_write(ADDR_CTRL, 32'h1);

        // Wait ~25 frames — should be deep into counting
        repeat(CYCLES_PER_FRAME * 25) @(posedge clk);

        axi_read(ADDR_STATUS, status);
        $display("[INFO] T3: status after ~25 frames: 0x%0h  (%s%s%s)",
                 status,
                 status[STATUS_IDLE_BIT] ? "IDLE " : "",
                 status[STATUS_BUSY_BIT] ? "BUSY " : "",
                 status[STATUS_DONE_BIT] ? "DONE"  : "");

        // Clear enable bit
        axi_write(ADDR_CTRL, 32'h0);  // enable=0

        // State machine should return to IDLE within one clock
        repeat(5) @(posedge clk);
        axi_read(ADDR_STATUS, status);
        assert (status[STATUS_IDLE_BIT] && !status[STATUS_BUSY_BIT] && !status[STATUS_DONE_BIT])
            else $error("[FAIL] T3: expected IDLE after disable, got status=0x%0h", status);
        $display("[PASS] T3: state returned to IDLE after disabling");

        // Cycle count should remain unchanged (stale) — just read it for info
        axi_read(ADDR_CYCLE_COUNT, cycle_count);
        $display("[INFO] T3: stale cycle_count=%0d (measurement was aborted)", cycle_count);

        // -----------------------------------------------------------------------
        $display("\n========================================================");
        $display("  All tests complete");
        $display("========================================================");
        $finish;
    end

endmodule
