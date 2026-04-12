// tb_isp_histogram.sv
// Testbench for isp_histogram (256-bin, 4-pixel-wide AXI-S snoop)
//
// Topology
//   tb ──pix AXI-S snoop──► isp_histogram (dut_isp_histogram)
//
//   The histogram module snoops the pixel bus (pix_data_vld_i & pix_data_rdy_i)
//   rather than consuming it, so the testbench asserts both signals together to
//   simulate a live upstream/downstream handshake.
//
//   Phase 1 (this file): histogram correctness is verified by reading
//   dut_isp_histogram.hist_mem[] directly instead of going through the
//   ram_addr_i / ram_data_o read-port interface.  The read-port path will be
//   exercised in a later phase once the scrub FSM is also complete.
//
// Test plan
//   1. Post-reset / init  — all 256 hist_mem bins are zero after reset
//   2. Repeated pixel     — 4× pixel 0x55 in one beat
//                           → hist_mem[0x55] must equal 4
//   3. Distinct pixels    — beat {0x04, 0x03, 0x02, 0x01}
//                           → hist_mem[0x01..0x04] each equal 1
//   4. Accumulation       — repeat the same beat from test 3
//                           → hist_mem[0x01..0x04] each equal 2
//   5. Scrub placeholder  — strobe ram_scrub_i with hist_en_i low
//                           (scrub FSM is TBD; test validates signal plumbing
//                            and will be completed alongside the FSM)

`timescale 1ns / 1ps

module tb_isp_histogram;

    // =========================================================================
    // Parameters  (must match DUT instantiation below)
    // =========================================================================
    localparam int STREAM_WIDTH = 32;
    localparam int RAM_WIDTH    = 20;

    // Generous drain budget: FIFO latency + 4 FSM cycles + registered RAM read
    localparam int DRAIN_CYCLES = 20;

    // =========================================================================
    // Clock / reset
    // =========================================================================
    localparam real CLK_HALF_NS = 2.5; // 200 MHz

    logic clk   = 1'b0;
    logic rst_n = 1'b0;

    always #CLK_HALF_NS clk = ~clk;

    // =========================================================================
    // DUT signals
    // =========================================================================
    logic                    hist_en_i    = 1'b0;
    logic                    ram_scrub_i  = 1'b0;
    logic                    hist_rdy_o;
    logic                    err_o;

    logic [7:0]              ram_addr_i   = 8'h00;
    logic [RAM_WIDTH-1:0]    ram_data_o;
    logic                    ram_data_o_vld;

    logic [STREAM_WIDTH-1:0] pix_data_i     = '0;
    logic                    pix_data_vld_i = 1'b0;
    logic                    pix_data_rdy_i = 1'b0;

    // =========================================================================
    // DUT instantiation
    // =========================================================================
    isp_histogram #(
        .STREAM_WIDTH (STREAM_WIDTH),
        .RAM_WIDTH    (RAM_WIDTH)
    ) dut_isp_histogram (
        .clk_i          (clk),
        .rst_n          (rst_n),
        .hist_en_i      (hist_en_i),
        .ram_scrub_i    (ram_scrub_i),
        .hist_rdy_o     (hist_rdy_o),
        .err_o          (err_o),

        .ram_addr_i     (ram_addr_i),
        .ram_data_o     (ram_data_o),
        .ram_data_o_vld (ram_data_o_vld),

        .pix_data_i     (pix_data_i),
        .pix_data_vld_i (pix_data_vld_i),
        .pix_data_rdy_i (pix_data_rdy_i)
    );

    // =========================================================================
    // Pass / fail counters
    // =========================================================================
    int pass_count = 0;
    int fail_count = 0;

    // -------------------------------------------------------------------------
    // check — single-bit assertion helper
    // -------------------------------------------------------------------------
    task automatic check(input string label,
                         input logic  got,
                         input logic  exp);
        if (got === exp) begin
            $display("[%0t ns] [PASS] %s", $time, label);
            pass_count++;
        end else begin
            $display("[%0t ns] [FAIL] %s — expected %0b, got %0b", $time, label, exp, got);
            fail_count++;
        end
    endtask

    // -------------------------------------------------------------------------
    // check_bin — direct hist_mem read, no read-port latency
    // -------------------------------------------------------------------------
    task automatic check_bin(input string               label,
                              input logic [7:0]          bin,
                              input logic [RAM_WIDTH-1:0] expected);
        logic [RAM_WIDTH-1:0] actual;
        actual = dut_isp_histogram.hist_mem[bin];
        if (actual === expected) begin
            $display("[%0t ns] [PASS] %s: hist_mem[0x%02h] = %0d", $time, label, bin, actual);
            pass_count++;
        end else begin
            $display("[%0t ns] [FAIL] %s: hist_mem[0x%02h] — expected %0d, got %0d",
                     $time, label, bin, expected, actual);
            fail_count++;
        end
    endtask

    // -------------------------------------------------------------------------
    // check_all_zero — scan all 256 bins; report as a single pass/fail
    // -------------------------------------------------------------------------
    task automatic check_all_zero(input string label);
        logic [RAM_WIDTH-1:0] val;
        logic ok;
        ok = 1'b1;
        for (int i = 0; i < 256; i++) begin
            val = dut_isp_histogram.hist_mem[i];
            if (val !== '0) begin
                $display("[%0t ns] [FAIL] %s: hist_mem[0x%02h] = %0d (expected 0)", $time, label, i, val);
                fail_count++;
                ok = 1'b0;
            end
        end
        if (ok) begin
            $display("[%0t ns] [PASS] %s: all 256 bins are zero", $time, label);
            pass_count++;
        end
    endtask

    // =========================================================================
    // Helper tasks
    // =========================================================================

    // send_beat — present one 32-bit pixel word on the snooped AXI-S bus for
    //             exactly one clock cycle with both vld and rdy asserted,
    //             simulating a live upstream/downstream handshake.
    task automatic send_beat(input logic [STREAM_WIDTH-1:0] beat);
        $display("[%0t ns] Sending beat 0x%08x", $time, beat);
        @(posedge clk);
        pix_data_i     = beat;
        pix_data_vld_i = 1'b1;
        pix_data_rdy_i = 1'b1;
        @(posedge clk);
        pix_data_vld_i = 1'b0;
        pix_data_rdy_i = 1'b0;
        pix_data_i     = '0;
    endtask

    // wait_drain — stall for DRAIN_CYCLES so the FSM can fully consume all
    //              four pixels and commit the incremented counts to hist_mem.
    task automatic wait_drain();
        repeat(DRAIN_CYCLES) @(posedge clk);
    endtask

    // =========================================================================
    // Main test sequence
    // =========================================================================
    initial begin : test_seq
        $timeformat(-9, 0, "", 1); // %t displays in nanoseconds, no auto-suffix
        $display("==============================================");
        $display("  tb_isp_histogram  STREAM_WIDTH=%0d  RAM_WIDTH=%0d",
                 STREAM_WIDTH, RAM_WIDTH);
        $display("==============================================");

        // ----------------------------------------------------------------
        // Reset
        // ----------------------------------------------------------------
        rst_n     = 1'b0;
        hist_en_i = 1'b0;
        repeat(8) @(posedge clk);
        rst_n = 1'b1;
        repeat(4) @(posedge clk);
        $display("[%0t ns] [INFO] Reset released", $time);

        // ----------------------------------------------------------------
        // Test 1: Post-reset — all 256 bins must be zero
        //   Relies on the initial block in isp_histogram that pre-zeros
        //   hist_mem at simulation time 0.
        // ----------------------------------------------------------------
        $display("\n[%0t ns] --- Test 1: post-reset, all bins zero ---", $time);
        @(posedge clk); 
        check_all_zero("post-reset");

        // ----------------------------------------------------------------
        // Test 2: Repeated pixel value
        //   Beat word: 32'h5555_5555  (all four bytes == 0x55)
        //   All four pixels are identical, exercising the write-hazard path.
        //   Expected: hist_mem[0x55] == 4
        // ----------------------------------------------------------------
        $display("\n[%0t ns] --- Test 2: 2 beats back to back ---", $time);
        hist_en_i = 1'b1;
        send_beat(32'h12345678);
        send_beat(32'h00343400);
        wait_drain();
        wait_drain();
        hist_en_i = 1'b0;
        check_bin("12", 8'h12, RAM_WIDTH'(1));
        check_bin("34", 8'h34, RAM_WIDTH'(3));
        check_bin("56", 8'h56, RAM_WIDTH'(1));
        check_bin("78", 8'h78, RAM_WIDTH'(1));
        check_bin("00", 8'h00, RAM_WIDTH'(2));

        $display("\n[%0t ns] --- Test 2: 2 beats back to back ---", $time);
        hist_en_i = 1'b1;
        send_beat(32'h12345678);
        send_beat(32'h00003400);
        wait_drain();
        wait_drain();
        hist_en_i = 1'b0;
        check_bin("00", 8'h00, RAM_WIDTH'(5));

        // ----------------------------------------------------------------
        // Test 3: Four distinct pixel values
        //   Beat word: 32'h0403_0201
        //     bytes[7:0]   = 0x01  (pixel 0)
        //     bytes[15:8]  = 0x02  (pixel 1)
        //     bytes[23:16] = 0x03  (pixel 2)
        //     bytes[31:24] = 0x04  (pixel 3)
        //   Expected: hist_mem[0x01..0x04] each == 1; all others unchanged
        // ----------------------------------------------------------------
        $display("\n[%0t ns] --- Test 3: distinct pixels {0x04, 0x03, 0x02, 0x01} ---", $time);
        hist_en_i = 1'b1;
        send_beat(32'h04030201);
        wait_drain();
        hist_en_i = 1'b0;
        check_bin("pixel 0x01", 8'h01, RAM_WIDTH'(1));
        check_bin("pixel 0x02", 8'h02, RAM_WIDTH'(1));
        check_bin("pixel 0x03", 8'h03, RAM_WIDTH'(1));
        check_bin("pixel 0x04", 8'h04, RAM_WIDTH'(1));

        // ----------------------------------------------------------------
        // Test 4: Accumulation
        //   Send the identical beat a second time; counts must double.
        //   Expected: hist_mem[0x01..0x04] each == 2
        // ----------------------------------------------------------------
        $display("\n[%0t ns] --- Test 4: accumulation (repeat beat from test 3) ---", $time);
        hist_en_i = 1'b1;
        send_beat(32'h04030201);
        wait_drain();
        hist_en_i = 1'b0;
        check_bin("pixel 0x01 x2", 8'h01, RAM_WIDTH'(2));
        check_bin("pixel 0x02 x2", 8'h02, RAM_WIDTH'(2));
        check_bin("pixel 0x03 x2", 8'h03, RAM_WIDTH'(2));
        check_bin("pixel 0x04 x2", 8'h04, RAM_WIDTH'(2));

        // ----------------------------------------------------------------
        // Test 5: Scrub placeholder
        //   Assert ram_scrub_i for one cycle while hist_en_i is low.
        //   The scrub FSM that iterates over all 256 bins has not yet been
        //   implemented; this test confirms the signal routing is in place
        //   and that a strobe does not hang or corrupt the design.
        //
        //   TODO: once the scrub FSM is complete, replace the INFO line
        //         below with:  check_all_zero("post-scrub");
        // ----------------------------------------------------------------
        $display("\n[%0t ns] --- Test 5: scrub placeholder (scrub FSM TBD) ---", $time);
        hist_en_i   = 1'b0;
        @(posedge clk);
        ram_scrub_i = 1'b1;
        @(posedge clk);
        ram_scrub_i = 1'b0;
        repeat(10) @(posedge clk);
        $display("[%0t ns] [INFO] ram_scrub_i strobed — scrub FSM not yet implemented, skipping bin check", $time);
        // TODO: check_all_zero("post-scrub");

        // ----------------------------------------------------------------
        // Summary
        // ----------------------------------------------------------------
        $display("\n==============================================");
        $display("[%0t ns]   Results: %0d passed, %0d failed", $time, pass_count, fail_count);
        $display("==============================================");
        if (fail_count == 0)
            $display("[%0t ns]   SUCCESS: ALL TESTS PASSED", $time);
        else
            $display("[%0t ns]   ERROR: FAILURES DETECTED", $time);

        $finish;
    end

    // =========================================================================
    // Timeout watchdog — abort if the simulation hangs inside a wait loop
    // =========================================================================
    initial begin : timeout_watchdog
        #1ms;
        $display("[%0t ns] [TIMEOUT] Simulation exceeded 1 ms — possible hang", $time);
        $finish;
    end

endmodule
