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
//   Bin verification uses two paths:
//     backdoor  — direct read of dut_isp_histogram.hist_mem[] (fast, always available)
//     frontdoor — hardware ram_addr_i / ram_data_o interface (requires hist_en_i=0)
//
// Test plan
//   1. Post-reset / init  — all 256 hist_mem bins are zero after reset
//   2. Back-to-back beats — accumulation and write-hazard across multiple beats
//   3. Distinct pixels    — beat {0x04, 0x03, 0x02, 0x01}
//                           → hist_mem[0x01..0x04] each equal 1
//   4. Accumulation       — repeat the same beat from test 3
//                           → hist_mem[0x01..0x04] each equal 2
//   5. RAM scrub          — strobe ram_scrub_i, wait for hist_rdy_o,
//                           verify all 256 bins are zero
//   6. Full-image zeros   — stream 1280×800 zero-valued pixels
//                           → hist_mem[0x00] == 1,024,000
//   7. RAM read-port      — verify ram_data_o / ram_data_o_vld interface:
//                           vld gating, frontdoor read after stream, after scrub
//   8. Randomized bins    — 20 iterations × RAND_TEST_PIXELS random pixels;
//                           each iteration checked via backdoor and frontdoor

`timescale 1ns / 1ps

module tb_isp_histogram;

    // =========================================================================
    // Parameters  (must match DUT instantiation below)
    // =========================================================================
    localparam int STREAM_WIDTH      = 32;
    localparam int RAM_WIDTH         = 20;
    localparam int RAND_TEST_PIXELS  = 128; // pixels per randomized iteration

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

    // =========================================================================
    // Randomized-test scratch space (module-scope to avoid SV lifetime issues)
    // =========================================================================
    int          rand_expected[256];
    logic [7:0]  rand_pix[RAND_TEST_PIXELS];
    logic [31:0] rand_beat_word;

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

    // -------------------------------------------------------------------------
    // check_all_bins — compare every hist_mem bin against a software reference.
    //   backdoor=1  : sweep via direct hist_mem[] access
    //   frontdoor=1 : sweep via ram_addr_i / ram_data_o (requires hist_en_i=0)
    //   Both default to 1; pass 0 to skip either path.
    // -------------------------------------------------------------------------
    task automatic check_all_bins(
        input string label,
        input int    expected[256],
        input bit    backdoor  = 1,
        input bit    frontdoor = 1
    );
        logic                 ok;
        logic [RAM_WIDTH-1:0] actual;

        if (backdoor) begin
            ok = 1'b1;
            for (int i = 0; i < 256; i++) begin
                actual = dut_isp_histogram.hist_mem[i];
                if (actual !== RAM_WIDTH'(expected[i])) begin
                    $display("[%0t ns] [FAIL] %s (backdoor): hist_mem[0x%02h] — expected %0d, got %0d",
                             $time, label, i[7:0], expected[i], actual);
                    fail_count++;
                    ok = 1'b0;
                end
            end
            if (ok)
                $display("[%0t ns] [PASS] %s (backdoor): all 256 bins match", $time, label);
        end

        if (frontdoor) begin
            ok = 1'b1;
            for (int i = 0; i < 256; i++) begin
                read_bin_ram(i[7:0], actual);
                if (actual !== RAM_WIDTH'(expected[i])) begin
                    $display("[%0t ns] [FAIL] %s (frontdoor): ram_data_o[0x%02h] — expected %0d, got %0d",
                             $time, label, i[7:0], expected[i], actual);
                    fail_count++;
                    ok = 1'b0;
                end
            end
            if (ok)
                $display("[%0t ns] [PASS] %s (frontdoor): all 256 bins match", $time, label);
        end
    endtask

    // =========================================================================
    // Helper tasks
    // =========================================================================

    // send_beat — present one 32-bit pixel word on the snooped AXI-S bus for
    //             exactly one clock cycle with both vld and rdy asserted,
    //             simulating a live upstream/downstream handshake.
    task automatic send_beat(input logic [STREAM_WIDTH-1:0] beat, int info=1);
        if(info) $display("[%0t ns] Sending beat 0x%08x", $time, beat);
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

    // wait_scrub_done — block until hist_rdy_o goes high, or fail after
    //                   timeout_cycles if the scrub FSM never completes.
    task automatic wait_scrub_done(input int timeout_cycles);
        fork
            begin : t_wait_rdy
                @(posedge hist_rdy_o);
            end
            begin : t_scrub_timeout
                repeat(timeout_cycles) @(posedge clk);
                $display("[%0t ns] [FAIL] Timed out waiting for hist_rdy_o after scrub", $time);
                fail_count++;
                disable t_wait_rdy;
            end
        join_any
        disable fork;
    endtask

    // read_bin_ram — drive ram_addr_i, wait one clock, return ram_data_o.
    //               ram_data_o is combinational from hist_mem[ram_addr_i], so
    //               data is valid immediately after the address is applied.
    //               The single clock wait lets any in-flight write commit first.
    //               Requires hist_en_i=0 on entry (caller's responsibility).
    task automatic read_bin_ram(input  logic [7:0]          addr,
                                 output logic [RAM_WIDTH-1:0] data);
        ram_addr_i = addr;
        @(posedge clk); 
        #1;
        data = ram_data_o;
    endtask

    // check_bin_ram — frontdoor equivalent of check_bin.
    task automatic check_bin_ram(input string               label,
                                  input logic [7:0]          bin,
                                  input logic [RAM_WIDTH-1:0] expected);
        logic [RAM_WIDTH-1:0] actual;
        read_bin_ram(bin, actual);
        if (actual === expected) begin
            $display("[%0t ns] [PASS] %s: ram_data_o[0x%02h] = %0d", $time, label, bin, actual);
            pass_count++;
        end else begin
            $display("[%0t ns] [FAIL] %s: ram_data_o[0x%02h] — expected %0d, got %0d",
                     $time, label, bin, expected, actual);
            fail_count++;
        end
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
        // Test 2: Back-to-back beats with accumulation and write-hazard.
        //   Round A: {0x12345678, 0x00343400}
        //     → 0x12=1, 0x34=3, 0x56=1, 0x78=1, 0x00=2
        //   Round B: {0x12345678, 0x00003400}
        //     → 0x00 accumulates to 5
        //   Round C: {0x55555555} — all four bytes identical
        //     → 0x55=4, exercises the same-bin write-hazard path
        // ----------------------------------------------------------------
        $display("\n[%0t ns] --- Test 2: back-to-back beats, accumulation, hazard ---", $time);
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

        hist_en_i = 1'b1;
        send_beat(32'h12345678);
        send_beat(32'h00003400);
        wait_drain();
        wait_drain();
        hist_en_i = 1'b0;
        check_bin("00", 8'h00, RAM_WIDTH'(5));

        repeat(2)@(posedge clk);
        hist_en_i = 1'b1;
        send_beat(32'h55555555);
        wait_drain();
        check_bin("55", 8'h55, RAM_WIDTH'(4));
        hist_en_i = 1'b0;

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
        // Test 5: RAM scrub
        //   With hist_en_i low, strobe ram_scrub_i for one cycle, then
        //   block until hist_rdy_o goes high indicating the scrub FSM has
        //   finished zeroing all 256 bins.  Verify every bin is zero.
        // ----------------------------------------------------------------
        $display("\n[%0t ns] --- Test 5: RAM scrub ---", $time);
        hist_en_i   = 1'b0;
        @(posedge clk);
        $display("[%0t ns] [INFO] Asserting ram_scrub_i", $time);
        ram_scrub_i = 1'b1;
        @(posedge clk);
        ram_scrub_i = 1'b0;
        $display("[%0t ns] [INFO] ram_scrub_i deasserted, waiting for hist_rdy_o", $time);
        wait_scrub_done(280);
        repeat(260)@(posedge clk);
        check_all_zero("post-scrub");

        repeat(20)@(posedge clk);
        $display("\n[%0t ns] --- Test 6: Sending all zeros for one full image ---", $time);
        hist_en_i   = 1'b1;
        // Stream of all zeros
        for (int i = 0; i < (1280*800/4); i++)begin
            send_beat(32'h00000000, 0);
            repeat(3)@(posedge clk);
        end

        repeat(8)@(posedge clk);

        check_bin("All zero", 8'h0, 1280*800);

        // ----------------------------------------------------------------
        // Test 7: RAM read-port sanity
        //   7a. ram_data_o_vld gating — high only when hist_en_i=0 and IDLE
        //   7b. Frontdoor read after known stream — bins 1-4 via ram interface
        //   7c. Frontdoor read after scrub — all bins via ram interface = 0
        // ----------------------------------------------------------------
        $display("\n[%0t ns] --- Test 7: RAM read-port sanity ---", $time);

        // 7a: vld must be low while hist_en_i=1, high when hist_en_i=0
        $display("[%0t ns] [INFO] Test 7a: ram_data_o_vld gating", $time);
        hist_en_i = 1'b1;
        @(posedge clk);
        check("7a: vld low  (hist_en_i=1)", ram_data_o_vld, 1'b0);
        hist_en_i = 1'b0;
        @(posedge clk);
        check("7a: vld high (hist_en_i=0)", ram_data_o_vld, 1'b1);

        // 7b: stream a known beat, read specific bins via frontdoor.
        //     After Test 6, bins 0x01-0x04 are 0 (only 0x00 was incremented).
        $display("[%0t ns] [INFO] Test 7b: frontdoor read after known stream", $time);
        hist_en_i = 1'b1;
        send_beat(32'h04030201);
        wait_drain();
        hist_en_i = 1'b0;
        check_bin_ram("7b pixel 0x01", 8'h01, RAM_WIDTH'(1));
        check_bin_ram("7b pixel 0x02", 8'h02, RAM_WIDTH'(1));
        check_bin_ram("7b pixel 0x03", 8'h03, RAM_WIDTH'(1));
        check_bin_ram("7b pixel 0x04", 8'h04, RAM_WIDTH'(1));

        // 7c: scrub then sweep all 256 bins via frontdoor only.
        $display("[%0t ns] [INFO] Test 7c: frontdoor read after scrub", $time);
        @(posedge clk);
        ram_scrub_i = 1'b1;
        @(posedge clk);
        ram_scrub_i = 1'b0;
        wait_scrub_done(280);
        for (int b = 0; b < 256; b++) rand_expected[b] = 0;
        check_all_bins("post-scrub 7c", rand_expected, /*backdoor=*/0, /*frontdoor=*/1);

        // ----------------------------------------------------------------
        // Test 8: Randomized bins — 20 iterations × RAND_TEST_PIXELS pixels
        //   Each iteration generates random byte-valued pixels, builds a
        //   software reference histogram, streams through the DUT, then
        //   checks every bin via both backdoor and frontdoor paths.
        //   Iter 0 clears via ram_scrub_i; iters 1-19 zero hist_mem[] directly.
        // ----------------------------------------------------------------
        $display("\n[%0t ns] --- Test 8: randomized bins (20 iters x %0d pixels) ---", $time, RAND_TEST_PIXELS);

        // Test 7c already scrubbed the RAM via the hardware FSM.
        for (int iter = 0; iter < 100; iter++) begin
            // Build software reference model
            for (int b = 0; b < 256; b++) rand_expected[b] = 0;
            for (int p = 0; p < RAND_TEST_PIXELS; p++) begin
                rand_pix[p] = $urandom_range(0, 255);
                rand_expected[rand_pix[p]]++;
            end

            // Stream RAND_TEST_PIXELS/4 beats; 3-cycle gap keeps the FIFO from filling
            hist_en_i = 1'b1;
            for (int p = 0; p < RAND_TEST_PIXELS; p += 4) begin
                rand_beat_word = {rand_pix[p+3], rand_pix[p+2], rand_pix[p+1], rand_pix[p]};
                send_beat(rand_beat_word, 0);
                repeat($urandom_range(0, 5)) @(posedge clk);
            end
            wait_drain();
            wait_drain();
            hist_en_i = 1'b0;

            // Verify every bin against the reference
            check_all_bins($sformatf("rand iter %0d", iter), rand_expected);

            // Clear for next iteration
            if (iter == 0) begin
                $display("[%0t ns] [INFO] iter 0: clearing via ram_scrub_i FSM", $time);
                @(posedge clk);
                ram_scrub_i = 1'b1;
                @(posedge clk);
                ram_scrub_i = 1'b0;
                wait_scrub_done(280);
            end else begin
                for (int b = 0; b < 256; b++)
                    dut_isp_histogram.hist_mem[b] = '0;
            end
        end

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
        #10ms;
        $display("[%0t ns] [TIMEOUT] Simulation exceeded 10 ms — possible hang", $time);
        $finish;
    end

    // =========================================================================
    // Signal monitors
    // =========================================================================
    // initial begin : monitor_hist_rdy
    //     forever @(hist_rdy_o)
    //         $display("[%0t ns] [MON] hist_rdy_o -> %0b", $time, hist_rdy_o);
    // end

    // initial begin : monitor_ram_data_o_vld
    //     forever @(ram_data_o_vld)
    //         $display("[%0t ns] [MON] ram_data_o_vld -> %0b", $time, ram_data_o_vld);
    // end

endmodule
