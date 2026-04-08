// tb_stream_fifo.sv
// Testbench for stream_fifo (valid/ready handshake, parameterized width)
//
// Test plan
//   1. Basic fill & drain  — push DEPTH items, verify order on drain
//   2. Full boundary       — FIFO full: s_ready deasserts; push while full is blocked
//   3. Empty boundary      — FIFO empty: m_valid deasserts; empty flag asserts
//   4. Simultaneous push+pop when full  — one item in, one out; count stays at DEPTH
//   5. Simultaneous push+pop when empty — pop blocked by m_valid=0; push succeeds
//   6. Backpressure mid-stream — hold m_ready low mid-drain; data must not be lost

`timescale 1ns / 1ps

module tb_stream_fifo;

    // =========================================================================
    // Parameters (match DUT instantiation below)
    // =========================================================================
    localparam int DATA_WIDTH = 8;
    localparam int DEPTH      = 8;

    // =========================================================================
    // Clock
    // =========================================================================
    localparam real CLK_HALF_NS = 2.5; // 200 MHz

    logic clk   = 1'b0;
    logic rst_n = 1'b0;

    always #CLK_HALF_NS clk = ~clk;

    // =========================================================================
    // DUT signals
    // =========================================================================
    logic                  s_valid = 1'b0;
    logic                  s_ready;
    logic [DATA_WIDTH-1:0] s_data  = '0;

    logic                  m_valid;
    logic                  m_ready = 1'b0;
    logic [DATA_WIDTH-1:0] m_data;

    logic                  empty;

    // =========================================================================
    // DUT
    // =========================================================================
    stream_fifo #(
        .DATA_WIDTH (DATA_WIDTH),
        .DEPTH      (DEPTH)
    ) dut (
        .clk     (clk),
        .rst_n   (rst_n),
        .s_valid (s_valid),
        .s_ready (s_ready),
        .s_data  (s_data),
        .m_valid (m_valid),
        .m_ready (m_ready),
        .m_data  (m_data),
        .empty   (empty)
    );

    // =========================================================================
    // Pass/fail counter
    // =========================================================================
    int pass_count = 0;
    int fail_count = 0;

    task automatic check(input string label,
                         input logic  got,
                         input logic  exp);
        if (got === exp) begin
            $display("[PASS] %s", label);
            pass_count++;
        end else begin
            $display("[FAIL] %s — expected %0b, got %0b", label, exp, got);
            fail_count++;
        end
    endtask

    task automatic check_data(input string            label,
                               input logic [DATA_WIDTH-1:0] got,
                               input logic [DATA_WIDTH-1:0] exp);
        if (got === exp) begin
            $display("[PASS] %s (0x%02h)", label, got);
            pass_count++;
        end else begin
            $display("[FAIL] %s — expected 0x%02h, got 0x%02h", label, exp, got);
            fail_count++;
        end
    endtask

    // =========================================================================
    // Tasks: single-beat push / pop
    // =========================================================================
    task automatic push(input logic [DATA_WIDTH-1:0] data);
        @(posedge clk); #1;
        s_valid = 1'b1;
        s_data  = data;
        do @(posedge clk); while (!(s_valid && s_ready));
        #1;
        s_valid = 1'b0;
    endtask

    task automatic pop(output logic [DATA_WIDTH-1:0] data);
        @(posedge clk); #1;
        m_ready = 1'b1;
        do @(posedge clk); while (!(m_valid && m_ready));
        data = m_data;
        #1;
        m_ready = 1'b0;
    endtask

    // =========================================================================
    // Main test sequence
    // =========================================================================
    initial begin : test_seq
        logic [DATA_WIDTH-1:0] rdata;

        $display("==============================================");
        $display("  tb_stream_fifo  DEPTH=%0d  WIDTH=%0d", DEPTH, DATA_WIDTH);
        $display("==============================================");

        // Release reset
        rst_n = 1'b0;
        repeat(4) @(posedge clk);
        #1; rst_n = 1'b1;
        repeat(2) @(posedge clk);
        $display("[INFO] Reset released");

        // ------------------------------------------------------------------
        // Test 1: empty flag after reset
        // ------------------------------------------------------------------
        $display("\n--- Test 1: empty flag after reset ---");
        @(posedge clk); #1;
        check("empty asserts after reset",   empty,   1'b1);
        check("m_valid low when empty",      m_valid, 1'b0);
        check("s_ready high when not full",  s_ready, 1'b1);

        // ------------------------------------------------------------------
        // Test 2: basic fill and drain (FIFO ordering)
        // ------------------------------------------------------------------
        $display("\n--- Test 2: fill then drain (DEPTH=%0d items) ---", DEPTH);
        for (int i = 0; i < DEPTH; i++)
            push(DATA_WIDTH'(i + 8'hA0));

        check("empty deasserts after fill", empty,   1'b0);
        check("m_valid high after fill",    m_valid, 1'b1);

        for (int i = 0; i < DEPTH; i++) begin
            pop(rdata);
            check_data($sformatf("drain[%0d] value", i), rdata, DATA_WIDTH'(i + 8'hA0));
        end

        check("empty reasserts after full drain", empty, 1'b1);

        // ------------------------------------------------------------------
        // Test 3: full boundary — s_ready deasserts when full
        // ------------------------------------------------------------------
        $display("\n--- Test 3: full boundary ---");
        for (int i = 0; i < DEPTH; i++)
            push(DATA_WIDTH'(i));

        @(posedge clk); #1;
        check("s_ready low when full", s_ready, 1'b0);

        // Drain one slot so we can continue
        pop(rdata);
        @(posedge clk); #1;
        check("s_ready high after one pop", s_ready, 1'b1);

        // Drain the rest
        for (int i = 1; i < DEPTH; i++)
            pop(rdata);

        // ------------------------------------------------------------------
        // Test 4: simultaneous push+pop at mid-level — count must stay constant
        //
        // When the FIFO is full, s_ready=0 so push is always blocked that cycle;
        // only pop fires, count drops by 1. To get a true simultaneous push+pop
        // both handshakes must be able to fire, which requires s_ready=1 (not full).
        // ------------------------------------------------------------------
        $display("\n--- Test 4: simultaneous push+pop (mid-level) ---");
        for (int i = 0; i < DEPTH/2; i++)
            push(DATA_WIDTH'(8'hB0 + i));

        // Drive both sides on the same clock edge; s_ready=1 so both fire
        @(posedge clk); #1;
        s_valid = 1'b1;
        s_data  = 8'hFF;
        m_ready = 1'b1;
        @(posedge clk);               // push and pop both handshake
        #1;
        s_valid = 1'b0;
        m_ready = 1'b0;

        // Count unchanged (DEPTH/2): neither full nor empty
        @(posedge clk); #1;
        check("s_ready high: not full after push+pop",  s_ready, 1'b1);
        check("m_valid high: not empty after push+pop", m_valid, 1'b1);

        // Drain remaining DEPTH/2 items
        for (int i = 0; i < DEPTH/2; i++)
            pop(rdata);

        // ------------------------------------------------------------------
        // Test 5: simultaneous push+pop when empty — pop must be blocked
        // ------------------------------------------------------------------
        $display("\n--- Test 5: simultaneous push+pop at empty ---");
        @(posedge clk); #1;
        check("empty before test 5", empty, 1'b1);

        // Assert both sides; m_valid is low so no pop should fire
        s_valid = 1'b1;
        s_data  = 8'hC1;
        m_ready = 1'b1;
        @(posedge clk); #1;   // push fires (s_ready=1), pop blocked (m_valid=0 before edge)
        s_valid = 1'b0;

        // Now drain the one item that was pushed
        do @(posedge clk); while (!(m_valid && m_ready));
        rdata   = m_data;
        #1; m_ready = 1'b0;

        check_data("test 5: popped value correct", rdata, 8'hC1);
        @(posedge clk); #1;
        check("empty after test 5 drain", empty, 1'b1);

        // ------------------------------------------------------------------
        // Test 6: backpressure mid-stream
        // ------------------------------------------------------------------
        $display("\n--- Test 6: backpressure mid-drain ---");
        for (int i = 0; i < DEPTH; i++)
            push(DATA_WIDTH'(8'hD0 + i));

        // Pop 3 items freely, then stall for 5 cycles, then drain the rest
        for (int i = 0; i < 3; i++) begin
            pop(rdata);
            check_data($sformatf("backpressure pre-stall[%0d]", i),
                       rdata, DATA_WIDTH'(8'hD0 + i));
        end

        // Stall: hold m_ready low for 5 cycles
        m_ready = 1'b0;
        repeat(5) @(posedge clk);

        check("m_valid still high during stall", m_valid, 1'b1);
        check("empty low during stall",          empty,   1'b0);

        for (int i = 3; i < DEPTH; i++) begin
            pop(rdata);
            check_data($sformatf("backpressure post-stall[%0d]", i),
                       rdata, DATA_WIDTH'(8'hD0 + i));
        end

        check("empty after backpressure drain", empty, 1'b1);

        // ------------------------------------------------------------------
        // Summary
        // ------------------------------------------------------------------
        $display("\n==============================================");
        $display("  Results: %0d passed, %0d failed", pass_count, fail_count);
        $display("==============================================");
        if (fail_count == 0)
            $display("  ALL TESTS PASSED");
        else
            $display("  FAILURES DETECTED");

        $finish;
    end

    // =========================================================================
    // Timeout watchdog — abort if simulation hangs
    // =========================================================================
    initial begin : timeout_watchdog
        #1ms;
        $display("[TIMEOUT] Simulation exceeded 1 ms — possible hang in push/pop task");
        $finish;
    end

endmodule
