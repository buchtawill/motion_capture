`timescale 1ns / 1ps
// =============================================================================
// tb_gfi.sv -- self-checking, PORT-LEVEL testbench for the GFI 3x3 filter.
//
// For every frame x {bypass, s0, s1, s2} it drives the input stream and,
// CONCURRENTLY (the DUT is serial: it emits output while it would otherwise
// accept input, so driver and monitor must run in parallel), captures the
// output stream and compares it beat-exact against the golden expected hex
// produced by gfi_model.gfi_filter (via gen_and_expect.py).
//
// Only DUT PORTS are referenced (no hierarchical dut.internal reads), so the
// TB is independent of the DUT's internal structure. Random backpressure is
// applied on m_ready (output) and, optionally, as gaps on s_valid (input).
// =============================================================================
module tb_gfi;

    // 1280x8 is the widest frame -> 320*8 = 2560 beats.
    localparam int MAX_BEATS = 2560;

    logic clk = 1'b0;
    always #2.5 clk = ~clk;             // 200 MHz

    logic        rst_n;
    logic        enable;
    logic [1:0]  strength;
    logic [15:0] hres, vres;

    logic        s_valid, s_ready;
    logic [31:0] s_data;
    logic        s_sof, s_eol;

    logic        m_valid, m_ready;
    logic [31:0] m_data;
    logic        m_sof, m_eol;

    gfi #(.MAX_WIDTH(2048)) dut (
        .clk(clk), .rst_n(rst_n),
        .enable(enable), .strength(strength), .hres(hres), .vres(vres),
        .s_valid(s_valid), .s_ready(s_ready), .s_data(s_data),
        .s_sof(s_sof), .s_eol(s_eol),
        .m_valid(m_valid), .m_ready(m_ready), .m_data(m_data),
        .m_sof(m_sof), .m_eol(m_eol)
    );

    logic [31:0] in_mem  [0:MAX_BEATS-1];
    logic [31:0] exp_mem [0:MAX_BEATS-1];
    logic [31:0] got_mem [0:MAX_BEATS-1];
    int          got_sof [0:MAX_BEATS-1];
    int          got_eol [0:MAX_BEATS-1];

    int pass_count = 0;
    int fail_count = 0;

    // ---- output backpressure (m_ready): ~7/8 ready when enabled ------------
    bit          bp_out = 1'b1;
    logic [15:0] mr_lfsr = 16'hACE1;
    always @(posedge clk) mr_lfsr <= {mr_lfsr[14:0], mr_lfsr[15]^mr_lfsr[13]^mr_lfsr[12]^mr_lfsr[10]};
    assign m_ready = bp_out ? (mr_lfsr[2:0] != 3'b000) : 1'b1;

    // ---- input gap control (s_valid gaps) ----------------------------------
    bit          bp_in = 1'b1;
    logic [15:0] sv_lfsr = 16'hBEEF;

    task automatic do_reset();
        rst_n   = 1'b0;
        s_valid = 1'b0;
        s_sof   = 1'b0;
        s_eol   = 1'b0;
        s_data  = 32'd0;
        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        @(posedge clk);
    endtask

    // Drive one frame (w x h) starting at in_mem[base]. Row-major, s_sof on the
    // first beat, s_eol on each row's last beat. Holds s_valid until s_ready.
    task automatic drive_frame(input int w, input int h, input int base);
        int hbeats = w / 4;
        int nbeats = (w / 4) * h;
        for (int b = 0; b < nbeats; b++) begin
            // optional random input gap (bubble) before presenting the beat
            if (bp_in) begin
                while (sv_lfsr[1:0] == 2'b00) begin
                    s_valid <= 1'b0;
                    @(posedge clk);
                    sv_lfsr <= {sv_lfsr[14:0], sv_lfsr[15]^sv_lfsr[13]^sv_lfsr[12]^sv_lfsr[10]};
                end
            end
            s_data  <= in_mem[base + b];
            s_sof   <= (b == 0);
            s_eol   <= (((b + 1) % hbeats) == 0);
            s_valid <= 1'b1;
            do @(posedge clk); while (!s_ready);   // accepted on the edge where s_ready=1
            sv_lfsr <= {sv_lfsr[14:0], sv_lfsr[15]^sv_lfsr[13]^sv_lfsr[12]^sv_lfsr[10]};
        end
        s_valid <= 1'b0;
        s_sof   <= 1'b0;
        s_eol   <= 1'b0;
    endtask

    // Capture nbeats output beats (m_valid && m_ready). ok=0 on stall timeout.
    task automatic capture_frame(input int nbeats, output bit ok);
        int got   = 0;
        int watch = 0;
        ok = 1'b1;
        while (got < nbeats) begin
            @(posedge clk);
            if (m_valid && m_ready) begin
                got_mem[got] = m_data;
                got_sof[got] = m_sof;
                got_eol[got] = m_eol;
                got++;
                watch = 0;
            end else begin
                watch++;
                if (watch > 200000) begin
                    ok = 1'b0;
                    $display("[%0t ns] [STALL] output stalled after %0d/%0d beats (s_ready=%0b m_valid=%0b m_ready=%0b)",
                             $time, got, nbeats, s_ready, m_valid, m_ready);
                    return;
                end
            end
        end
    endtask

    // Run one (frame,config) case end-to-end.
    task automatic run_case(input int idx, input int w, input int h,
                            input string tag, input logic en, input logic [1:0] str);
        int  hbeats = w / 4;
        int  nbeats = (w / 4) * h;
        bit  ok;
        int  first_bad;
        int  exp_sof, exp_eol;

        $readmemh($sformatf("frame_%04d.hex", idx), in_mem);
        $readmemh($sformatf("gfi_exp_%04d_%s.hex", idx, tag), exp_mem);

        do_reset();
        enable   = en;
        strength = str;
        hres     = w[15:0];
        vres     = h[15:0];

        fork
            drive_frame(w, h, 0);
            capture_frame(nbeats, ok);
        join

        if (!ok) begin
            fail_count++;
            $display("[%0t ns] [FAIL] frame %04d %0dx%0d %s -- output stalled (see above)",
                     $time, idx, w, h, tag);
            return;
        end

        // beat-exact data compare
        first_bad = -1;
        for (int b = 0; b < nbeats; b++) begin
            if (got_mem[b] !== exp_mem[b]) begin first_bad = b; break; end
        end
        if (first_bad != -1) begin
            fail_count++;
            $display("[%0t ns] [FAIL] frame %04d %0dx%0d %s -- data mismatch at beat %0d: exp %08x got %08x (row %0d col %0d)",
                     $time, idx, w, h, tag, first_bad, exp_mem[first_bad], got_mem[first_bad],
                     first_bad / hbeats, first_bad % hbeats);
            return;
        end

        // sideband (sof/eol) compare
        for (int b = 0; b < nbeats; b++) begin
            exp_sof = (b == 0) ? 1 : 0;
            exp_eol = (((b + 1) % hbeats) == 0) ? 1 : 0;
            if (got_sof[b] !== exp_sof || got_eol[b] !== exp_eol) begin
                fail_count++;
                $display("[%0t ns] [FAIL] frame %04d %0dx%0d %s -- sideband mismatch at beat %0d: exp sof=%0b eol=%0b got sof=%0b eol=%0b",
                         $time, idx, w, h, tag, b, exp_sof, exp_eol, got_sof[b], got_eol[b]);
                return;
            end
        end

        pass_count++;
        $display("[%0t ns] [PASS] frame %04d %0dx%0d %-6s (%0d beats)", $time, idx, w, h, tag, nbeats);
    endtask

    // Run all 4 configs for one frame.
    task automatic run_frame(input int idx, input int w, input int h);
        run_case(idx, w, h, "bypass", 1'b0, 2'd0);
        run_case(idx, w, h, "s0",     1'b1, 2'd0);
        run_case(idx, w, h, "s1",     1'b1, 2'd1);
        run_case(idx, w, h, "s2",     1'b1, 2'd2);
    endtask

    // Two frames back-to-back (no reset between) to prove per-frame reset.
    task automatic run_back_to_back(input int idx, input int w, input int h, input logic [1:0] str);
        int hbeats = w / 4;
        int nbeats = (w / 4) * h;
        bit ok1, ok2;
        int bad;
        $readmemh($sformatf("frame_%04d.hex", idx), in_mem);
        $readmemh($sformatf("gfi_exp_%04d_s%0d.hex", idx, str), exp_mem);
        do_reset();
        enable = 1'b1; strength = str; hres = w[15:0]; vres = h[15:0];
        fork
            begin drive_frame(w, h, 0); drive_frame(w, h, 0); end
            begin
                capture_frame(nbeats, ok1);
                for (int b = 0; b < nbeats; b++) begin got_mem[b] = 32'hDEAD; end // clear
                capture_frame(nbeats, ok2);
            end
        join
        bad = -1;
        if (!ok1 || !ok2) begin
            fail_count++;
            $display("[%0t ns] [FAIL] back-to-back frame %04d s%0d -- stalled (ok1=%0b ok2=%0b)", $time, idx, str, ok1, ok2);
            return;
        end
        for (int b = 0; b < nbeats; b++) if (got_mem[b] !== exp_mem[b]) begin bad = b; break; end
        if (bad != -1) begin
            fail_count++;
            $display("[%0t ns] [FAIL] back-to-back frame %04d s%0d -- 2nd frame mismatch at beat %0d: exp %08x got %08x",
                     $time, idx, str, bad, exp_mem[bad], got_mem[bad]);
            return;
        end
        pass_count++;
        $display("[%0t ns] [PASS] back-to-back frame %04d s%0d (per-frame reset ok)", $time, idx, str);
    endtask

    initial begin
        $display("=== tb_gfi: GFI 3x3 filter bit-exact self-check ===");
        do_reset();

        run_frame(0, 64, 48);
        run_frame(1, 64, 48);
        run_frame(2, 64, 48);
        run_frame(3, 64, 48);
        run_frame(4, 64, 48);
        run_frame(5, 1280, 8);

        run_back_to_back(1, 64, 48, 2'd1);

        $display("\n===== Results: %0d passed, %0d failed =====", pass_count, fail_count);
        if (fail_count == 0)
            $display("SUCCESS: ALL TESTS PASSED");
        else
            $display("FAILURE: %0d test(s) failed", fail_count);
        $finish;
    end

    initial begin
        #50ms;
        $display("[TIMEOUT] tb_gfi exceeded 50 ms");
        $fatal;
    end

endmodule
