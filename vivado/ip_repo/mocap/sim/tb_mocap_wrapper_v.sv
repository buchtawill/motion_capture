// tb_mocap_wrapper_v.sv
// Integration testbench for the *Verilog* IP-Integrator shell `mocap_wrapper`
// (mocap_wrapper.v), NOT the SystemVerilog core mocap_top directly. Its job is
// to prove the wrapper wires every port through 1:1 and that every hardware
// block inside is reachable and functional through the wrapper's real port list
// -- i.e. what IP Integrator will instantiate on the KV260.
//
// Frontdoor only (AXI4-Lite + AXI-Stream + the interrupt pin): the wrapper hides
// the SV hierarchy, so unlike tb_mocap_wrapper (which backdoors hist_mem[] etc.)
// this suite exercises the block purely through its external interface. Datapath
// bit-exactness vs the Python goldens is already proven by tb_mocap_wrapper; here
// we use self-consistent invariants (histogram mass, pixel-sum identity) plus the
// one known golden (frame_0002 has 10 blobs) so no model files are needed.
//
// Two targeted tests, each touching every block:
//   W1 -- one full frame end to end: regblock, frame-control FSM, BOTH
//         isp_histogram instances (+ pixel-sum), the blob core (run_extractor/
//         row_merger/blob_table) + copy-FSM, the in/out stream FIFOs
//         (passthrough), the ownership double buffer + bank flip, and the
//         frame_done_irq_o pin. A second frame proves ping-pong.
//   W2 -- the free-running 64-bit cycle counter + CTRL.CYCLE_SNAPSHOT.

`timescale 1ns / 1ps

`include "mocap_regs_defines.svh"

module tb_mocap_wrapper_v;

    localparam real CLK_HALF_NS     = 2.5; // 200 MHz
    localparam int  MAX_BLOBS       = 128;
    localparam int  MAX_FRAME_WORDS = 1280 * 800 / 4;
    localparam int  IRQ_TIMEOUT     = 2_000_000;

    logic clk = 1'b0, aresetn = 1'b0;
    always #CLK_HALF_NS clk = ~clk;

    // ---- AXI4-Lite (7-bit addr) ----
    logic [6:0]  s_axi_awaddr = '0;  logic [2:0] s_axi_awprot = '0;
    logic        s_axi_awvalid = 0;  logic       s_axi_awready;
    logic [31:0] s_axi_wdata  = '0;  logic [3:0] s_axi_wstrb = 4'hF;
    logic        s_axi_wvalid = 0;   logic       s_axi_wready;
    logic [1:0]  s_axi_bresp;        logic       s_axi_bvalid;  logic s_axi_bready = 1'b1;
    logic [6:0]  s_axi_araddr = '0;  logic [2:0] s_axi_arprot = '0;
    logic        s_axi_arvalid = 0;  logic       s_axi_arready;
    logic [31:0] s_axi_rdata;        logic [1:0] s_axi_rresp;
    logic        s_axi_rvalid;       logic       s_axi_rready = 1'b1;

    // ---- AXI-Stream ----
    logic [31:0] s_axis_tdata = '0;  logic [9:0] s_axis_tdest = '0;
    logic [0:0]  s_axis_tuser = '0;  logic       s_axis_tlast = 0;
    logic        s_axis_tvalid = 0;  logic       s_axis_tready;
    logic [31:0] m_axis_tdata;       logic [3:0] m_axis_tkeep;
    logic [0:0]  m_axis_tuser;       logic       m_axis_tlast;
    logic        m_axis_tvalid;      logic       m_axis_tready = 1'b1;
    logic        frame_done_irq_o;

    // =========================================================================
    // DUT: the plain-Verilog wrapper (module `mocap_wrapper` from mocap_wrapper.v)
    // =========================================================================
    mocap_wrapper #(
        .MAX_BLOBS        (MAX_BLOBS),
        .MAX_RUNS_PER_ROW (640)
    ) dut (
        .aclk             (clk),
        .aresetn          (aresetn),
        .s_axi_awaddr     (s_axi_awaddr),   .s_axi_awprot  (s_axi_awprot),
        .s_axi_awvalid    (s_axi_awvalid),  .s_axi_awready (s_axi_awready),
        .s_axi_wdata      (s_axi_wdata),    .s_axi_wstrb   (s_axi_wstrb),
        .s_axi_wvalid     (s_axi_wvalid),   .s_axi_wready  (s_axi_wready),
        .s_axi_bresp      (s_axi_bresp),    .s_axi_bvalid  (s_axi_bvalid),
        .s_axi_bready     (s_axi_bready),
        .s_axi_araddr     (s_axi_araddr),   .s_axi_arprot  (s_axi_arprot),
        .s_axi_arvalid    (s_axi_arvalid),  .s_axi_arready (s_axi_arready),
        .s_axi_rdata      (s_axi_rdata),    .s_axi_rresp   (s_axi_rresp),
        .s_axi_rvalid     (s_axi_rvalid),   .s_axi_rready  (s_axi_rready),
        .s_axis_tdata     (s_axis_tdata),   .s_axis_tdest  (s_axis_tdest),
        .s_axis_tuser     (s_axis_tuser),   .s_axis_tlast  (s_axis_tlast),
        .s_axis_tvalid    (s_axis_tvalid),  .s_axis_tready (s_axis_tready),
        .m_axis_tdata     (m_axis_tdata),   .m_axis_tkeep  (m_axis_tkeep),
        .m_axis_tuser     (m_axis_tuser),   .m_axis_tlast  (m_axis_tlast),
        .m_axis_tvalid    (m_axis_tvalid),  .m_axis_tready (m_axis_tready),
        .frame_done_irq_o (frame_done_irq_o)
    );

    // =========================================================================
    // Scoreboard
    // =========================================================================
    int pass_count = 0, fail_count = 0;

    task automatic check32(input string label, input logic [31:0] got, input logic [31:0] exp);
        if (got === exp) pass_count++;
        else begin $display("[%0t ns] [FAIL] %s -- exp 0x%08x got 0x%08x", $time, label, exp, got); fail_count++; end
    endtask
    task automatic check(input string label, input logic got, input logic exp);
        if (got === exp) pass_count++;
        else begin $display("[%0t ns] [FAIL] %s -- exp %0b got %0b", $time, label, exp, got); fail_count++; end
    endtask
    task automatic pass_msg(input string label); pass_count++; $display("[%0t ns] [PASS] %s", $time, label); endtask
    task automatic fail_msg(input string label); fail_count++; $display("[%0t ns] [FAIL] %s", $time, label); endtask

    // =========================================================================
    // AXI4-Lite master
    // =========================================================================
    task automatic axi_write(input logic [6:0] addr, input logic [31:0] data);
        @(posedge clk);
        s_axi_awaddr <= addr; s_axi_awvalid <= 1'b1;
        s_axi_wdata  <= data; s_axi_wvalid  <= 1'b1;
        fork
            begin wait (s_axi_awready); @(posedge clk); s_axi_awvalid <= 1'b0; s_axi_awaddr <= '0; end
            begin wait (s_axi_wready);  @(posedge clk); s_axi_wvalid  <= 1'b0; s_axi_wdata  <= '0; end
        join
        wait (s_axi_bvalid);
        @(posedge clk);
    endtask

    task automatic axi_read(input logic [6:0] addr, output logic [31:0] data);
        @(posedge clk);
        s_axi_araddr <= addr; s_axi_arvalid <= 1'b1;
        wait (s_axi_arready);
        @(posedge clk);
        s_axi_arvalid <= 1'b0; s_axi_araddr <= '0;
        wait (s_axi_rvalid);
        data = s_axi_rdata;
        @(posedge clk);
    endtask

    // =========================================================================
    // Stimulus (frame_*.hex generated into the run dir by the Makefile)
    // =========================================================================
    logic [31:0] frame_mem [0:MAX_FRAME_WORDS-1];
    int cur_hres = 640, cur_vres = 400;

    task automatic load_frame_hex(input string fname, input int w, input int h);
        for (int i = 0; i < MAX_FRAME_WORDS; i++) frame_mem[i] = 32'h0;
        $readmemh(fname, frame_mem);
        cur_hres = w; cur_vres = h;
    endtask

    task automatic arm_continuous(input int hres, input int vres, input int thr);
        axi_write(`MOCAP_REG_HRES, 32'(hres));
        axi_write(`MOCAP_REG_VRES, 32'(vres));
        axi_write(`MOCAP_REG_CTRL, `MOCAP_CTRL_ENABLE
                                 | `MOCAP_CTRL_HIST_ADDR_AUTOINC
                                 | `MOCAP_CTRL_BLOB_ADDR_AUTOINC
                                 | `MOCAP_CTRL_THRESHOLD(8'(thr)));
    endtask

    task automatic results_ack();
        axi_write(`MOCAP_REG_CTRL, `MOCAP_CTRL_ENABLE | `MOCAP_CTRL_RESULTS_ACK
                                 | `MOCAP_CTRL_HIST_ADDR_AUTOINC
                                 | `MOCAP_CTRL_BLOB_ADDR_AUTOINC
                                 | `MOCAP_CTRL_THRESHOLD(8'd128));
    endtask

    task automatic stream_beat(input logic [31:0] data, input logic sof, input logic eol);
        int gap;
        s_axis_tdata <= data; s_axis_tuser <= sof; s_axis_tlast <= eol; s_axis_tvalid <= 1'b1;
        @(posedge clk);
        while (!s_axis_tready) @(posedge clk);
        s_axis_tvalid <= 1'b0; s_axis_tdata <= '0; s_axis_tuser <= '0; s_axis_tlast <= 1'b0;
        gap = 4 + $urandom_range(0, 2); // keep the histogram snoop FIFO drained
        repeat (gap) @(posedge clk);
    endtask

    task automatic stream_frame();
        int total_beats;
        total_beats = (cur_hres * cur_vres) / 4;
        for (int i = 0; i < total_beats; i++)
            stream_beat(frame_mem[i], (i == 0), ((((i + 1) * 4) % cur_hres) == 0));
    endtask

    // ---- passthrough capture ----
    logic [31:0] pt_mem [0:MAX_FRAME_WORDS-1];
    int   pt_wr = 0;
    logic pt_active = 1'b0;
    always @(posedge clk)
        if (pt_active && m_axis_tvalid && m_axis_tready) begin
            pt_mem[pt_wr] <= m_axis_tdata; pt_wr <= pt_wr + 1;
        end

    task automatic verify_passthrough(input int total_beats);
        int mism;
        repeat (32) @(posedge clk);
        mism = 0;
        for (int i = 0; i < total_beats; i++) if (pt_mem[i] !== frame_mem[i]) mism++;
        if (mism == 0) pass_count++;
        else begin $display("[%0t ns] [FAIL] passthrough: %0d mismatches", $time, mism); fail_count++; end
        check32("W1: passthrough word count", 32'(pt_wr), 32'(total_beats));
    endtask

    task automatic wait_irq(input int timeout_cycles, output bit ok);
        int t; ok = 1'b1; t = 0;
        while (!frame_done_irq_o) begin
            @(posedge clk); t++;
            if (t > timeout_cycles) begin fail_msg("Timeout waiting for frame_done_irq_o"); ok = 1'b0; return; end
        end
    endtask

    task automatic dump_histogram(output logic [31:0] hist [0:255]);
        logic [31:0] rd;
        axi_write(`MOCAP_REG_HIST_ADDR, 32'h0);
        for (int i = 0; i < 256; i++) begin axi_read(`MOCAP_REG_HIST_DATA, rd); hist[i] = rd & 32'hFFFFF; end
    endtask

    task automatic hw_reset();
        aresetn = 1'b0; repeat (20) @(posedge clk);
        aresetn = 1'b1; repeat (400) @(posedge clk); // post-reset scrub of both hist banks
    endtask

    // =========================================================================
    // Test sequence
    // =========================================================================
    bit irq_ok;

    initial begin : test_seq
        logic [31:0] status, frame_id, pixel_sum, blob_cnt, rd;
        logic [31:0] hist [0:255];
        logic [63:0] hist_mass, weighted;
        logic [31:0] read_bank_1st;
        logic [31:0] lo1, hi1, lo2, hi2;
        logic [63:0] snap1, snap2;
        int total_beats;

        $display("\n==============================================");
        $display("  tb_mocap_wrapper_v -- Verilog-wrapper integration suite");
        $display("==============================================");

        hw_reset();

        // Sanity: regblock reachable through the wrapper (reset defaults).
        axi_read(`MOCAP_REG_HRES, rd); check32("W0: HRES default", rd, 32'd640);
        axi_read(`MOCAP_REG_VRES, rd); check32("W0: VRES default", rd, 32'd400);
        axi_read(`MOCAP_REG_MAX_BLOBS_CFG, rd); check32("W0: MAX_BLOBS_CFG", rd, 32'(MAX_BLOBS));

        // =====================================================================
        // W1 -- one full frame through every block, then a 2nd frame (ping-pong)
        //   frame_0002 is 640x400 with a known golden of 10 blobs.
        // =====================================================================
        begin : w1
            load_frame_hex("frame_0002.hex", 640, 400);
            total_beats = (640 * 400) / 4;

            arm_continuous(640, 400, 128);
            pt_wr = 0; pt_active = 1'b1;
            stream_frame();

            wait_irq(IRQ_TIMEOUT, irq_ok);         // FC FSM + isp_done & blob_done + IRQ pin
            check("W1: frame_done_irq_o asserted", frame_done_irq_o, 1'b1);

            axi_read(`MOCAP_REG_STATUS, status);
            check("W1: RESULTS_VALID after publish", status[`MOCAP_STATUS_RESULTS_VALID_B], 1'b1);
            check("W1: FRAME_DONE_IRQ bit set",      status[`MOCAP_STATUS_FRAME_DONE_IRQ_B], 1'b1);
            read_bank_1st = 32'(status[`MOCAP_STATUS_READ_BANK_B]);

            axi_read(`MOCAP_REG_FRAME_ID, frame_id);
            check32("W1: FRAME_ID == 1", frame_id, 32'd1);

            // --- histogram (both instances via snoop) + pixel-sum identity ---
            dump_histogram(hist);
            hist_mass = 0; weighted = 0;
            for (int i = 0; i < 256; i++) begin
                hist_mass += hist[i];
                weighted  += hist[i] * i;
            end
            axi_read(`MOCAP_REG_PIXEL_SUM, pixel_sum);
            if (hist_mass == 64'(640 * 400))
                pass_msg($sformatf("W1: histogram mass == all pixels (%0d)", hist_mass));
            else
                fail_msg($sformatf("W1: histogram mass %0d != %0d", hist_mass, 640 * 400));
            check32("W1: sum(bin[i]*i) == PIXEL_SUM", weighted[31:0], pixel_sum);

            // --- blob core + copy FSM: known golden count for frame_0002 ---
            axi_read(`MOCAP_REG_STATUS, status);
            blob_cnt = `MOCAP_STATUS_GET_BLOB_COUNT(status);
            check32("W1: BLOB_COUNT == 10 (frame_0002 golden)", blob_cnt, 32'd10);

            // Blob field readback wired sanely (bbox ordered, centroid in bbox).
            begin : blob_fields
                logic [31:0] cnt, sx, sy, xmin, xmax, ymin, ymax;
                int bad; bad = 0;
                // autoinc off during the multi-field read (same rule as the SV TB)
                axi_write(`MOCAP_REG_CTRL, `MOCAP_CTRL_ENABLE
                                         | `MOCAP_CTRL_HIST_ADDR_AUTOINC
                                         | `MOCAP_CTRL_THRESHOLD(8'd128));
                for (int b = 0; b < 10; b++) begin
                    axi_write(`MOCAP_REG_BLOB_ADDR, 32'(b));
                    axi_read(`MOCAP_REG_BLOB_COUNT_RD, cnt);
                    axi_read(`MOCAP_REG_BLOB_SX,   sx);
                    axi_read(`MOCAP_REG_BLOB_SY,   sy);
                    axi_read(`MOCAP_REG_BLOB_XMIN, xmin);
                    axi_read(`MOCAP_REG_BLOB_XMAX, xmax);
                    axi_read(`MOCAP_REG_BLOB_YMIN, ymin);
                    axi_read(`MOCAP_REG_BLOB_YMAX, ymax);
                    if (!(cnt > 0 && xmin <= xmax && ymin <= ymax &&
                          sx >= cnt * xmin && sx <= cnt * xmax &&
                          sy >= cnt * ymin && sy <= cnt * ymax)) bad++;
                end
                axi_write(`MOCAP_REG_CTRL, `MOCAP_CTRL_ENABLE
                                         | `MOCAP_CTRL_HIST_ADDR_AUTOINC
                                         | `MOCAP_CTRL_BLOB_ADDR_AUTOINC
                                         | `MOCAP_CTRL_THRESHOLD(8'd128));
                if (bad == 0) pass_msg("W1: all 10 blob records internally consistent");
                else          fail_msg($sformatf("W1: %0d blob records inconsistent", bad));
            end

            // --- datapath: m_axis == s_axis (in/out stream FIFOs) ---
            verify_passthrough(total_beats);

            // --- ACK releases the buffer ---
            results_ack();
            @(posedge clk);
            axi_read(`MOCAP_REG_STATUS, status);
            check("W1: RESULTS_VALID cleared by ACK", status[`MOCAP_STATUS_RESULTS_VALID_B], 1'b0);

            // --- 2nd frame: ping-pong (bank flips, FRAME_ID advances) ---
            pt_wr = 0; pt_active = 1'b0;
            stream_frame();
            wait_irq(IRQ_TIMEOUT, irq_ok);
            axi_read(`MOCAP_REG_FRAME_ID, frame_id);
            check32("W1: FRAME_ID == 2 after 2nd frame", frame_id, 32'd2);
            axi_read(`MOCAP_REG_STATUS, status);
            check("W1: READ_BANK toggled (double buffer)",
                  status[`MOCAP_STATUS_READ_BANK_B], ~read_bank_1st[0]);
            results_ack();
        end

        // =====================================================================
        // W2 -- free-running cycle counter + snapshot, through the wrapper
        // =====================================================================
        begin : w2
            axi_write(`MOCAP_REG_CTRL, `MOCAP_CTRL_CYCLE_SNAPSHOT);
            axi_read(`MOCAP_REG_CYCLE_SNAP_LO, lo1);
            axi_read(`MOCAP_REG_CYCLE_SNAP_HI, hi1);
            snap1 = {hi1, lo1};
            repeat (1000) @(posedge clk);
            axi_write(`MOCAP_REG_CTRL, `MOCAP_CTRL_CYCLE_SNAPSHOT);
            axi_read(`MOCAP_REG_CYCLE_SNAP_LO, lo2);
            axi_read(`MOCAP_REG_CYCLE_SNAP_HI, hi2);
            snap2 = {hi2, lo2};
            if ((snap2 > snap1) && ((snap2 - snap1) >= 64'd1000) && ((snap2 - snap1) < 64'd1400))
                pass_msg($sformatf("W2: cycle counter free-runs through wrapper (delta=%0d)", snap2 - snap1));
            else
                fail_msg($sformatf("W2: cycle delta out of range: %0d", snap2 - snap1));
        end

        $display("\n==============================================");
        $display("  Results: %0d passed, %0d failed", pass_count, fail_count);
        $display("==============================================");
        if (fail_count == 0) $display("  SUCCESS: ALL TESTS PASSED");
        else                 $display("  ERROR: %0d FAILURE(S) DETECTED", fail_count);
        $finish;
    end

    initial begin : timeout_watchdog
        #50ms;
        $display("[TIMEOUT] Simulation exceeded 50 ms");
        $fatal;
    end

endmodule
