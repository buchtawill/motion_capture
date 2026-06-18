// tb_blob_detect_grid.sv
// Testbench for blob_detect_grid_wrapper (grid-based blob detection IP).
//
// Test sequence:
//   1. Reset
//   2. Read MAX_BLOBS_CFG, verify == 128
//   3. Configure HRES=1280, VRES=800
//   4. Load frame from hex file
//   5. Write CTRL: START=1, THRESHOLD=128
//   6. Wait for STATUS.READY to go low
//   7. Stream frame via AXIS with random gaps; capture passthrough output
//   8. Wait for frame_done_irq_o
//   9. Read STATUS, extract blob count
//  10. Read blob descriptors (auto-inc disabled)
//  11. Write RTL results to blobs_rtl.hex
//  12. Verify AXIS passthrough
//  13. Print pass/fail summary, $finish

`timescale 1ns / 1ps

module tb_blob_detect_grid;

    // =========================================================================
    // Parameters
    // =========================================================================
    localparam real    CLK_HALF_NS     = 2.5;          // 200 MHz
    localparam int     HRES            = 1280;
    localparam int     VRES            = 800;
    localparam int     MAX_FRAME_WORDS = HRES * VRES / 4; // 256000 words
    localparam int     MAX_BLOBS_PARAM = 128;
    localparam string  FRAME_FILE      = "frame_0000.hex";

    // AXI-Lite register addresses
    localparam logic [5:0] ADDR_CTRL          = 6'h00;
    localparam logic [5:0] ADDR_STATUS        = 6'h04;
    localparam logic [5:0] ADDR_HRES          = 6'h08;
    localparam logic [5:0] ADDR_VRES          = 6'h0C;
    localparam logic [5:0] ADDR_BLOB_ADDR     = 6'h10;
    localparam logic [5:0] ADDR_BLOB_COUNT_RD = 6'h14;
    localparam logic [5:0] ADDR_BLOB_SX       = 6'h18;
    localparam logic [5:0] ADDR_BLOB_SY       = 6'h1C;
    localparam logic [5:0] ADDR_BLOB_XMIN     = 6'h20;
    localparam logic [5:0] ADDR_BLOB_XMAX     = 6'h24;
    localparam logic [5:0] ADDR_BLOB_YMIN     = 6'h28;
    localparam logic [5:0] ADDR_BLOB_YMAX     = 6'h2C;
    localparam logic [5:0] ADDR_FRAME_CNT     = 6'h30;
    localparam logic [5:0] ADDR_MAX_BLOBS_CFG = 6'h34;

    // =========================================================================
    // Clock / reset
    // =========================================================================
    logic clk    = 1'b0;
    logic aresetn = 1'b0;

    always #CLK_HALF_NS clk = ~clk;

    // =========================================================================
    // DUT signals
    // =========================================================================

    // AXI-Lite
    logic [5:0]  s_axi_awaddr  = '0;
    logic [2:0]  s_axi_awprot  = '0;
    logic        s_axi_awvalid = 1'b0;
    logic        s_axi_awready;

    logic [31:0] s_axi_wdata   = '0;
    logic [3:0]  s_axi_wstrb   = 4'hF;
    logic        s_axi_wvalid  = 1'b0;
    logic        s_axi_wready;

    logic [1:0]  s_axi_bresp;
    logic        s_axi_bvalid;
    logic        s_axi_bready  = 1'b1;

    logic [5:0]  s_axi_araddr  = '0;
    logic [2:0]  s_axi_arprot  = '0;
    logic        s_axi_arvalid = 1'b0;
    logic        s_axi_arready;

    logic [31:0] s_axi_rdata;
    logic [1:0]  s_axi_rresp;
    logic        s_axi_rvalid;
    logic        s_axi_rready  = 1'b1;

    // AXI-Stream slave (input)
    logic [31:0] s_axis_tdata  = '0;
    logic [0:0]  s_axis_tuser  = '0;
    logic        s_axis_tlast  = 1'b0;
    logic        s_axis_tvalid = 1'b0;
    logic        s_axis_tready;

    // AXI-Stream master (passthrough output)
    logic [31:0] m_axis_tdata;
    logic [3:0]  m_axis_tkeep;
    logic [0:0]  m_axis_tuser;
    logic        m_axis_tlast;
    logic        m_axis_tvalid;
    logic        m_axis_tready = 1'b1;

    // Interrupt
    logic        frame_done_irq_o;

    // =========================================================================
    // DUT instantiation (wrapper)
    // =========================================================================
    blob_detect_grid_wrapper dut (
        .aclk             (clk),
        .aresetn          (aresetn),

        .s_axi_awaddr     (s_axi_awaddr),
        .s_axi_awprot     (s_axi_awprot),
        .s_axi_awvalid    (s_axi_awvalid),
        .s_axi_awready    (s_axi_awready),

        .s_axi_wdata      (s_axi_wdata),
        .s_axi_wstrb      (s_axi_wstrb),
        .s_axi_wvalid     (s_axi_wvalid),
        .s_axi_wready     (s_axi_wready),

        .s_axi_bresp      (s_axi_bresp),
        .s_axi_bvalid     (s_axi_bvalid),
        .s_axi_bready     (s_axi_bready),

        .s_axi_araddr     (s_axi_araddr),
        .s_axi_arprot     (s_axi_arprot),
        .s_axi_arvalid    (s_axi_arvalid),
        .s_axi_arready    (s_axi_arready),

        .s_axi_rdata      (s_axi_rdata),
        .s_axi_rresp      (s_axi_rresp),
        .s_axi_rvalid     (s_axi_rvalid),
        .s_axi_rready     (s_axi_rready),

        .s_axis_tdata     (s_axis_tdata),
        .s_axis_tuser     (s_axis_tuser),
        .s_axis_tlast     (s_axis_tlast),
        .s_axis_tvalid    (s_axis_tvalid),
        .s_axis_tready    (s_axis_tready),

        .m_axis_tdata     (m_axis_tdata),
        .m_axis_tkeep     (m_axis_tkeep),
        .m_axis_tuser     (m_axis_tuser),
        .m_axis_tlast     (m_axis_tlast),
        .m_axis_tvalid    (m_axis_tvalid),
        .m_axis_tready    (m_axis_tready),

        .frame_done_irq_o (frame_done_irq_o)
    );

    // =========================================================================
    // Pass / fail counters
    // =========================================================================
    int pass_count = 0;
    int fail_count = 0;

    // =========================================================================
    // Frame and passthrough memories
    // =========================================================================
    logic [31:0] frame_mem       [0:MAX_FRAME_WORDS-1];
    logic [31:0] passthrough_mem [0:MAX_FRAME_WORDS-1];

    // Passthrough capture index (driven by always block)
    int pt_idx = 0;

    // Blob descriptor storage
    typedef struct {
        logic [31:0] count;
        logic [31:0] sum_x;
        logic [31:0] sum_y;
        logic [31:0] xmin;
        logic [31:0] xmax;
        logic [31:0] ymin;
        logic [31:0] ymax;
    } blob_desc_t;

    blob_desc_t blob_data [0:MAX_BLOBS_PARAM-1];

    // =========================================================================
    // Passthrough capture (always block)
    // =========================================================================
    always @(posedge clk) begin
        if (m_axis_tvalid && m_axis_tready) begin
            passthrough_mem[pt_idx] <= m_axis_tdata;
            pt_idx <= pt_idx + 1;
        end
    end

    // Backpressure generation: occasionally deassert m_axis_tready.
    // Uses a simple LFSR-style toggle — not cycle-accurate, just stress test.
    logic [7:0] bp_lfsr = 8'hA5;
    always @(posedge clk) begin
        if (aresetn) begin
            bp_lfsr      <= {bp_lfsr[6:0], bp_lfsr[7] ^ bp_lfsr[5]};
            m_axis_tready <= (bp_lfsr[1:0] != 2'b00); // deassert ~25% of cycles
        end else begin
            m_axis_tready <= 1'b1;
        end
    end

    // =========================================================================
    // check — single-bit assertion helper
    // =========================================================================
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

    // =========================================================================
    // check32 — 32-bit value assertion helper
    // =========================================================================
    task automatic check32(input string label,
                            input logic [31:0] got,
                            input logic [31:0] exp);
        if (got === exp) begin
            $display("[%0t ns] [PASS] %s: got 0x%08x", $time, label, got);
            pass_count++;
        end else begin
            $display("[%0t ns] [FAIL] %s — expected 0x%08x, got 0x%08x",
                     $time, label, exp, got);
            fail_count++;
        end
    endtask

    // =========================================================================
    // AXI-Lite write task
    // =========================================================================
    task automatic axi_write(input logic [5:0] addr, input logic [31:0] data);
        @(posedge clk);
        s_axi_awaddr  <= addr;
        s_axi_awprot  <= 3'b000;
        s_axi_awvalid <= 1'b1;
        s_axi_wdata   <= data;
        s_axi_wstrb   <= 4'hF;
        s_axi_wvalid  <= 1'b1;

        // Wait for both awready and wready
        fork
            begin : wait_aw
                wait (s_axi_awready);
                @(posedge clk);
                s_axi_awvalid <= 1'b0;
                s_axi_awaddr  <= '0;
            end
            begin : wait_w
                wait (s_axi_wready);
                @(posedge clk);
                s_axi_wvalid <= 1'b0;
                s_axi_wdata  <= '0;
            end
        join

        // Wait for write response
        wait (s_axi_bvalid);
        @(posedge clk);
    endtask

    // =========================================================================
    // AXI-Lite read task
    // =========================================================================
    task automatic axi_read(input logic [5:0] addr, output logic [31:0] data);
        @(posedge clk);
        s_axi_araddr  <= addr;
        s_axi_arprot  <= 3'b000;
        s_axi_arvalid <= 1'b1;

        // Wait for arready
        wait (s_axi_arready);
        @(posedge clk);
        s_axi_arvalid <= 1'b0;
        s_axi_araddr  <= '0;

        // Wait for rvalid
        wait (s_axi_rvalid);
        data = s_axi_rdata;
        @(posedge clk);
    endtask

    // =========================================================================
    // Main test sequence
    // =========================================================================
    initial begin : test_seq
        int          total_beats;
        logic [31:0] rdata;
        logic [31:0] status_val;
        int          blob_count;
        int          fd;

        $timeformat(-9, 0, "", 1);
        $display("==============================================");
        $display("  tb_blob_detect_grid  %0dx%0d", HRES, VRES);
        $display("==============================================");

        total_beats = MAX_FRAME_WORDS; // HRES * VRES / 4

        // ----------------------------------------------------------------
        // Step 1: Reset (aresetn low 20 cycles, then high, wait 10)
        // ----------------------------------------------------------------
        aresetn = 1'b0;
        repeat(20) @(posedge clk);
        aresetn = 1'b1;
        repeat(10) @(posedge clk);
        $display("[%0t ns] [INFO] Reset released", $time);

        // ----------------------------------------------------------------
        // Step 2: Read MAX_BLOBS_CFG, verify == 128
        // ----------------------------------------------------------------
        $display("\n[%0t ns] --- Step 2: Read MAX_BLOBS_CFG ---", $time);
        axi_read(ADDR_MAX_BLOBS_CFG, rdata);
        check32("MAX_BLOBS_CFG", rdata & 32'hFFFF, 32'd128);

        // ----------------------------------------------------------------
        // Step 3: Configure HRES and VRES
        // ----------------------------------------------------------------
        $display("\n[%0t ns] --- Step 3: Configure HRES=%0d VRES=%0d ---",
                 $time, HRES, VRES);
        axi_write(ADDR_HRES, 32'(HRES));
        axi_write(ADDR_VRES, 32'(VRES));

        // ----------------------------------------------------------------
        // Step 4: Load frame from hex file
        // ----------------------------------------------------------------
        $display("\n[%0t ns] --- Step 4: Load frame from %s ---", $time, FRAME_FILE);
        $readmemh(FRAME_FILE, frame_mem);
        $display("[%0t ns] [INFO] Frame loaded (%0d words)", $time, total_beats);

        // ----------------------------------------------------------------
        // Step 5: Write CTRL: START=1, THRESHOLD=128, AUTOINC=1
        // ----------------------------------------------------------------
        $display("\n[%0t ns] --- Step 5: Write CTRL (START, THRESHOLD=128) ---", $time);
        // CTRL: [15:8]=threshold, [3]=autoinc, [2]=irq_clear, [1]=start, [0]=reset
        // autoinc defaults to 1; set threshold=128 and start=1
        axi_write(ADDR_CTRL, (32'd128 << 4) | 32'h0A); // threshold=128, autoinc=1, start=1

        // ----------------------------------------------------------------
        // Step 6: Wait for STATUS.READY to go low (FSM left IDLE)
        // ----------------------------------------------------------------
        $display("\n[%0t ns] --- Step 6: Wait for STATUS.READY = 0 ---", $time);
        begin : wait_not_ready
            int timeout;
            timeout = 0;
            do begin
                @(posedge clk);
                timeout++;
                if (timeout > 1000) begin
                    $display("[%0t ns] [FAIL] Timeout waiting for STATUS.READY=0", $time);
                    fail_count++;
                    disable wait_not_ready;
                end
            end while (dut.u_blob_detect_grid_top.state == 3'd0); // ST_IDLE=0
        end
        $display("[%0t ns] [INFO] FSM left IDLE", $time);

        // ----------------------------------------------------------------
        // Step 7: Stream the frame via AXIS with random gaps
        // ----------------------------------------------------------------
        $display("\n[%0t ns] --- Step 7: Stream %0d beats ---", $time, total_beats);
        begin : stream_frame
            int gap;
            for (int i = 0; i < total_beats; i++) begin
                // Deassert valid between beats for random gaps
                s_axis_tdata  <= frame_mem[i];
                s_axis_tuser  <= (i == 0) ? 1'b1 : 1'b0;
                s_axis_tlast  <= (((i + 1) * 4) % HRES == 0) ? 1'b1 : 1'b0;
                s_axis_tvalid <= 1'b1;

                // Wait for handshake
                @(posedge clk);
                while (!s_axis_tready) @(posedge clk);

                // Deassert valid and add random gap
                s_axis_tvalid <= 1'b0;
                s_axis_tdata  <= '0;
                s_axis_tuser  <= '0;
                s_axis_tlast  <= 1'b0;

                gap = $urandom_range(0, 3);
                repeat(gap) @(posedge clk);

                if (i % 32000 == 0)
                    $display("[%0t ns] [INFO]   beat %0d / %0d", $time, i, total_beats);
            end
        end
        $display("[%0t ns] [INFO] Frame streaming complete", $time);

        // ----------------------------------------------------------------
        // Step 8: Wait for frame_done_irq_o (timeout 50 ms)
        // ----------------------------------------------------------------
        $display("\n[%0t ns] --- Step 8: Wait for frame_done_irq_o ---", $time);
        begin : wait_irq
            int timeout;
            timeout = 0;
            while (!frame_done_irq_o) begin
                @(posedge clk);
                timeout++;
                if (timeout > 10_000_000) begin
                    $display("[%0t ns] [FAIL] Timeout waiting for frame_done_irq_o", $time);
                    fail_count++;
                    disable wait_irq;
                end
            end
        end
        $display("[%0t ns] [INFO] frame_done_irq_o asserted", $time);
        check("frame_done_irq_o", frame_done_irq_o, 1'b1);

        // Clear IRQ
        axi_write(ADDR_CTRL, (32'd128 << 4) | 32'h0C); // threshold=128, autoinc=1, irq_clear=1

        // ----------------------------------------------------------------
        // Step 9: Read STATUS, extract BLOB_COUNT
        // ----------------------------------------------------------------
        $display("\n[%0t ns] --- Step 9: Read STATUS ---", $time);
        axi_read(ADDR_STATUS, status_val);
        blob_count = int'(status_val[11:4]);
        $display("[%0t ns] [INFO] STATUS = 0x%08x  BLOB_COUNT = %0d",
                 $time, status_val, blob_count);
        check("STATUS.FRAME_DONE", status_val[1], 1'b1);

        // ----------------------------------------------------------------
        // Step 10: Read blob descriptors (auto-increment disabled)
        // ----------------------------------------------------------------
        $display("\n[%0t ns] --- Step 10: Read %0d blob descriptors ---",
                 $time, blob_count);

        // Disable auto-increment: threshold=128, autoinc=0, no pulses
        axi_write(ADDR_CTRL, (32'd128 << 4)); // bit[3]=0 => autoinc off

        for (int b = 0; b < blob_count; b++) begin
            axi_write(ADDR_BLOB_ADDR, 32'(b));
            axi_read(ADDR_BLOB_COUNT_RD, blob_data[b].count);
            axi_read(ADDR_BLOB_SX,       blob_data[b].sum_x);
            axi_read(ADDR_BLOB_SY,       blob_data[b].sum_y);
            axi_read(ADDR_BLOB_XMIN,     blob_data[b].xmin);
            axi_read(ADDR_BLOB_XMAX,     blob_data[b].xmax);
            axi_read(ADDR_BLOB_YMIN,     blob_data[b].ymin);
            axi_read(ADDR_BLOB_YMAX,     blob_data[b].ymax);
            $display("[%0t ns] [INFO] blob[%0d]: count=%0d sx=%0d sy=%0d xmin=%0d xmax=%0d ymin=%0d ymax=%0d",
                $time, b,
                blob_data[b].count,
                blob_data[b].sum_x,
                blob_data[b].sum_y,
                blob_data[b].xmin,
                blob_data[b].xmax,
                blob_data[b].ymin,
                blob_data[b].ymax);
        end

        // ----------------------------------------------------------------
        // Step 11: Write RTL results to blobs_rtl.hex
        // ----------------------------------------------------------------
        $display("\n[%0t ns] --- Step 11: Write blobs_rtl.hex ---", $time);
        fd = $fopen("blobs_rtl.hex", "w");
        if (fd == 0) begin
            $display("[%0t ns] [WARN] Could not open blobs_rtl.hex for writing", $time);
        end else begin
            $fwrite(fd, "%08x\n", blob_count);
            for (int b = 0; b < blob_count; b++) begin
                $fwrite(fd, "%08x\n", blob_data[b].count);
                $fwrite(fd, "%08x\n", blob_data[b].sum_x);
                $fwrite(fd, "%08x\n", blob_data[b].sum_y);
                $fwrite(fd, "%08x\n", blob_data[b].xmin);
                $fwrite(fd, "%08x\n", blob_data[b].xmax);
                $fwrite(fd, "%08x\n", blob_data[b].ymin);
                $fwrite(fd, "%08x\n", blob_data[b].ymax);
            end
            $fclose(fd);
            $display("[%0t ns] [INFO] Wrote %0d blobs to blobs_rtl.hex", $time, blob_count);
        end

        // ----------------------------------------------------------------
        // Step 12: Verify AXIS passthrough
        // ----------------------------------------------------------------
        $display("\n[%0t ns] --- Step 12: Verify AXIS passthrough (%0d words) ---",
                 $time, total_beats);

        // Wait a few cycles for any remaining passthrough beats to drain
        repeat(32) @(posedge clk);

        begin : check_passthrough
            int pt_mismatches;
            pt_mismatches = 0;
            for (int i = 0; i < total_beats; i++) begin
                if (passthrough_mem[i] !== frame_mem[i]) begin
                    if (pt_mismatches < 10)
                        $display("[%0t ns] [FAIL] Passthrough mismatch at word %0d: got 0x%08x expected 0x%08x",
                                 $time, i, passthrough_mem[i], frame_mem[i]);
                    pt_mismatches++;
                end
            end
            if (pt_mismatches == 0) begin
                $display("[%0t ns] [PASS] AXIS passthrough: all %0d words match",
                         $time, total_beats);
                pass_count++;
            end else begin
                $display("[%0t ns] [FAIL] AXIS passthrough: %0d mismatches out of %0d words",
                         $time, pt_mismatches, total_beats);
                fail_count++;
            end
        end

        // Also check total beat count
        check32("Passthrough word count", 32'(pt_idx), 32'(total_beats));

        // ----------------------------------------------------------------
        // Step 13: Summary and finish
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
    // Timeout watchdog — abort if simulation hangs
    // =========================================================================
    initial begin : timeout_watchdog
        #50ms;
        $display("[%0t ns] [TIMEOUT] Simulation exceeded 50 ms — possible hang", $time);
        $fatal;
    end

endmodule
