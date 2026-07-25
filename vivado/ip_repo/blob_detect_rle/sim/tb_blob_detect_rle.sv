`timescale 1ns / 1ps

module tb_blob_detect_rle;

    localparam real    CLK_HALF_NS     = 2.5;
    localparam int     MAX_FRAME_WORDS = 1280 * 800 / 4;
    localparam int     MAX_BLOBS_PARAM = 128;

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
    // AXI-Lite signals
    // =========================================================================
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

    // =========================================================================
    // AXI-Stream signals
    // =========================================================================
    logic [31:0] s_axis_tdata  = '0;
    logic [0:0]  s_axis_tuser  = '0;
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
    blob_detect_rle_wrapper dut (
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
    // Test infrastructure
    // =========================================================================
    int pass_count = 0;
    int fail_count = 0;

    logic [31:0] frame_mem [0:MAX_FRAME_WORDS-1];
    logic [31:0] pt_mem    [0:MAX_FRAME_WORDS-1];
    int          pt_wr      = 0;
    logic        pt_active  = 1'b0;

    typedef struct {
        logic [31:0] count, sum_x, sum_y, xmin, xmax, ymin, ymax;
    } blob_desc_t;

    // Passthrough capture
    always @(posedge clk) begin
        if (pt_active && m_axis_tvalid && m_axis_tready) begin
            pt_mem[pt_wr] <= m_axis_tdata;
            pt_wr <= pt_wr + 1;
        end
    end

    // Downstream backpressure (LFSR-based random ready)
    logic [7:0] bp_lfsr = 8'hA5;
    always @(posedge clk) begin
        if (aresetn) begin
            bp_lfsr       <= {bp_lfsr[6:0], bp_lfsr[7] ^ bp_lfsr[5]};
            m_axis_tready <= (bp_lfsr[1:0] != 2'b00);
        end else begin
            m_axis_tready <= 1'b1;
        end
    end

    // =========================================================================
    // Utility tasks
    // =========================================================================
    task automatic check(input string label, input logic got, input logic exp);
        if (got === exp) begin
            pass_count++;
        end else begin
            $display("[%0t ns] [FAIL] %s — expected %0b, got %0b", $time, label, exp, got);
            fail_count++;
        end
    endtask

    task automatic check32(input string label, input logic [31:0] got, input logic [31:0] exp);
        if (got === exp) begin
            pass_count++;
        end else begin
            $display("[%0t ns] [FAIL] %s — expected 0x%08x, got 0x%08x", $time, label, exp, got);
            fail_count++;
        end
    endtask

    task automatic axi_write(input logic [5:0] addr, input logic [31:0] data);
        @(posedge clk);
        s_axi_awaddr  <= addr;
        s_axi_awprot  <= 3'b000;
        s_axi_awvalid <= 1'b1;
        s_axi_wdata   <= data;
        s_axi_wstrb   <= 4'hF;
        s_axi_wvalid  <= 1'b1;
        fork
            begin wait (s_axi_awready); @(posedge clk); s_axi_awvalid <= 1'b0; s_axi_awaddr <= '0; end
            begin wait (s_axi_wready);  @(posedge clk); s_axi_wvalid  <= 1'b0; s_axi_wdata  <= '0; end
        join
        wait (s_axi_bvalid);
        @(posedge clk);
    endtask

    task automatic axi_read(input logic [5:0] addr, output logic [31:0] data);
        @(posedge clk);
        s_axi_araddr  <= addr;
        s_axi_arprot  <= 3'b000;
        s_axi_arvalid <= 1'b1;
        wait (s_axi_arready);
        @(posedge clk);
        s_axi_arvalid <= 1'b0;
        s_axi_araddr  <= '0;
        wait (s_axi_rvalid);
        data = s_axi_rdata;
        @(posedge clk);
    endtask

    // =========================================================================
    // Stream one beat with random inter-beat gaps
    // =========================================================================
    task automatic stream_beat(input logic [31:0] data, input logic sof, input logic eol);
        int gap;
        s_axis_tdata  <= data;
        s_axis_tuser  <= sof;
        s_axis_tlast  <= eol;
        s_axis_tvalid <= 1'b1;
        @(posedge clk);
        while (!s_axis_tready) @(posedge clk);
        s_axis_tvalid <= 1'b0;
        s_axis_tdata  <= '0;
        s_axis_tuser  <= '0;
        s_axis_tlast  <= 1'b0;
        gap = $urandom_range(0, 3);
        repeat (gap) @(posedge clk);
    endtask

    // =========================================================================
    // Wait for IRQ with timeout
    // =========================================================================
    task automatic wait_irq(output logic ok);
        int timeout;
        ok = 1'b1;
        timeout = 0;
        while (!frame_done_irq_o) begin
            @(posedge clk);
            timeout++;
            if (timeout > 10_000_000) begin
                $display("[%0t ns] [FAIL] Timeout waiting for frame_done_irq_o", $time);
                fail_count++;
                ok = 1'b0;
                return;
            end
        end
    endtask

    // =========================================================================
    // Write blob results to hex file
    // =========================================================================
    task automatic write_blobs_hex(
        input string       filename,
        input int           count,
        input blob_desc_t   blobs [0:MAX_BLOBS_PARAM-1]
    );
        int fd;
        fd = $fopen(filename, "w");
        if (fd == 0) begin
            $display("[%0t ns] [WARN] Could not open %s", $time, filename);
            return;
        end
        $fwrite(fd, "%08x\n", count);
        for (int b = 0; b < count; b++) begin
            $fwrite(fd, "%08x\n", blobs[b].count);
            $fwrite(fd, "%08x\n", blobs[b].sum_x);
            $fwrite(fd, "%08x\n", blobs[b].sum_y);
            $fwrite(fd, "%08x\n", blobs[b].xmin);
            $fwrite(fd, "%08x\n", blobs[b].xmax);
            $fwrite(fd, "%08x\n", blobs[b].ymin);
            $fwrite(fd, "%08x\n", blobs[b].ymax);
        end
        $fclose(fd);
    endtask

    // =========================================================================
    // Verify passthrough data matches input
    // =========================================================================
    task automatic verify_passthrough(input int total_beats);
        int mismatches;
        repeat (32) @(posedge clk);
        mismatches = 0;
        for (int i = 0; i < total_beats; i++) begin
            if (pt_mem[i] !== frame_mem[i]) begin
                if (mismatches < 5)
                    $display("[%0t ns] [FAIL]   passthrough mismatch word %0d: 0x%08x != 0x%08x",
                             $time, i, pt_mem[i], frame_mem[i]);
                mismatches++;
            end
        end
        if (mismatches == 0)
            pass_count++;
        else begin
            $display("[%0t ns] [FAIL]   passthrough: %0d mismatches", $time, mismatches);
            fail_count++;
        end
        check32("  passthrough word count", 32'(pt_wr), 32'(total_beats));
    endtask

    // =========================================================================
    // Run one frame end-to-end
    // =========================================================================
    task automatic run_frame(
        input int     idx,
        input string  frame_file,
        input int     hres,
        input int     vres,
        input int     threshold,
        input int     expected_blobs,
        input string  desc
    );
        int             total_beats;
        int             blob_count;
        logic [31:0]    status_val;
        logic           irq_ok;
        string          out_file;
        blob_desc_t     blobs [0:MAX_BLOBS_PARAM-1];

        total_beats = hres * vres / 4;

        $display("\n========== [%0d] %s ==========", idx, desc);
        $display("[%0t ns] %s  %0dx%0d  thr=%0d  expect=%0d",
                 $time, frame_file, hres, vres, threshold,
                 expected_blobs);

        // --- Configure ---
        axi_write(ADDR_HRES, 32'(hres));
        axi_write(ADDR_VRES, 32'(vres));

        // --- Load frame ---
        for (int i = 0; i < MAX_FRAME_WORDS; i++) frame_mem[i] = 32'h0;
        $readmemh(frame_file, frame_mem);

        // --- Reset passthrough capture ---
        pt_active = 1'b0;
        @(posedge clk);
        pt_wr = 0;
        @(posedge clk);
        pt_active = 1'b1;

        // --- Start (threshold + START + AUTOINC) ---
        axi_write(ADDR_CTRL, (32'(threshold) << 4) | 32'h0A);

        // --- Wait for FSM to leave IDLE ---
        begin : wait_start
            int timeout;
            timeout = 0;
            do begin
                @(posedge clk);
                timeout++;
                if (timeout > 1000) begin
                    $display("[%0t ns] [FAIL] Timeout: FSM stuck in IDLE", $time);
                    fail_count++;
                    return;
                end
            end while (dut.u_blob_detect_rle_top.state == 3'd0);
        end

        // --- Stream frame ---
        for (int i = 0; i < total_beats; i++) begin
            stream_beat(
                frame_mem[i],
                (i == 0) ? 1'b1 : 1'b0,
                (((i + 1) * 4) % hres == 0) ? 1'b1 : 1'b0
            );
        end

        // --- Wait for IRQ ---
        wait_irq(irq_ok);
        if (!irq_ok) return;
        check("  frame_done_irq", frame_done_irq_o, 1'b1);

        // --- Clear IRQ, disable AUTOINC for explicit BLOB_ADDR reads ---
        axi_write(ADDR_CTRL, (32'(threshold) << 4) | 32'h04);

        // --- Read STATUS ---
        axi_read(ADDR_STATUS, status_val);
        blob_count = int'(status_val[11:4]);
        $display("[%0t ns] STATUS=0x%08x  blobs=%0d  overflow=%0b",
                 $time, status_val, blob_count, status_val[2]);
        check("  STATUS.FRAME_DONE", status_val[1], 1'b1);

        // --- Check blob count ---
        if (expected_blobs >= 0)
            check32("  blob_count", 32'(blob_count), 32'(expected_blobs));

        // --- Read blob descriptors ---
        for (int b = 0; b < blob_count; b++) begin
            axi_write(ADDR_BLOB_ADDR, 32'(b));
            axi_read(ADDR_BLOB_COUNT_RD, blobs[b].count);
            axi_read(ADDR_BLOB_SX,       blobs[b].sum_x);
            axi_read(ADDR_BLOB_SY,       blobs[b].sum_y);
            axi_read(ADDR_BLOB_XMIN,     blobs[b].xmin);
            axi_read(ADDR_BLOB_XMAX,     blobs[b].xmax);
            axi_read(ADDR_BLOB_YMIN,     blobs[b].ymin);
            axi_read(ADDR_BLOB_YMAX,     blobs[b].ymax);
            $display("[%0t ns]   blob[%0d]: count=%0d centroid=(%0d/%0d,%0d/%0d) bbox=(%0d,%0d)-(%0d,%0d)",
                     $time, b,
                     blobs[b].count,
                     blobs[b].sum_x, blobs[b].count,
                     blobs[b].sum_y, blobs[b].count,
                     blobs[b].xmin, blobs[b].ymin,
                     blobs[b].xmax, blobs[b].ymax);
        end

        // --- Write output hex ---
        $sformat(out_file, "blobs_rtl_%04d.hex", idx);
        write_blobs_hex(out_file, blob_count, blobs);
        $display("[%0t ns] Wrote %0d blobs to %s", $time, blob_count, out_file);

        // --- Verify passthrough ---
        verify_passthrough(total_beats);

    endtask

    // =========================================================================
    // Main test sequence
    // =========================================================================
    initial begin : test_seq
        logic [31:0] rdata;

        $timeformat(-9, 0, "", 1);
        $display("==============================================");
        $display("  tb_blob_detect_rle — multi-frame test suite");
        $display("==============================================");

        // --- Hardware reset ---
        aresetn = 1'b0;
        repeat (20) @(posedge clk);
        aresetn = 1'b1;
        repeat (10) @(posedge clk);

        // --- Sanity: MAX_BLOBS_CFG ---
        axi_read(ADDR_MAX_BLOBS_CFG, rdata);
        check32("MAX_BLOBS_CFG", rdata & 32'hFFFF, 32'd128);

        // --- Test suite ---
        //       idx   file               hres  vres  thr  exp  description
        run_frame(0, "frame_0000.hex",    1280,  800, 128,   8, "8 random blobs 1280x800");
        run_frame(1, "frame_0001.hex",    1280,  720, 128,   5, "5 random blobs 1280x720");
        run_frame(2, "frame_0002.hex",     640,  400, 128,  10, "10 random blobs 640x400");
        run_frame(3, "frame_0003.hex",    1280,  800, 128,   0, "all-black (no blobs)");
        run_frame(5, "frame_0005.hex",     640,  400, 128,   1, "single pixel blob");
        run_frame(6, "frame_0006.hex",    1280,  800, 128,   2, "nearly touching (gap=2)");
        run_frame(7, "frame_0007.hex",    1280,  800, 128,   1, "overlapping (merge to 1)");

        // --- Summary ---
        $display("\n==============================================");
        $display("  Results: %0d passed, %0d failed", pass_count, fail_count);
        $display("==============================================");
        if (fail_count == 0)
            $display("  SUCCESS: ALL TESTS PASSED");
        else
            $display("  ERROR: %0d FAILURE(S) DETECTED", fail_count);

        $finish;
    end

    initial begin : timeout_watchdog
        #200ms;
        $display("[TIMEOUT] Simulation exceeded 200 ms");
        $fatal;
    end

endmodule
