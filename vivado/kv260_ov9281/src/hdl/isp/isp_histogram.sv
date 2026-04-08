`timescale 1ns/1ps

// 256 bin histogram
// Pixel data is coming in through pix_data_i, 4 pixels per stream beat (8 bit pixels).
// This module has the following requirements:
//  - Histogram RAM is scrubbed (zerod out) when ram_scrub_i is strobed high. Strobes are 
//    only monitored in this module if hist_en_i is low.
//  - When hist_en_i is high, this module will sniff the connected axi stream and increment 
//    ram counts in hist_mem. 
//  - When hist_en_i is low and the ram is not being scrubbed, ram_data_o should show the 
//    histogram value at ram location ram_addr_i.

`define FF(q, d, rst_val, clk, rst_n) \
    always_ff @(posedge clk or negedge rst_n) begin \
        if (!rst_n) q <= rst_val; \
        else        q <= d; \
    end

module isp_histogram #(
    parameter STREAM_WIDTH = 32,
    parameter RAM_WIDTH = 20
    )(
    input  logic clk_i,
    input  logic rst_n,
    input  logic hist_en_i,    // 1 when the counting is enabled. Higher level module controls this bit
                               // RAM can only be read back when histogram is not enabled
    input  logic ram_scrub_i,  // strobe 1 when the RAM needs to be cleared
    output logic hist_rdy_o,   // output when the RAM is not being scrubbed / ready to listen
    output logic err_o,        // Error bit. Raised if data coming in too fast or count is full etc

    // RAM controls - read only exposed
    input  logic [7:0]           ram_addr_i,
    output logic [RAM_WIDTH-1:0] ram_data_o,
    output logic                 ram_data_o_vld,

    // Axi stream that we are monitoring
    input  logic [STREAM_WIDTH-1:0] pix_data_i,
    input  logic                    pix_data_vld_i,
    input  logic                    pix_data_rdy_i 
);

    // Pixels arrive 4-wide at ~147 Mpix/s, so the stream beats arrive at
    // ~36.75 MHz.  At 200 MHz we process one pixel per clock (200 Mpix/s),
    // so a shallow FIFO is enough to absorb bursts.
    // FIFO stores full 32-bit beats; the read side unpacks one byte per clock.

    // -------------------------------------------------------------------------
    // Input FIFO (write side)
    // -------------------------------------------------------------------------
    localparam FIFO_DEPTH = 16;

    logic                    fifo_s_valid;
    logic                    fifo_s_ready;
    logic [STREAM_WIDTH-1:0] fifo_s_data;
    logic                    fifo_m_valid;
    logic                    fifo_m_ready;
    logic [STREAM_WIDTH-1:0] fifo_m_data;
    logic                    fifo_empty;

    // Enqueue when a valid pixel beat is snooped and histogram is enabled
    assign fifo_s_valid = pix_data_vld_i & pix_data_rdy_i & hist_en_i;
    assign fifo_s_data  = pix_data_i;

    stream_fifo #(
        .DATA_WIDTH (STREAM_WIDTH),
        .DEPTH      (FIFO_DEPTH)
    ) u_input_fifo (
        .clk     (clk_i),
        .rst_n   (rst_n),
        .s_valid (fifo_s_valid),
        .s_ready (fifo_s_ready),
        .s_data  (fifo_s_data),
        .m_valid (fifo_m_valid),
        .m_ready (fifo_m_ready),
        .m_data  (fifo_m_data),
        .empty   (fifo_empty)
    );

    // Overflow error: a valid beat was lost because the FIFO was full
    `FF(err_o, fifo_s_valid & ~fifo_s_ready | err_o, 1'b0, clk_i, rst_n)

    // -------------------------------------------------------------------------
    // Histogram RAM
    // -------------------------------------------------------------------------
    logic [RAM_WIDTH-1:0] hist_mem[0:255];

    // -------------------------------------------------------------------------
    // Read-side FSM: pop one beat, process each byte in turn
    // -------------------------------------------------------------------------
    typedef enum logic [1:0] {
        S_IDLE = 2'b00,   // waiting for FIFO data
        S_ACTIVE
    } state_t;

    state_t                  state,     next_state;
    logic [3:0]              byte_idx_q,  byte_idx_d;
    logic [RAM_WIDTH-1:0]    ram_rd_val, ram_wr_val;
    logic [STREAM_WIDTH-1:0] beat_shift_d, beat_shift_q;
    logic [7:0]              ram_wr_addr, ram_rd_addr;
    logic                    ram_wr_valid;
    logic [7:0]              pixel_q, pixel_d; // current pixel and previous pixel

    // Pop from FIFO only when idle (about to start a new beat)
    // assign fifo_m_ready = (state == S_IDLE);

    // ---- State registers (FF macro) -----------------------------------------
    `FF(state,          next_state,      S_IDLE, clk_i, rst_n)
    `FF(byte_idx_q,     byte_idx_d,      '0,     clk_i, rst_n)
    `FF(ram_addr_d_q,   ram_addr_d_d,    '0,     clk_i, rst_n)
    `FF(pixel_q,        pixel_d,         '0,     clk_i, rst_n)
    `FF(beat_shift_q,   beat_shift_d,    '0,     clk_i, rst_n)

    // ---- RAM port A (read) + port B (write) - single always_ff block --------
    always_ff @(posedge clk_i) begin
        // Port A: read current bin value whenever we are about to write
        // (read is registered; result is available the cycle after S_READ)
        ram_rd_val <= hist_mem[ram_rd_addr];

        // Port B: write incremented value back on S_WRITE
        if (ram_wr_valid)
            hist_mem[ram_wr_addr] <= ram_wr_val;
    end

    // ---- Next-state logic ---------------------------------------------------
    always_comb begin
        next_state    = state;
        byte_idx_d = byte_idx_q;
        fifo_m_ready = 1'b0;

        ram_wr_valid  = 1'b0;
        ram_wr_val    = '0;
        ram_wr_addr   = 8'h0;
        pixel_d = pixel_q;
        beat_shift_d = beat_shift_q;


        case (state)
            // Cold start case
            S_IDLE: begin
                byte_idx_d = '0;
                if (fifo_m_valid) begin
                    fifo_m_ready = 1'b1;
                    beat_shift_d = fifo_m_data;
                    next_state = S_ACTIVE;
                end
            end

            S_ACTIVE: begin
                byte_idx_d = byte_idx_d + 1;
                pixel_d = fifo_m_data
            end


            default: next_state = S_IDLE;
        endcase
    end


endmodule