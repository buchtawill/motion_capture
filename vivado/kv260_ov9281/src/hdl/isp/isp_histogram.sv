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

// pixel_d = beat_sh_q[7:0]
// ram_wr_addr = pixel_q
// ram_rd_addr = pixel_d
// hazard = pixel_d == pixel_q
// ram_wr_val = hazard ? (ram_wr_val_dly + 1) : (ram_rd_val + 1);
// ram_wr_en  = (~hazard) && (pix_cnt_d != 0) && (pix_cnt_q != 0);

/*
Non hazard single beat

{signal: [
    {name: 'clk', wave: 'p........'},
    {name: 'state'     , wave: 'x==....=.', data: "IDLE RUN IDLE"},
    {name: 'next_state', wave: 'x=....=..', data: "RUN IDLE"},
    {name: 'fifo_mvld' , wave: '010......'},
    {name: 'fifo_mrdy' , wave: '1.0......'},
    {name: 'fifo_data' , wave: 'x=x......', data: "12345678 12343400"},
    {name: 'beat_sh_d' , wave: 'x====xxx.', data: "12345678 123456 1234 12 "},
    {name: 'beat_sh_q' , wave: 'x.====xx.', data: "12345678 123456 1234 12"},
    {name: 'pixel_d'   , wave: 'xx====x..', data: "78 56 34 12"},
    {name: 'pixel_q'   , wave: 'xxx====x.', data: "78 56 34 12"},
    {name: 'ram_rd_val', wave: 'xxx====x.', data: "00 00 00 00"},
    {name: 'ram_wr_val', wave: 'xxx====x.', data: "01 01 01 01 "},
    {name: 'wr_valid'  , wave: '0..1...0.', data: ""},
    {name: 'hazard'    , wave: 'x0.......', data: ""},
    {name: 'pix_cnt_d' , wave: 'x======x.', data: "0 1 2 3 4"},
    {name: 'pix_cnt_q' , wave: 'x.=====x.', data: "0  1 2 3 4"},
  ],
  config: { hscale: 2 }  
}

Two beats, hazard in second beat
{signal: [
    {name: 'clk', wave: 'p............'},
    {name: 'state'     , wave: 'x==........=.', data: "IDLE RUN IDLE"},
    {name: 'next_state', wave: 'x=........=..', data: "RUN IDLE"},
    {name: 'fifo_mvld' , wave: '01....0......'},
    {name: 'fifo_mrdy' , wave: '1.0..10......'},
    {name: 'fifo_data' , wave: 'x==...xxxxxxx', data: "12345678 12343400"},
    {name: 'beat_sh_d' , wave: 'x========xxxx', data: "12345678 123456 1234 12 00 34 34 12"},
    {name: 'beat_sh_q' , wave: 'x.========xxx', data: "12345678 123456 1234 12 00 34 34 12"},
    {name: 'pixel_d'   , wave: 'xx========xxx', data: "78 56 34 12 00 34 34 12"},
    {name: 'pixel_q'   , wave: 'xxx========xx', data: "78 56 34 12 00 34 34 12"},
    {name: 'ram_rd_val', wave: 'xxx========xx', data: "00 00 00 00 00 00 00 01"},
    {name: 'ram_wr_val', wave: 'xxx========xx', data: "01 01 01 01 01 01 02 02"},
    {name: 'ram_wr_val_dly', wave: 'xxxx========x', data: "01 01 01 01 01 01 02 02 02"},
    {name: 'wr_valid'  , wave: '0..1....01.0.', data: ""},
    {name: 'hazard'    , wave: 'x0......10...', data: ""},
    {name: 'pix_cnt_d' , wave: '=.=========..', data: "0 1 2 3 0 1 2 3 4 0"},
    {name: 'pix_cnt_q' , wave: 'x.==========.', data: "0  1 2 3 0 1 2 3 4 0"},
  ],
  config: { hscale: 2 }  
}
*/

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
    
    // Zero every bin at sim time 0 so the RAM does not start with X.
    // The run-time scrub path (ram_scrub_i) will also zero this array;
    // that FSM is left as a placeholder for a later pass.
    initial begin
        for (int i = 0; i < 256; i++)
            hist_mem[i] = '0;
    end

    // -------------------------------------------------------------------------
    // Read-side FSM: pop one beat, process each byte in turn
    // -------------------------------------------------------------------------
    typedef enum logic [1:0] {
        S_IDLE = 2'b00,   // waiting for FIFO data
        S_ACTIVE,
        S_SCRUB
    } state_t;

    state_t                  state,        next_state;
    logic [3:0]              pix_cnt_d,    pix_cnt_q;
    logic [RAM_WIDTH-1:0]    ram_rd_val,   pix_ram_wr_val_d, ram_wr_val_q, running_cnt_d, running_cnt_q;
    logic [STREAM_WIDTH-1:0] beat_shift_d, beat_shift_q;
    logic [7:0]              ram_wr_addr,  ram_rd_addr;
    logic                    ram_pix_wr_valid, pixel_d_valid;
    logic [7:0]              pixel_q, pixel_d; // current pixel and previous pixel
    logic                    hazard, hazard_q;
    assign pixel_d_valid = next_state != S_IDLE;
    assign hazard = pixel_d_valid && (pixel_d == pixel_q) && (running_cnt_q != '0);
    // assign ram_pix_wr_valid = ((state == S_ACTIVE) && (~hazard) && (running_cnt_q != '0));
    // assign pix_ram_wr_val_d = (state == S_ACTIVE) ? (hazard_q ? (ram_wr_val_q + 1) : (ram_rd_val + 1)) : '0; 
    assign hist_rdy_o = (state == S_IDLE);

    // Pop from FIFO only when idle (about to start a new beat)

    // ---- State registers (FF macro) -----------------------------------------
    `FF(state,          next_state,      S_IDLE, clk_i, rst_n)
    `FF(pix_cnt_q,      pix_cnt_d,       '0,     clk_i, rst_n)
    `FF(pixel_q,        pixel_d,         '0,     clk_i, rst_n)
    `FF(beat_shift_q,   beat_shift_d,    '0,     clk_i, rst_n)
    `FF(ram_wr_val_q,   pix_ram_wr_val_d,    '0,     clk_i, rst_n)
    `FF(running_cnt_q,  running_cnt_d,   '0,     clk_i, rst_n)
    `FF(hazard_q,       hazard,          '0,     clk_i, rst_n)

    // ---- RAM port A (read) + port B (write) - single always_ff block --------
    always_ff @(posedge clk_i) begin
        // Port A: read current bin value whenever we are about to write
        // (read is registered; result is available the cycle after S_READ)
        ram_rd_val <= hist_mem[ram_rd_addr];

        // Port B: write incremented value back on S_WRITE
        if (state == S_ACTIVE)begin
            if (ram_pix_wr_valid)
                hist_mem[ram_wr_addr] <= pix_ram_wr_val_d;
        end
        else if (state == S_SCRUB) begin
            hist_mem[ram_wr_addr] <= '0;
        end
    end

    // ---- Next-state logic ---------------------------------------------------
    always_comb begin
        next_state    = state;
        pix_cnt_d     = pix_cnt_q;
        fifo_m_ready  = 1'b0;

        ram_wr_addr   = 8'h0;
        
        pixel_d       = pixel_q;
        beat_shift_d  = beat_shift_q;
        running_cnt_d = running_cnt_q;
        ram_rd_addr = '0;

        pix_ram_wr_val_d = ram_wr_val_q;
        ram_pix_wr_valid = 1'b0;

        case (state)
            // Cold start case
            S_IDLE: begin
                running_cnt_d = '0;
                pix_cnt_d = '0;
                if (hist_en_i && fifo_m_valid) begin
                    fifo_m_ready = 1'b1;

                    beat_shift_d = fifo_m_data;
                    next_state = S_ACTIVE;
                end
                else if(ram_scrub_i) begin
                    next_state = S_SCRUB;
                end
            end

            S_ACTIVE: begin
                running_cnt_d = running_cnt_q + 1;
                beat_shift_d = {8'b0, beat_shift_q[31:8]};

                // Next Pixel
                pixel_d = beat_shift_q[7:0];
                ram_rd_addr = pixel_d;
                pix_cnt_d = pix_cnt_q + 1;

                ram_wr_addr = pixel_q;

                if(~hazard && running_cnt_q > '0)begin
                    ram_pix_wr_valid = 1;
                end 

                // Write collision
                if(hazard_q)begin
                    pix_ram_wr_val_d = ram_wr_val_q + 1;
                end else begin
                    pix_ram_wr_val_d = ram_rd_val + 1;
                end

                if((pix_cnt_q == 4'h3) && fifo_m_valid)begin
                    fifo_m_ready = 1'b1;
                    beat_shift_d = fifo_m_data;
                    pix_cnt_d = '0;
                end
                else if(pix_cnt_q == 4'h4)begin
                    pix_cnt_d = 4'h0; next_state = S_IDLE;
                end
            end // S_ACTIVE

            S_SCRUB: begin
                ram_wr_addr = running_cnt_q;
                running_cnt_d = running_cnt_q + 1;
                if(running_cnt_q == 8'hFF)begin
                    next_state = S_IDLE;
                end
            end

            default: next_state = S_IDLE;
        endcase
    end


endmodule