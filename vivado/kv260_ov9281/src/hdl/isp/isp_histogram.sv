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

    // Pixels are coming in this module at ~147 million pixels per second. 
    // So as long at clk > 147MHz and we process at 1 pixel/clk, there 
    // will be no stall

    // This module cannot process rates at 4 pixels per second, so adding a FIFO to buffer

    logic [RAM_WIDTH-1:0] hist_mem[0:255];

    // Port A: READ


    // Port B: WRITE

endmodule