`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 03/27/2026 04:52:10 PM
// Design Name: 
// Module Name: vid_8bpp_2px_to_rgb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module vid_8bpp_2px_to_rgb(
    input wire [15:0] y8x2_in,
    output wire [47:0] rgb_out
    );

    wire [7:0] pix0, pix1;

    assign pix0 = y8x2_in[7:0];
    assign pix1 = y8x2_in[15:8];

    assign rgb_out[7:0]   = pix0;
    assign rgb_out[15:8]  = pix0;
    assign rgb_out[23:16] = pix0;

    assign rgb_out[31:24] = pix1;
    assign rgb_out[39:32] = pix1;
    assign rgb_out[47:40] = pix1;
endmodule
