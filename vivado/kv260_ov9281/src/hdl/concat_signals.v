module expand_8_to_12 (
    input  wire [7:0]  data_in,
    output wire [11:0] data_out
);

    assign data_out = {data_in, 4'b0000};

endmodule