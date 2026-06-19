// Clauded

`timescale 1ns / 1ps
module stream_fifo #(
    parameter int DATA_WIDTH = 8,
    parameter int DEPTH      = 16  // must be a power of 2
) (
    input  logic                  clk,
    input  logic                  rst_n,

    // Slave (input) port
    input  logic                  s_valid,
    output logic                  s_ready,
    input  logic [DATA_WIDTH-1:0] s_data,

    // Master (output) port
    output logic                  m_valid,
    input  logic                  m_ready,
    output logic [DATA_WIDTH-1:0] m_data,

    output logic empty
);

    localparam int PTR_WIDTH = $clog2(DEPTH);

    logic [DATA_WIDTH-1:0] mem [0:DEPTH-1];
    logic [PTR_WIDTH-1:0]  wr_ptr, rd_ptr;
    logic [PTR_WIDTH:0]    count;  // one extra bit to distinguish full from empty

    wire push = s_valid && s_ready;
    wire pop  = m_valid && m_ready;

    assign s_ready = (count < DEPTH);
    assign m_valid = (count > 0);
    assign m_data  = mem[rd_ptr];
    assign empty   = (count == '0);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr <= '0;
            rd_ptr <= '0;
            count  <= '0;
        end else begin
            if (push) begin
                mem[wr_ptr] <= s_data;
                wr_ptr      <= wr_ptr + 1'b1;
            end
            if (pop)
                rd_ptr <= rd_ptr + 1'b1;
            case ({push, pop})
                2'b10:   count <= count + 1'b1;
                2'b01:   count <= count - 1'b1;
                default: count <= count;
            endcase
        end
    end

endmodule
