// Clauded

`timescale 1ns / 1ps
module stream_fifo #(
    // Payload type carried through the FIFO. Defaults to a plain DATA_WIDTH-bit
    // vector so existing width-based instantiations (isp_histogram, blob core)
    // work unchanged. Pass a packed struct (e.g. an AXIS {tuser,tlast,tdata}
    // payload) via .T(...) to buffer the WHOLE beat type-safely -- the sideband
    // signals are stored and replayed verbatim, never re-sliced or regenerated.
    parameter int  DATA_WIDTH = 8,
    parameter type T          = logic [DATA_WIDTH-1:0],
    parameter int  DEPTH      = 16  // must be a power of 2
) (
    input  logic clk,
    input  logic rst_n,

    // Slave (input) port
    input  logic s_valid,
    output logic s_ready,
    input  T     s_data,

    // Master (output) port
    output logic m_valid,
    input  logic m_ready,
    output T     m_data,

    output logic empty
);

    localparam int PTR_WIDTH = $clog2(DEPTH);

    T                     mem [0:DEPTH-1];
    logic [PTR_WIDTH-1:0] wr_ptr, rd_ptr;
    logic [PTR_WIDTH:0]   count;  // one extra bit to distinguish full from empty

    wire push = s_valid && s_ready;
    wire pop  = m_valid && m_ready;

    assign s_ready = (count < DEPTH);
    assign m_valid = (count > 0);
    // Gate the payload with validity. mem[rd_ptr] retains the last beat written
    // to that slot, so when the FIFO is empty the combinational read would still
    // expose a stale beat -- and after DEPTH pops rd_ptr wraps back onto a slot
    // whose retained beat had tuser(SOF)/tlast(EOL) set, making those sideband
    // bits glitch on the bus while tvalid is low. Legal per AXIS (payload is
    // don't-care when !tvalid) but it pollutes the bus/ILA; drive zero when idle.
    assign m_data  = m_valid ? mem[rd_ptr] : '0;
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
