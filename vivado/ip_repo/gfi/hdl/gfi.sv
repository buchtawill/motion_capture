`timescale 1ns / 1ps
// =============================================================================
// gfi.sv -- Gaussian For Image: streaming 3x3 spatial pre-filter
//
// Bit-exact to vivado/ip_repo/gfi/model/gfi_model.py (kernels / rounding /
// border rules). Pure dataflow stage, no configuration registers -- enable,
// strength, hres, vres arrive as wires.
//
// ARCHITECTURE: serial receive/emit (no concurrent recv+emit -> no lockstep
// hazards). One row at a time:
//   RECV  : accept exactly hres/4 input beats of input row r into a line
//           buffer. s_ready high (we are accepting).
//   EMIT  : then stream hres/4 output beats of ONE output row, reading the
//           three most-recent rows back from the buffers. s_ready LOW (we are
//           not accepting), so input and output can never drift.
// Schedule (output row ro uses input rows ro-1, ro, ro+1; borders replicate):
//   recv row 0            -> no emit (priming, just store row 0)
//   recv row r (1..V-1)   -> EMIT output row r-1  (top replicate when r==1)
//   recv row V-1 (last)   -> EMIT output row V-2, then EMIT output row V-1
//                            with the bottom border replicated.
// -> exactly (hres/4)*vres output beats/frame, m_sof on the first output beat,
//    m_eol on each output row's last beat. Throughput ~0.5 beat/cycle; at
//    200 MHz (~400 Mpx/s) that is >6x the camera rate, so it never bottlenecks
//    the best-effort blob snoop path.
//
// Line buffers: 3 x (MAX_WIDTH/4) x 32b, ASYNC read (distributed RAM) so the
// read latency is identical in simulation and synthesis (a registered BRAM
// read would add a cycle in synth only). One 32b word == one beat == 4 px.
//
// Pixel packing: s_data[7:0]=px0 (leftmost) .. s_data[31:24]=px3 (rightmost).
// Horizontal 3x3 crosses beat boundaries: output pixel column 4c-1 is the prev
// beat's px3, 4c+4 is the next beat's px0 -- handled by reading words c-1/c/c+1
// (address-clamped) and picking the right byte; column 0 / last column
// replicate their own edge pixel.
//
// Bypass (enable=0): output == the centre (mid) pixel, out of the same pipeline
// so latency never changes. enable/strength are latched at the s_sof beat and
// held for the whole frame.
// =============================================================================

module gfi #(
    parameter int MAX_WIDTH   = 2048,   // line-buffer width (>= max hres)
    parameter int PX_PER_BEAT = 4       // fixed by the 32-bit data format
) (
    input  logic        clk,
    input  logic        rst_n,

    // config wires (no internal registers)
    input  logic        enable,          // 0 = bypass (identity, same latency)
    input  logic [1:0]  strength,        // 0=light 1=medium 2=strong
    input  logic [15:0] hres,            // active width in pixels (mult of 4)
    input  logic [15:0] vres,            // active height in rows

    // slave (input)
    input  logic        s_valid,
    output logic        s_ready,
    input  logic [31:0] s_data,          // 4 px LSB-first
    input  logic        s_sof,           // start of frame (first beat)
    input  logic        s_eol,           // end of line (last beat of a row)

    // master (output)
    output logic        m_valid,
    input  logic        m_ready,
    output logic [31:0] m_data,
    output logic        m_sof,
    output logic        m_eol
);

    localparam int BEATS_MAX = MAX_WIDTH / PX_PER_BEAT;
    localparam int AW        = $clog2(BEATS_MAX);

    // Three line buffers, async (distributed) read.
    (* ram_style = "distributed" *) logic [31:0] lb0 [0:BEATS_MAX-1];
    (* ram_style = "distributed" *) logic [31:0] lb1 [0:BEATS_MAX-1];
    (* ram_style = "distributed" *) logic [31:0] lb2 [0:BEATS_MAX-1];

    wire [15:0] hbeats = {2'b0, hres[15:2]};   // hres/4 (hres is a mult of 4)

    // =========================================================================
    // Control state
    // =========================================================================
    localparam logic S_RECV = 1'b0, S_EMIT = 1'b1;
    logic        state;

    logic [15:0] row_idx;      // input row currently being received (0-based)
    logic [15:0] rcol;         // beat index within the receiving row
    logic [1:0]  wsel;         // buffer holding the row being received (0/1/2)

    logic [15:0] ecol;         // output beat index during EMIT
    logic        emit_bottom;  // this EMIT is the trailing bottom-replicate row

    logic        frame_en;     // latched enable for the current frame
    logic [1:0]  frame_str;    // latched strength for the current frame

    // next-buffer helper (mod 3)
    function automatic logic [1:0] nsel(input logic [1:0] s);
        nsel = (s == 2'd2) ? 2'd0 : (s + 2'd1);
    endfunction

    // =========================================================================
    // EMIT buffer-role selection (combinational)
    //   normal emit  -> output row = row_idx-1, uses rows r-2/r-1/r =
    //                   bufs (wsel+1)/(wsel+2)/(wsel) ; top replicates when
    //                   the output row is 0.
    //   bottom emit  -> output row = row_idx (== vres-1), uses rows r-1/r/(r) ;
    //                   top = (wsel+2), mid = wsel, bottom replicates mid.
    // =========================================================================
    logic [15:0] emit_row;
    logic [1:0]  top_buf, mid_buf, bot_buf;
    logic        top_rep, bot_rep;

    always_comb begin
        emit_row = emit_bottom ? row_idx : (row_idx - 16'd1);
        if (emit_bottom) begin
            mid_buf = wsel;
            top_buf = (wsel == 2'd0) ? 2'd2 : (wsel - 2'd1);   // (wsel+2)%3
            bot_buf = wsel;                                    // (unused: replicated)
            top_rep = 1'b0;
            bot_rep = 1'b1;
        end else begin
            bot_buf = wsel;                                    // row r
            mid_buf = (wsel == 2'd0) ? 2'd2 : (wsel - 2'd1);   // (wsel+2)%3 -> row r-1
            top_buf = (wsel == 2'd2) ? 2'd0 : (wsel + 2'd1);   // (wsel+1)%3 -> row r-2
            top_rep = (emit_row == 16'd0);                     // top border
            bot_rep = 1'b0;
        end
    end

    // Effective buffers after vertical-border replication.
    wire [1:0] eff_top = top_rep ? mid_buf : top_buf;
    wire [1:0] eff_bot = bot_rep ? mid_buf : bot_buf;

    // Column addresses (clamped) for the horizontal window.
    wire [AW-1:0] a_c = ecol[AW-1:0];
    wire [AW-1:0] a_l = (ecol == 16'd0)            ? ecol[AW-1:0] : (ecol[AW-1:0] - 1'b1);
    wire [AW-1:0] a_r = (ecol == (hbeats - 16'd1)) ? ecol[AW-1:0] : (ecol[AW-1:0] + 1'b1);
    wire          col_first = (ecol == 16'd0);
    wire          col_last  = (ecol == (hbeats - 16'd1));

    // Direct async reads of each buffer at each of the 3 column addresses. Must
    // be direct mem[addr] indexing in the continuous assigns (NOT wrapped in a
    // function) so the read stays sensitive to line-buffer writes.
    wire [31:0] w0_l = lb0[a_l], w0_c = lb0[a_c], w0_r = lb0[a_r];
    wire [31:0] w1_l = lb1[a_l], w1_c = lb1[a_c], w1_r = lb1[a_r];
    wire [31:0] w2_l = lb2[a_l], w2_c = lb2[a_c], w2_r = lb2[a_r];

    // Three words per row (left/centre/right beat), muxed by the effective role.
    wire [31:0] tl = (eff_top == 2'd0) ? w0_l : (eff_top == 2'd1) ? w1_l : w2_l;
    wire [31:0] tc = (eff_top == 2'd0) ? w0_c : (eff_top == 2'd1) ? w1_c : w2_c;
    wire [31:0] tr = (eff_top == 2'd0) ? w0_r : (eff_top == 2'd1) ? w1_r : w2_r;
    wire [31:0] ml = (mid_buf == 2'd0) ? w0_l : (mid_buf == 2'd1) ? w1_l : w2_l;
    wire [31:0] mc = (mid_buf == 2'd0) ? w0_c : (mid_buf == 2'd1) ? w1_c : w2_c;
    wire [31:0] mr = (mid_buf == 2'd0) ? w0_r : (mid_buf == 2'd1) ? w1_r : w2_r;
    wire [31:0] bl = (eff_bot == 2'd0) ? w0_l : (eff_bot == 2'd1) ? w1_l : w2_l;
    wire [31:0] bc = (eff_bot == 2'd0) ? w0_c : (eff_bot == 2'd1) ? w1_c : w2_c;
    wire [31:0] br = (eff_bot == 2'd0) ? w0_r : (eff_bot == 2'd1) ? w1_r : w2_r;

    function automatic logic [7:0] bof(input logic [31:0] w, input int idx);
        bof = w[8*idx +: 8];
    endfunction

    // =========================================================================
    // Per-output-pixel 3x3 window + weighted sum (shift-add, bit-exact to model)
    // =========================================================================
    logic [7:0] out_px [0:3];

    always_comb begin
        for (int i = 0; i < 4; i++) begin
            automatic logic [7:0] t_l, t_c, t_r, m_l, m_c, m_r, b_l, b_c, b_r;

            // centre column of this output pixel
            t_c = bof(tc, i);  m_c = bof(mc, i);  b_c = bof(bc, i);

            // left neighbour (pixel column 4c+i-1)
            if (i == 0) begin
                t_l = col_first ? bof(tc, 0) : bof(tl, 3);
                m_l = col_first ? bof(mc, 0) : bof(ml, 3);
                b_l = col_first ? bof(bc, 0) : bof(bl, 3);
            end else begin
                t_l = bof(tc, i - 1);  m_l = bof(mc, i - 1);  b_l = bof(bc, i - 1);
            end

            // right neighbour (pixel column 4c+i+1)
            if (i == 3) begin
                t_r = col_last ? bof(tc, 3) : bof(tr, 0);
                m_r = col_last ? bof(mc, 3) : bof(mr, 0);
                b_r = col_last ? bof(bc, 3) : bof(br, 0);
            end else begin
                t_r = bof(tc, i + 1);  m_r = bof(mc, i + 1);  b_r = bof(bc, i + 1);
            end

            begin
                automatic logic [10:0] sum4;     // edge neighbours: TC+ML+MR+BC
                automatic logic [10:0] corners;  // TL+TR+BL+BR
                automatic logic [12:0] acc;      // weighted sum, max 16*255=4080

                sum4    = {3'b0, t_c} + {3'b0, m_l} + {3'b0, m_r} + {3'b0, b_c};
                corners = {3'b0, t_l} + {3'b0, t_r} + {3'b0, b_l} + {3'b0, b_r};

                unique case (frame_str)
                    2'd0:    acc = {2'b0, sum4} + ({5'b0, m_c} << 3) + ({5'b0, m_c} << 2); // light  (+12*mc)
                    2'd1:    acc = ({2'b0, sum4} << 1) + {2'b0, corners} + ({5'b0, m_c} << 2); // medium
                    2'd2:    acc = ({2'b0, sum4} + {2'b0, corners}) << 1;                     // strong
                    default: acc = {2'b0, sum4} + ({5'b0, m_c} << 3) + ({5'b0, m_c} << 2);
                endcase

                out_px[i] = frame_en ? ((acc + 13'd8) >> 4) : m_c;   // bypass = centre px
            end
        end
    end

    wire [31:0] out_word = {out_px[3], out_px[2], out_px[1], out_px[0]};

    // =========================================================================
    // Handshake
    // =========================================================================
    assign s_ready = (state == S_RECV);
    wire   in_fire  = s_valid && s_ready;
    wire   emit_adv = (state == S_EMIT) && (!m_valid || m_ready);
    wire   emit_last_beat = emit_adv && (ecol == (hbeats - 16'd1));

    // Was the row we just received the last one of the frame?
    wire   recv_row_last = in_fire && s_eol && (row_idx == (vres - 16'd1));

    // =========================================================================
    // Line-buffer write (RECV only)
    // =========================================================================
    always_ff @(posedge clk) begin
        if (in_fire) begin
            case (wsel)
                2'd0:    lb0[rcol[AW-1:0]] <= s_data;
                2'd1:    lb1[rcol[AW-1:0]] <= s_data;
                default: lb2[rcol[AW-1:0]] <= s_data;
            endcase
        end
    end

    // =========================================================================
    // Control FSM
    // =========================================================================
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state       <= S_RECV;
            row_idx     <= 16'd0;
            rcol        <= 16'd0;
            wsel        <= 2'd0;
            ecol        <= 16'd0;
            emit_bottom <= 1'b0;
            frame_en    <= 1'b0;
            frame_str   <= 2'd0;
        end else begin
            case (state)
                // -------------------------------------------------------------
                S_RECV: begin
                    if (in_fire) begin
                        // sof beat: (re)start the frame at row 0, latch config
                        if (s_sof) begin
                            row_idx   <= 16'd0;
                            frame_en  <= enable;
                            frame_str <= strength;
                        end
                        if (s_eol) begin
                            rcol <= 16'd0;
                            if ((s_sof ? 16'd0 : row_idx) == 16'd0) begin
                                // row 0 complete: no emit, go receive row 1
                                row_idx <= 16'd1;
                                wsel    <= nsel(wsel);
                            end else begin
                                // rows 1..V-1: emit the previous output row
                                state       <= S_EMIT;
                                ecol        <= 16'd0;
                                emit_bottom <= 1'b0;
                            end
                        end else begin
                            rcol <= rcol + 16'd1;
                        end
                    end
                end
                // -------------------------------------------------------------
                S_EMIT: begin
                    if (emit_adv) ecol <= ecol + 16'd1;
                    if (emit_last_beat) begin
                        if (!emit_bottom && (row_idx == (vres - 16'd1))) begin
                            // last input row: do the trailing bottom-replicate row
                            emit_bottom <= 1'b1;
                            ecol        <= 16'd0;
                        end else begin
                            // this row's emit(s) done -> receive next input row
                            state       <= S_RECV;
                            emit_bottom <= 1'b0;
                            ecol        <= 16'd0;
                            row_idx     <= row_idx + 16'd1;   // reset to 0 by next sof
                            wsel        <= nsel(wsel);
                        end
                    end
                end
            endcase
        end
    end

    // =========================================================================
    // Output register (single-beat, lossless: load only when !m_valid||m_ready)
    // =========================================================================
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            m_valid <= 1'b0;
            m_data  <= 32'd0;
            m_sof   <= 1'b0;
            m_eol   <= 1'b0;
        end else if (emit_adv) begin
            m_valid <= 1'b1;
            m_data  <= out_word;
            m_sof   <= (emit_row == 16'd0) && (ecol == 16'd0) && !emit_bottom;
            m_eol   <= (ecol == (hbeats - 16'd1));
        end else if (m_valid && m_ready) begin
            m_valid <= 1'b0;
        end
    end

endmodule
