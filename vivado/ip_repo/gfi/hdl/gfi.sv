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
//   RECV : accept exactly hres/4 input beats of input row r into the line
//          buffers. s_ready high (accepting).
//   EMIT : then stream hres/4 output beats of ONE output row, reading the three
//          most-recent rows back. s_ready LOW (not accepting), so input and
//          output can never drift.
// Schedule (output row ro uses input rows ro-1, ro, ro+1; borders replicate):
//   recv row 0            -> no emit (priming, just store row 0)
//   recv row r (1..V-1)   -> EMIT output row r-1  (top replicate when r==1)
//   recv row V-1 (last)   -> EMIT output row V-2, then EMIT output row V-1 with
//                            the bottom border replicated.
// -> exactly (hres/4)*vres output beats/frame, m_sof on the first output beat,
//    m_eol on each output row's last beat. Throughput ~0.5 beat/cycle; at
//    200 MHz (~400 Mpx/s) that is >6x the camera rate, so it never bottlenecks
//    the best-effort blob snoop path -- and it leaves plenty of headroom to
//    PIPELINE the read/filter for timing.
//
// LINE BUFFERS = BLOCK RAM. For a 3x3 the horizontal window needs three words
// per row (columns c-1/c/c+1). BRAM has limited read ports, so each of the 3
// line buffers is REPLICATED x3 (bXl/bXc/bXr, written identically) giving one
// registered read port per neighbour -- all three window words per row are read
// in parallel from block RAM. One 32b word == one beat == 4 px.
//
// PIPELINE (EMIT), all stages gated by pipe_adv so backpressure stalls the whole
// thing losslessly:
//   S0  ecol -> read addresses a_l/a_c/a_r; metadata (roles, borders, sof/eol,
//       valid, enable/strength) computed for this column.
//   S1  BRAM registered reads (9 words) + metadata reg.
//   S2  role mux -> 9 window words registered + metadata reg.
//   S3  border mux + weighted sum + round -> m_data (+ sof/eol/valid).
// So m_data lags the read address by 3 cycles; the metadata pipe keeps sof/eol/
// valid aligned. Pixel packing: s_data[7:0]=px0 (leftmost)..[31:24]=px3.
//
// Bypass (enable=0): output == the centre (mid) pixel, out of the same pipeline
// so latency never changes. enable/strength latched at s_sof, held per frame.
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

    // 3 line buffers, each replicated x3 (left/centre/right read port). BRAM.
    (* ram_style = "block" *) logic [31:0] b0l [0:BEATS_MAX-1];
    (* ram_style = "block" *) logic [31:0] b0c [0:BEATS_MAX-1];
    (* ram_style = "block" *) logic [31:0] b0r [0:BEATS_MAX-1];
    (* ram_style = "block" *) logic [31:0] b1l [0:BEATS_MAX-1];
    (* ram_style = "block" *) logic [31:0] b1c [0:BEATS_MAX-1];
    (* ram_style = "block" *) logic [31:0] b1r [0:BEATS_MAX-1];
    (* ram_style = "block" *) logic [31:0] b2l [0:BEATS_MAX-1];
    (* ram_style = "block" *) logic [31:0] b2c [0:BEATS_MAX-1];
    (* ram_style = "block" *) logic [31:0] b2r [0:BEATS_MAX-1];

    wire [15:0] hbeats = {2'b0, hres[15:2]};   // hres/4 (hres is a mult of 4)

    // =========================================================================
    // Control state (FSM)
    // =========================================================================
    localparam logic S_RECV = 1'b0, S_EMIT = 1'b1;
    logic        state;

    logic [15:0] row_idx;      // input row currently being received (0-based)
    logic [15:0] rcol;         // beat index within the receiving row
    logic [1:0]  wsel;         // buffer holding the row being received (0/1/2)

    logic [15:0] ecol;         // output beat index (read-address stage) during EMIT
    logic        emit_bottom;  // this EMIT is the trailing bottom-replicate row

    logic        frame_en;     // latched enable for the current frame
    logic [1:0]  frame_str;    // latched strength for the current frame

    function automatic logic [1:0] nsel(input logic [1:0] s);
        nsel = (s == 2'd2) ? 2'd0 : (s + 2'd1);
    endfunction

    // =========================================================================
    // S0: per-column metadata (roles / borders / sof-eol / valid / cfg)
    // =========================================================================
    logic [15:0] emit_row;
    logic [1:0]  top_buf, mid_buf, bot_buf;
    logic        top_rep, bot_rep;

    always_comb begin
        emit_row = emit_bottom ? row_idx : (row_idx - 16'd1);
        if (emit_bottom) begin
            mid_buf = wsel;
            top_buf = (wsel == 2'd0) ? 2'd2 : (wsel - 2'd1);   // (wsel+2)%3
            bot_buf = wsel;
            top_rep = 1'b0;
            bot_rep = 1'b1;
        end else begin
            bot_buf = wsel;                                    // row r
            mid_buf = (wsel == 2'd0) ? 2'd2 : (wsel - 2'd1);   // (wsel+2)%3 -> r-1
            top_buf = (wsel == 2'd2) ? 2'd0 : (wsel + 2'd1);   // (wsel+1)%3 -> r-2
            top_rep = (emit_row == 16'd0);                     // top border
            bot_rep = 1'b0;
        end
    end

    wire [1:0] eff_top = top_rep ? mid_buf : top_buf;
    wire [1:0] eff_bot = bot_rep ? mid_buf : bot_buf;

    // read addresses (clamped) for this column
    wire [AW-1:0] a_c = ecol[AW-1:0];
    wire [AW-1:0] a_l = (ecol == 16'd0)            ? ecol[AW-1:0] : (ecol[AW-1:0] - 1'b1);
    wire [AW-1:0] a_r = (ecol == (hbeats - 16'd1)) ? ecol[AW-1:0] : (ecol[AW-1:0] + 1'b1);

    // Metadata carried down the pipeline so sof/eol/roles/borders stay aligned.
    typedef struct packed {
        logic        valid;
        logic        sof;
        logic        eol;
        logic        col_first;
        logic        col_last;
        logic [1:0]  eff_top;
        logic [1:0]  mid_buf;
        logic [1:0]  eff_bot;
        logic        en;
        logic [1:0]  str;
    } meta_t;

    meta_t m0, m1, m2;
    always_comb begin
        m0.valid     = (state == S_EMIT) && (ecol < hbeats);
        m0.sof       = (emit_row == 16'd0) && (ecol == 16'd0) && !emit_bottom;
        m0.eol       = (ecol == (hbeats - 16'd1));
        m0.col_first = (ecol == 16'd0);
        m0.col_last  = (ecol == (hbeats - 16'd1));
        m0.eff_top   = eff_top;
        m0.mid_buf   = mid_buf;
        m0.eff_bot   = eff_bot;
        m0.en        = frame_en;
        m0.str       = frame_str;
    end

    // =========================================================================
    // Handshake / pipeline advance
    // =========================================================================
    assign s_ready = (state == S_RECV);
    wire   in_fire  = s_valid && s_ready;
    wire   pipe_adv = (state == S_EMIT) && (!m_valid || m_ready);
    // last output beat of the row leaves the pipe when the eol metadata reaches S3
    wire   emit_row_done = pipe_adv && m2.valid && m2.eol;

    // =========================================================================
    // Line-buffer write (RECV): write all 3 replicas of the target buffer.
    // Registered reads (S1): one read port per neighbour, gated by pipe_adv.
    // =========================================================================
    logic [31:0] q0l,q0c,q0r, q1l,q1c,q1r, q2l,q2c,q2r;

    always_ff @(posedge clk) begin
        if (in_fire) begin
            case (wsel)
                2'd0: begin b0l[rcol[AW-1:0]]<=s_data; b0c[rcol[AW-1:0]]<=s_data; b0r[rcol[AW-1:0]]<=s_data; end
                2'd1: begin b1l[rcol[AW-1:0]]<=s_data; b1c[rcol[AW-1:0]]<=s_data; b1r[rcol[AW-1:0]]<=s_data; end
                default: begin b2l[rcol[AW-1:0]]<=s_data; b2c[rcol[AW-1:0]]<=s_data; b2r[rcol[AW-1:0]]<=s_data; end
            endcase
        end
        if (pipe_adv) begin
            q0l<=b0l[a_l]; q0c<=b0c[a_c]; q0r<=b0r[a_r];
            q1l<=b1l[a_l]; q1c<=b1c[a_c]; q1r<=b1r[a_r];
            q2l<=b2l[a_l]; q2c<=b2c[a_c]; q2r<=b2r[a_r];
        end
    end

    // S2: role mux of the registered reads -> 9 window words, registered.
    wire [31:0] tl_c = (m1.eff_top==2'd0)?q0l:(m1.eff_top==2'd1)?q1l:q2l;
    wire [31:0] tc_c = (m1.eff_top==2'd0)?q0c:(m1.eff_top==2'd1)?q1c:q2c;
    wire [31:0] tr_c = (m1.eff_top==2'd0)?q0r:(m1.eff_top==2'd1)?q1r:q2r;
    wire [31:0] ml_c = (m1.mid_buf==2'd0)?q0l:(m1.mid_buf==2'd1)?q1l:q2l;
    wire [31:0] mc_c = (m1.mid_buf==2'd0)?q0c:(m1.mid_buf==2'd1)?q1c:q2c;
    wire [31:0] mr_c = (m1.mid_buf==2'd0)?q0r:(m1.mid_buf==2'd1)?q1r:q2r;
    wire [31:0] bl_c = (m1.eff_bot==2'd0)?q0l:(m1.eff_bot==2'd1)?q1l:q2l;
    wire [31:0] bc_c = (m1.eff_bot==2'd0)?q0c:(m1.eff_bot==2'd1)?q1c:q2c;
    wire [31:0] br_c = (m1.eff_bot==2'd0)?q0r:(m1.eff_bot==2'd1)?q1r:q2r;

    logic [31:0] tl,tc,tr, ml,mc,mr, bl,bc,br;

    // metadata + window pipeline registers
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            m1.valid <= 1'b0; m2.valid <= 1'b0;
        end else begin
            // Flush the pipeline valids outside EMIT so a new EMIT starts clean.
            if (state != S_EMIT) begin
                m1.valid <= 1'b0; m2.valid <= 1'b0;
            end else if (pipe_adv) begin
                m1 <= m0;
                m2 <= m1;
            end
        end
        if (pipe_adv) begin
            tl<=tl_c; tc<=tc_c; tr<=tr_c;
            ml<=ml_c; mc<=mc_c; mr<=mr_c;
            bl<=bl_c; bc<=bc_c; br<=br_c;
        end
    end

    // =========================================================================
    // S3: 3x3 window (border replicate) + weighted sum (shift-add) -> out_word
    // =========================================================================
    function automatic logic [7:0] bof(input logic [31:0] w, input int idx);
        bof = w[8*idx +: 8];
    endfunction

    logic [7:0] out_px [0:3];
    always_comb begin
        for (int i = 0; i < 4; i++) begin
            automatic logic [7:0] t_l,t_c,t_r, m_l,m_c,m_r, b_l,b_c,b_r;
            t_c = bof(tc, i);  m_c = bof(mc, i);  b_c = bof(bc, i);
            if (i == 0) begin
                t_l = m2.col_first ? bof(tc,0) : bof(tl,3);
                m_l = m2.col_first ? bof(mc,0) : bof(ml,3);
                b_l = m2.col_first ? bof(bc,0) : bof(bl,3);
            end else begin
                t_l = bof(tc,i-1); m_l = bof(mc,i-1); b_l = bof(bc,i-1);
            end
            if (i == 3) begin
                t_r = m2.col_last ? bof(tc,3) : bof(tr,0);
                m_r = m2.col_last ? bof(mc,3) : bof(mr,0);
                b_r = m2.col_last ? bof(bc,3) : bof(br,0);
            end else begin
                t_r = bof(tc,i+1); m_r = bof(mc,i+1); b_r = bof(bc,i+1);
            end
            begin
                automatic logic [10:0] sum4, corners;
                automatic logic [12:0] acc;
                sum4    = {3'b0,t_c} + {3'b0,m_l} + {3'b0,m_r} + {3'b0,b_c};
                corners = {3'b0,t_l} + {3'b0,t_r} + {3'b0,b_l} + {3'b0,b_r};
                unique case (m2.str)
                    2'd0:    acc = {2'b0,sum4} + ({5'b0,m_c}<<3) + ({5'b0,m_c}<<2); // light
                    2'd1:    acc = ({2'b0,sum4}<<1) + {2'b0,corners} + ({5'b0,m_c}<<2); // medium
                    2'd2:    acc = ({2'b0,sum4} + {2'b0,corners})<<1;                 // strong
                    default: acc = {2'b0,sum4} + ({5'b0,m_c}<<3) + ({5'b0,m_c}<<2);
                endcase
                out_px[i] = m2.en ? ((acc + 13'd8) >> 4) : m_c;   // bypass = centre
            end
        end
    end
    wire [31:0] out_word = {out_px[3], out_px[2], out_px[1], out_px[0]};

    // =========================================================================
    // FSM
    // =========================================================================
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            state <= S_RECV; row_idx <= 16'd0; rcol <= 16'd0; wsel <= 2'd0;
            ecol <= 16'd0; emit_bottom <= 1'b0; frame_en <= 1'b0; frame_str <= 2'd0;
        end else begin
            case (state)
                S_RECV: begin
                    if (in_fire) begin
                        if (s_sof) begin
                            row_idx   <= 16'd0;
                            frame_en  <= enable;
                            frame_str <= strength;
                        end
                        if (s_eol) begin
                            rcol <= 16'd0;
                            if ((s_sof ? 16'd0 : row_idx) == 16'd0) begin
                                row_idx <= 16'd1;          // row 0 stored, no emit
                                wsel    <= nsel(wsel);
                            end else begin
                                state       <= S_EMIT;     // emit output row r-1
                                ecol        <= 16'd0;
                                emit_bottom <= 1'b0;
                            end
                        end else begin
                            rcol <= rcol + 16'd1;
                        end
                    end
                end
                S_EMIT: begin
                    if (pipe_adv && (ecol < hbeats)) ecol <= ecol + 16'd1;
                    if (emit_row_done) begin
                        if (!emit_bottom && (row_idx == (vres - 16'd1))) begin
                            emit_bottom <= 1'b1;           // trailing bottom row
                            ecol        <= 16'd0;
                        end else begin
                            state       <= S_RECV;
                            emit_bottom <= 1'b0;
                            ecol        <= 16'd0;
                            row_idx     <= row_idx + 16'd1;
                            wsel        <= nsel(wsel);
                        end
                    end
                end
            endcase
        end
    end

    // =========================================================================
    // Output register (single-beat, lossless): load from S3 on pipe_adv.
    // =========================================================================
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            m_valid <= 1'b0; m_data <= 32'd0; m_sof <= 1'b0; m_eol <= 1'b0;
        end else if (pipe_adv) begin
            m_valid <= m2.valid;
            m_data  <= out_word;
            m_sof   <= m2.sof;
            m_eol   <= m2.eol;
        end else if (m_valid && m_ready) begin
            m_valid <= 1'b0;
        end
    end

endmodule
