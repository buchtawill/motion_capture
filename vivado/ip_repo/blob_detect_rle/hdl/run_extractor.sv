`timescale 1ns / 1ps

module run_extractor (
    input  logic        clk,
    input  logic        rst_n,

    input  logic        enable,
    input  logic        clear,
    input  logic [7:0]  threshold,
    input  logic [15:0] hres,
    input  logic [15:0] vres,

    // AXIS pixel input (32b = 4 x 8-bit pixels, LSB-first)
    input  logic [31:0] s_data,
    input  logic        s_valid,
    output logic        s_ready,

    // Run output (valid/ready handshake)
    output logic [15:0] run_xs,
    output logic [15:0] run_xe,
    output logic [15:0] run_row,
    output logic        run_last_in_row,
    output logic        run_valid,
    input  logic        run_ready,

    output logic        frame_done
);

    // =========================================================================
    // FSM
    // =========================================================================
    typedef enum logic [1:0] {
        RE_IDLE   = 2'd0,
        RE_ACTIVE = 2'd1
    } re_state_t;

    re_state_t state, state_next;

    // =========================================================================
    // Pixel position tracking
    // =========================================================================
    logic [15:0] pix_col;
    logic [15:0] pix_row;

    // =========================================================================
    // Run tracking (persists across beats)
    // =========================================================================
    logic        in_run;
    logic [15:0] cur_run_xs;

    // =========================================================================
    // Output skid buffer (up to 2 runs can be produced per beat)
    // =========================================================================
    // Entry format: {last_in_row[49], row[48:33], xe[32:17], xs[16:1], valid[0]}
    logic [49:0] out_buf [0:1];
    logic [1:0]  out_wr_count;
    logic [1:0]  out_rd_count;
    logic        out_rd_ptr;

    wire out_empty = (out_rd_count == 2'd0);

    // =========================================================================
    // Combinational: threshold + run detection within 4-pixel beat
    // =========================================================================
    logic [7:0]  px [0:3];
    logic [3:0]  fg;
    logic [15:0] abs_col [0:3];
    logic [3:0]  valid_px;

    logic [49:0] new_buf [0:1];
    logic [1:0]  new_count;
    logic        next_in_run;
    logic [15:0] next_run_xs;
    logic        is_last_beat;
    logic        beat_go;

    always_comb begin
        // Unpack pixels
        px[0] = s_data[ 7: 0];
        px[1] = s_data[15: 8];
        px[2] = s_data[23:16];
        px[3] = s_data[31:24];

        for (int i = 0; i < 4; i++) begin
            abs_col[i] = pix_col + 16'(i);
            valid_px[i] = (abs_col[i] < hres);
            fg[i] = valid_px[i] && (px[i] >= threshold);
        end

        is_last_beat = (pix_col + 16'd4 >= hres);

        new_count   = 2'd0;
        new_buf[0]  = '0;
        new_buf[1]  = '0;
        next_in_run = in_run;
        next_run_xs = cur_run_xs;

        beat_go = (state == RE_ACTIVE) && s_valid && s_ready;

        if (beat_go) begin
            for (int i = 0; i < 4; i++) begin
                if (valid_px[i]) begin
                    if (fg[i] && !next_in_run) begin
                        next_in_run = 1'b1;
                        next_run_xs = abs_col[i];
                    end else if (!fg[i] && next_in_run) begin
                        next_in_run = 1'b0;
                        if (new_count < 2'd2) begin
                            new_buf[new_count] = {1'b0, pix_row, abs_col[i] - 16'd1, next_run_xs};
                            new_count = new_count + 2'd1;
                        end
                    end
                end
            end

            // End of row: flush open run
            if (is_last_beat && next_in_run) begin
                if (new_count < 2'd2) begin
                    new_buf[new_count] = {1'b0, pix_row, hres - 16'd1, next_run_xs};
                    new_count = new_count + 2'd1;
                end
                next_in_run = 1'b0;
            end

            // Mark the last run in this row
            if (is_last_beat && new_count > 2'd0) begin
                new_buf[new_count - 2'd1][49] = 1'b1;
            end
        end
    end

    // =========================================================================
    // Output mux
    // =========================================================================
    always_comb begin
        run_valid = !out_empty;
        {run_last_in_row, run_row, run_xe, run_xs} = out_buf[out_rd_ptr];
    end

    // Accept new beat when output buffer is drained
    assign s_ready = (state == RE_ACTIVE) && out_empty && enable;

    // =========================================================================
    // Next-state logic
    // =========================================================================
    always_comb begin
        state_next = state;
        case (state)
            RE_IDLE: begin
                if (enable && !clear)
                    state_next = RE_ACTIVE;
            end
            RE_ACTIVE: begin
                if (clear)
                    state_next = RE_IDLE;
            end
            default: state_next = RE_IDLE;
        endcase
    end

    // =========================================================================
    // Sequential logic
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= RE_IDLE;
            pix_col     <= '0;
            pix_row     <= '0;
            in_run      <= 1'b0;
            cur_run_xs  <= '0;
            out_wr_count <= '0;
            out_rd_count <= '0;
            out_rd_ptr  <= 1'b0;
            frame_done  <= 1'b0;
            out_buf[0]  <= '0;
            out_buf[1]  <= '0;
        end else begin
            frame_done <= 1'b0;
            state      <= state_next;

            if (clear) begin
                pix_col     <= '0;
                pix_row     <= '0;
                in_run      <= 1'b0;
                cur_run_xs  <= '0;
                out_wr_count <= '0;
                out_rd_count <= '0;
                out_rd_ptr  <= 1'b0;
            end else begin
                // Drain output buffer on handshake
                if (run_valid && run_ready) begin
                    if (out_rd_count == 2'd1) begin
                        out_rd_count <= 2'd0;
                        out_rd_ptr   <= 1'b0;
                    end else if (out_rd_count == 2'd2) begin
                        out_rd_count <= 2'd1;
                        out_rd_ptr   <= 1'b1;
                    end
                end

                // Load new runs from accepted beat
                if (beat_go) begin
                    out_buf[0]   <= new_buf[0];
                    out_buf[1]   <= new_buf[1];
                    out_wr_count <= new_count;
                    out_rd_count <= new_count;
                    out_rd_ptr   <= 1'b0;

                    in_run      <= next_in_run;
                    cur_run_xs  <= next_run_xs;

                    if (is_last_beat) begin
                        pix_col <= '0;
                        if (pix_row + 16'd1 >= vres) begin
                            pix_row    <= '0;
                            frame_done <= 1'b1;
                        end else begin
                            pix_row <= pix_row + 16'd1;
                        end
                    end else begin
                        pix_col <= pix_col + 16'd4;
                    end
                end
            end
        end
    end

endmodule
