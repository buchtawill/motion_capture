// cell_accumulator.sv
// Processes 4 pixels per 32-bit AXIS beat. For each foreground pixel
// (value >= threshold), increments the cell's count and adds pixel x/y
// to the cell's running sums. Cell grid is stored in a dual-port BRAM
// (port A = read-modify-write by this module; port B = read-only for grid_scanner).
//
// Cell entry layout (80 bits):
//   [79:60] count  [19:0]
//   [59:28] sum_x  [31:0]
//   [27:0]  sum_y  [27:0]
//
// CELL_W and CELL_H must be powers of 2 (shift-based division).

`timescale 1ns / 1ps

module cell_accumulator #(
    parameter int CELL_W    = 32,   // pixels per cell, power of 2
    parameter int CELL_H    = 32,   // pixels per cell, power of 2
    parameter int MAX_CELLS = 2048  // max cells in grid
)(
    input  logic        clk,
    input  logic        rst_n,

    // Control
    input  logic        enable,     // high while accepting frames
    input  logic        clear,      // pulse: zero-fill BRAM before frame
    input  logic [7:0]  threshold,
    input  logic [15:0] hres,       // image width  in pixels
    input  logic [15:0] vres,       // image height in pixels

    // AXIS pixel input (32b = 4 x 8-bit pixels, little-endian)
    input  logic [31:0] s_data,
    input  logic        s_valid,
    output logic        s_ready,

    // Status
    output logic        frame_done,  // pulses high for 1 cycle at end of frame
    output logic        overflow,    // sticky: cell index out of range

    // Derived grid dimensions (for grid_scanner)
    output logic [10:0] grid_cols,
    output logic [10:0] grid_rows,

    // Cell BRAM port B (read-only, for grid_scanner, 1-cycle latency)
    input  logic [$clog2(MAX_CELLS)-1:0] portb_addr,
    output logic [79:0]                  portb_data
);

    // -------------------------------------------------------------------------
    // Local parameters
    // -------------------------------------------------------------------------
    localparam int CELL_W_LOG2 = $clog2(CELL_W);
    localparam int CELL_H_LOG2 = $clog2(CELL_H);
    localparam int ADDR_W      = $clog2(MAX_CELLS);

    // =========================================================================
    // Grid dimension (ceiling division, combinational)
    // =========================================================================
    always_comb begin
        grid_cols = 11'((hres + (16'(CELL_W) - 16'h1)) >> CELL_W_LOG2);
        grid_rows = 11'((vres + (16'(CELL_H) - 16'h1)) >> CELL_H_LOG2);
    end

    // =========================================================================
    // Scrub FSM
    // =========================================================================
    typedef enum logic [1:0] {
        ACC_SCRUB = 2'd0,
        ACC_IDLE  = 2'd1
    } acc_state_t;

    acc_state_t        acc_state;
    logic [ADDR_W-1:0] scrub_addr;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc_state  <= ACC_SCRUB;
            scrub_addr <= '0;
        end else begin
            case (acc_state)
                ACC_SCRUB: begin
                    if (scrub_addr == ADDR_W'(MAX_CELLS - 1)) begin
                        scrub_addr <= '0;
                        acc_state  <= ACC_IDLE;
                    end else begin
                        scrub_addr <= scrub_addr + 1'b1;
                    end
                end
                ACC_IDLE: begin
                    if (clear) begin
                        scrub_addr <= '0;
                        acc_state  <= ACC_SCRUB;
                    end
                end
                default: acc_state <= ACC_IDLE;
            endcase
        end
    end

    // =========================================================================
    // Pixel unpack: one pixel per cycle from a 32-bit beat (4 cycles/beat)
    // =========================================================================
    typedef enum logic [2:0] {
        PX_WAIT = 3'd0,
        PX_P0   = 3'd1,
        PX_P1   = 3'd2,
        PX_P2   = 3'd3,
        PX_P3   = 3'd4
    } px_state_t;

    px_state_t  px_state;
    logic [31:0] beat_hold;
    logic [1:0]  px_sub;

    assign s_ready = (acc_state == ACC_IDLE) && (px_state == PX_WAIT) && enable;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            px_state  <= PX_WAIT;
            beat_hold <= '0;
            px_sub    <= '0;
        end else begin
            case (px_state)
                PX_WAIT: begin
                    if (s_valid && s_ready) begin
                        beat_hold <= s_data;
                        px_state  <= PX_P0;
                        px_sub    <= 2'd0;
                    end
                end
                PX_P0: begin px_state <= PX_P1; px_sub <= 2'd1; end
                PX_P1: begin px_state <= PX_P2; px_sub <= 2'd2; end
                PX_P2: begin px_state <= PX_P3; px_sub <= 2'd3; end
                PX_P3: begin px_state <= PX_WAIT; px_sub <= 2'd0; end
                default: px_state <= PX_WAIT;
            endcase
        end
    end

    // Current pixel value and "processing this cycle" flag
    logic [7:0] cur_pix;
    logic       cur_valid;

    always_comb begin
        case (px_sub)
            2'd0: cur_pix = beat_hold[ 7: 0];
            2'd1: cur_pix = beat_hold[15: 8];
            2'd2: cur_pix = beat_hold[23:16];
            2'd3: cur_pix = beat_hold[31:24];
        endcase
        cur_valid = (px_state != PX_WAIT) && (acc_state == ACC_IDLE);
    end

    // =========================================================================
    // Pixel coordinates
    // =========================================================================
    logic [15:0] x_cnt, y_cnt;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x_cnt <= 16'h0;
            y_cnt <= 16'h0;
        end else if (clear) begin
            x_cnt <= 16'h0;
            y_cnt <= 16'h0;
        end else if (cur_valid) begin
            if (x_cnt == hres - 16'h1) begin
                x_cnt <= 16'h0;
                y_cnt <= (y_cnt == vres - 16'h1) ? 16'h0 : y_cnt + 16'h1;
            end else begin
                x_cnt <= x_cnt + 16'h1;
            end
        end
    end

    // =========================================================================
    // Frame done pulse
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            frame_done <= 1'b0;
        else
            frame_done <= cur_valid
                       && (hres != 16'h0) && (vres != 16'h0)
                       && (x_cnt == hres - 16'h1)
                       && (y_cnt == vres - 16'h1);
    end

    // =========================================================================
    // Cell index (S0): combinational, then registered to drive BRAM read
    // =========================================================================
    logic [ADDR_W-1:0] cell_idx;
    logic              cell_fg;

    logic [10:0] ccol, crow;   // cell column/row for current pixel
    always_comb begin
        ccol     = 11'(x_cnt >> CELL_W_LOG2);
        crow     = 11'(y_cnt >> CELL_H_LOG2);
        cell_idx = ADDR_W'(crow * grid_cols + ccol);
        cell_fg  = cur_valid && (cur_pix >= threshold);
    end

    // S0 -> S1 pipeline registers (after BRAM read latency)
    logic              s1_valid;
    logic [ADDR_W-1:0] s1_addr;
    logic [15:0]       s1_px_x, s1_px_y;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s1_valid <= 1'b0;
            s1_addr  <= '0;
            s1_px_x  <= '0;
            s1_px_y  <= '0;
            overflow <= 1'b0;
        end else begin
            s1_valid <= cell_fg;
            s1_addr  <= cell_idx;
            s1_px_x  <= x_cnt;
            s1_px_y  <= y_cnt;
            if (clear)
                overflow <= 1'b0;
            else if (cell_fg && (cell_idx >= ADDR_W'(MAX_CELLS)))
                overflow <= 1'b1;
        end
    end

    // =========================================================================
    // Dual-port BRAM (single inferred memory, two access patterns)
    //   Port A: write path for scrub and RMW write-back
    //   Port B: read-only for grid_scanner
    // =========================================================================
    logic [79:0]       cell_ram [0:MAX_CELLS-1];

    // Port A read (registered, 1-cycle latency)
    logic [79:0] pa_rd_data;

    always_ff @(posedge clk) begin
        pa_rd_data <= cell_ram[cell_idx];  // read current cell address (S0)
    end

    // Port B read (for grid_scanner)
    always_ff @(posedge clk) begin
        portb_data <= cell_ram[portb_addr];
    end

    // =========================================================================
    // Forwarding register: avoid stale read when same cell hit consecutively
    // =========================================================================
    logic              fwd_valid;
    logic [ADDR_W-1:0] fwd_addr;
    logic [79:0]       fwd_data;

    // Accumulation (S1): compute updated entry
    logic [79:0] base_data;
    logic [79:0] new_entry;
    logic        use_fwd_acc;

    always_comb begin
        use_fwd_acc = fwd_valid && (s1_addr == fwd_addr);
        base_data   = use_fwd_acc ? fwd_data : pa_rd_data;

        new_entry = {
            base_data[79:60] + 20'h1,                 // count
            base_data[59:28] + {16'h0, s1_px_x},      // sum_x
            base_data[27: 0] + {12'h0, s1_px_y}       // sum_y
        };
    end

    // =========================================================================
    // Single BRAM write always_ff (mux scrub vs. RMW)
    // =========================================================================
    always_ff @(posedge clk) begin
        if (acc_state == ACC_SCRUB) begin
            cell_ram[scrub_addr] <= 80'h0;
            fwd_valid            <= 1'b0;
            fwd_addr             <= '0;
            fwd_data             <= '0;
        end else if (s1_valid) begin
            cell_ram[s1_addr]    <= new_entry;
            fwd_valid            <= 1'b1;
            fwd_addr             <= s1_addr;
            fwd_data             <= new_entry;
        end else begin
            fwd_valid <= 1'b0;
        end
    end

endmodule
