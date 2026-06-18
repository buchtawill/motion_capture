// blob_emitter.sv
// Receives per-cell blob assignments from grid_scanner and aggregates
// statistics into per-blob descriptors stored in a result BRAM.
//
// Blob BRAM layout (160 bits per entry):
//   [159:128] count  [31:0]
//   [127: 96] sum_x  [31:0]
//   [ 95: 64] sum_y  [31:0]
//   [ 63: 48] xmin   [15:0]
//   [ 47: 32] xmax   [15:0]
//   [ 31: 16] ymin   [15:0]
//   [ 15:  0] ymax   [15:0]
//
// Pixel bounding box derived from cell coordinates:
//   cell_xmin = cell_col * CELL_W  (shift)
//   cell_xmax = min((cell_col+1)*CELL_W - 1, hres-1)
//   cell_ymin = cell_row * CELL_H  (shift)
//   cell_ymax = min((cell_row+1)*CELL_H - 1, vres-1)
//
// Throughput: one cell per 2 cycles (accept then write-back; alternating phases).

`timescale 1ns / 1ps

module blob_emitter #(
    parameter int MAX_BLOBS = 128,
    parameter int CELL_W    = 32,
    parameter int CELL_H    = 32,
    parameter int MAX_CELLS = 2048
)(
    input  logic        clk,
    input  logic        rst_n,

    // Control
    input  logic        clear,    // pulse: zero result BRAM
    output logic        busy,     // high while clearing

    // Image dimensions (for bbox clamping)
    input  logic [15:0] hres,
    input  logic [15:0] vres,

    // Cell data from grid_scanner (valid/ready)
    input  logic        in_valid,
    output logic        in_ready,
    input  logic [$clog2(MAX_CELLS)-1:0] in_cell_idx,
    input  logic [10:0] in_cell_col,
    input  logic [10:0] in_cell_row,
    input  logic [$clog2(MAX_BLOBS)-1:0] in_blob_id,
    input  logic [19:0] in_cell_count,
    input  logic [31:0] in_cell_sum_x,
    input  logic [27:0] in_cell_sum_y,

    // Result BRAM read port (1-cycle latency)
    input  logic [$clog2(MAX_BLOBS)-1:0] result_rd_addr,
    output logic [159:0]                 result_rd_data
);

    localparam int BLOB_W      = $clog2(MAX_BLOBS);
    localparam int CELL_W_LOG2 = $clog2(CELL_W);
    localparam int CELL_H_LOG2 = $clog2(CELL_H);

    // =========================================================================
    // Result BRAM
    // =========================================================================
    logic [159:0] result_ram [0:MAX_BLOBS-1];

    // Port B read (for register interface)
    always_ff @(posedge clk) begin
        result_rd_data <= result_ram[result_rd_addr];
    end

    // =========================================================================
    // FSM
    // =========================================================================
    typedef enum logic [1:0] {
        EM_IDLE  = 2'd0,
        EM_CLEAR = 2'd1,
        EM_RUN   = 2'd2
    } em_state_t;

    em_state_t         em_state;
    logic [BLOB_W-1:0] clr_addr;

    assign busy = (em_state == EM_CLEAR);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            em_state <= EM_IDLE;
            clr_addr <= '0;
        end else begin
            case (em_state)
                EM_IDLE: begin
                    if (clear) begin
                        clr_addr <= '0;
                        em_state <= EM_CLEAR;
                    end
                end
                EM_CLEAR: begin
                    if (clr_addr == BLOB_W'(MAX_BLOBS - 1))
                        em_state <= EM_RUN;
                    else
                        clr_addr <= clr_addr + 1'b1;
                end
                EM_RUN: begin
                    if (clear) begin
                        clr_addr <= '0;
                        em_state <= EM_CLEAR;
                    end
                end
                default: em_state <= EM_IDLE;
            endcase
        end
    end

    // =========================================================================
    // Two-phase pipeline
    //   Phase 0 (ACCEPT): latch input; issue BRAM read for in_blob_id
    //   Phase 1 (WRITEBACK): BRAM data available; accumulate; write back
    //   Only accept new input in phase 0.
    // =========================================================================
    logic              phase;      // 0 = accept, 1 = write-back
    logic              s1_valid;
    logic [BLOB_W-1:0] s1_blob_id;
    logic [15:0]       s1_cell_xmin, s1_cell_xmax;
    logic [15:0]       s1_cell_ymin, s1_cell_ymax;
    logic [19:0]       s1_cell_count;
    logic [31:0]       s1_cell_sum_x;
    logic [27:0]       s1_cell_sum_y;

    // Forwarding register
    logic              fwd_valid;
    logic [BLOB_W-1:0] fwd_addr;
    logic [159:0]      fwd_data;

    // BRAM port A registered read
    logic [159:0]      pa_rd_data;

    assign in_ready = (em_state == EM_RUN) && (phase == 1'b0);

    // =========================================================================
    // Bounding box computation (combinational, module-scope temporaries)
    // =========================================================================
    logic [15:0] s0_xmax_raw, s0_ymax_raw;
    logic [15:0] s0_cell_xmin, s0_cell_xmax, s0_cell_ymin, s0_cell_ymax;

    always_comb begin
        s0_cell_xmin = {5'h0, in_cell_col} << CELL_W_LOG2;
        s0_cell_ymin = {5'h0, in_cell_row} << CELL_H_LOG2;

        s0_xmax_raw  = (({5'h0, in_cell_col} + 16'h1) << CELL_W_LOG2) - 16'h1;
        s0_cell_xmax = (hres != 16'h0 && s0_xmax_raw >= hres) ? (hres - 16'h1) : s0_xmax_raw;

        s0_ymax_raw  = (({5'h0, in_cell_row} + 16'h1) << CELL_H_LOG2) - 16'h1;
        s0_cell_ymax = (vres != 16'h0 && s0_ymax_raw >= vres) ? (vres - 16'h1) : s0_ymax_raw;
    end

    // =========================================================================
    // Accumulation (S1, combinational)
    //   Module-scope temporaries (no inline declarations)
    // =========================================================================
    logic [159:0] base_entry, new_entry;
    logic         is_first, use_fwd;
    logic [31:0]  acc_count, acc_sum_x, acc_sum_y;
    logic [15:0]  acc_xmin, acc_xmax, acc_ymin, acc_ymax;

    always_comb begin
        use_fwd    = fwd_valid && (s1_blob_id == fwd_addr);
        base_entry = use_fwd ? fwd_data : pa_rd_data;
        is_first   = (base_entry[159:128] == 32'h0);

        acc_count = base_entry[159:128] + {12'h0, s1_cell_count};
        acc_sum_x = base_entry[127: 96] + s1_cell_sum_x;
        acc_sum_y = base_entry[ 95: 64] + {4'h0, s1_cell_sum_y};

        acc_xmin  = is_first ? s1_cell_xmin :
                    (s1_cell_xmin < base_entry[63:48]) ? s1_cell_xmin : base_entry[63:48];
        acc_xmax  = is_first ? s1_cell_xmax :
                    (s1_cell_xmax > base_entry[47:32]) ? s1_cell_xmax : base_entry[47:32];
        acc_ymin  = is_first ? s1_cell_ymin :
                    (s1_cell_ymin < base_entry[31:16]) ? s1_cell_ymin : base_entry[31:16];
        acc_ymax  = is_first ? s1_cell_ymax :
                    (s1_cell_ymax > base_entry[15: 0]) ? s1_cell_ymax : base_entry[15: 0];

        new_entry = {acc_count, acc_sum_x, acc_sum_y, acc_xmin, acc_xmax, acc_ymin, acc_ymax};
    end

    // =========================================================================
    // Single BRAM write (clear zeros; RMW write-back)
    // =========================================================================
    always_ff @(posedge clk) begin
        if (em_state == EM_CLEAR) begin
            result_ram[clr_addr] <= 160'h0;
        end else if (em_state == EM_RUN && s1_valid && phase == 1'b1) begin
            result_ram[s1_blob_id] <= new_entry;
        end
    end

    // =========================================================================
    // BRAM port A read (latches blob entry in phase 0)
    // =========================================================================
    always_ff @(posedge clk) begin
        if (em_state == EM_RUN && phase == 1'b0 && in_valid) begin
            pa_rd_data <= result_ram[in_blob_id];
        end
    end

    // =========================================================================
    // Pipeline and phase control
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            phase         <= 1'b0;
            s1_valid      <= 1'b0;
            s1_blob_id    <= '0;
            s1_cell_xmin  <= '0;
            s1_cell_xmax  <= '0;
            s1_cell_ymin  <= '0;
            s1_cell_ymax  <= '0;
            s1_cell_count <= '0;
            s1_cell_sum_x <= '0;
            s1_cell_sum_y <= '0;
            fwd_valid     <= 1'b0;
            fwd_addr      <= '0;
            fwd_data      <= '0;
        end else if (em_state == EM_RUN) begin
            if (phase == 1'b0) begin
                // Accept phase
                if (in_valid) begin
                    s1_valid      <= 1'b1;
                    s1_blob_id    <= in_blob_id;
                    s1_cell_xmin  <= s0_cell_xmin;
                    s1_cell_xmax  <= s0_cell_xmax;
                    s1_cell_ymin  <= s0_cell_ymin;
                    s1_cell_ymax  <= s0_cell_ymax;
                    s1_cell_count <= in_cell_count;
                    s1_cell_sum_x <= in_cell_sum_x;
                    s1_cell_sum_y <= in_cell_sum_y;
                    phase         <= 1'b1;
                end
                // No input: stay in phase 0
            end else begin
                // Write-back phase
                if (s1_valid) begin
                    fwd_valid <= 1'b1;
                    fwd_addr  <= s1_blob_id;
                    fwd_data  <= new_entry;
                end else begin
                    fwd_valid <= 1'b0;
                end
                s1_valid <= 1'b0;
                phase    <= 1'b0;
            end
        end else begin
            phase     <= 1'b0;
            s1_valid  <= 1'b0;
            fwd_valid <= 1'b0;
        end
    end

endmodule
