`timescale 1ns / 1ps

module row_merger #(
    parameter int MAX_BLOBS        = 128,
    parameter int MAX_RUNS_PER_ROW = 640
)(
    input  logic        clk,
    input  logic        rst_n,

    input  logic        enable,
    input  logic        clear,

    // Run input from run_extractor
    input  logic [15:0] in_xs,
    input  logic [15:0] in_xe,
    input  logic [15:0] in_row,
    input  logic        in_last_in_row,
    input  logic        in_valid,
    output logic        in_ready,

    // Labeled run output to blob_table
    output logic [15:0] out_xs,
    output logic [15:0] out_xe,
    output logic [15:0] out_row,
    output logic [6:0]  out_blob_id,
    output logic        out_is_new,
    output logic        out_valid,
    input  logic        out_ready,

    // Merge events to blob_table
    output logic        merge_valid,
    output logic [6:0]  merge_a,
    output logic [6:0]  merge_b,
    input  logic        merge_ready,

    output logic        overflow
);

    localparam int BLOB_W = $clog2(MAX_BLOBS);
    localparam int RUN_W  = $clog2(MAX_RUNS_PER_ROW);

    // =========================================================================
    // Previous-row run buffer (register arrays for simplicity)
    // Each entry: xs[15:0], xe[15:0], blob_id[6:0]
    // =========================================================================
    logic [15:0] prev_xs  [0:MAX_RUNS_PER_ROW-1];
    logic [15:0] prev_xe  [0:MAX_RUNS_PER_ROW-1];
    logic [6:0]  prev_bid [0:MAX_RUNS_PER_ROW-1];
    logic [RUN_W-1:0] prev_count;

    // Current-row run buffer (for swap at end of row)
    logic [15:0] curr_xs  [0:MAX_RUNS_PER_ROW-1];
    logic [15:0] curr_xe  [0:MAX_RUNS_PER_ROW-1];
    logic [6:0]  curr_bid [0:MAX_RUNS_PER_ROW-1];
    logic [RUN_W-1:0] curr_count;

    // =========================================================================
    // FSM
    // =========================================================================
    typedef enum logic [2:0] {
        RM_IDLE     = 3'd0,
        RM_ACCEPT   = 3'd1,
        RM_SCAN     = 3'd2,
        RM_EMIT     = 3'd3,
        RM_MERGE    = 3'd4,
        RM_ROW_SWAP = 3'd5
    } rm_state_t;

    rm_state_t state, state_next;

    // =========================================================================
    // Working registers
    // =========================================================================
    logic [15:0] cur_xs, cur_xe, cur_row;
    logic        cur_last;
    logic [6:0]  cur_blob_id;
    logic        cur_is_new;
    logic        cur_has_match;

    // Scan pointer into prev-row buffer
    logic [RUN_W-1:0] scan_ptr;
    logic [RUN_W-1:0] scan_start;

    // Merge queue
    logic [6:0]  merge_queue_a, merge_queue_b;
    logic        merge_pending;

    // Blob ID allocator
    logic [7:0]  next_blob_id;

    // =========================================================================
    // Next-state logic
    // =========================================================================
    always_comb begin
        state_next = state;

        case (state)
            RM_IDLE: begin
                if (enable && !clear)
                    state_next = RM_ACCEPT;
            end

            RM_ACCEPT: begin
                if (clear)
                    state_next = RM_IDLE;
                else if (in_valid)
                    state_next = (prev_count == '0) ? RM_EMIT : RM_SCAN;
            end

            RM_SCAN: begin
                if (scan_ptr >= prev_count) begin
                    // Scanned all prev runs
                    state_next = RM_EMIT;
                end else if (prev_xs[scan_ptr] > cur_xe + 16'd1) begin
                    // Past possible overlap region
                    state_next = RM_EMIT;
                end
                // else stay in RM_SCAN (advance scan_ptr)
            end

            RM_EMIT: begin
                if (out_valid && out_ready) begin
                    if (merge_pending)
                        state_next = RM_MERGE;
                    else if (cur_last)
                        state_next = RM_ROW_SWAP;
                    else
                        state_next = RM_ACCEPT;
                end
            end

            RM_MERGE: begin
                if (merge_valid && merge_ready) begin
                    if (cur_last)
                        state_next = RM_ROW_SWAP;
                    else
                        state_next = RM_ACCEPT;
                end
            end

            RM_ROW_SWAP: begin
                state_next = RM_ACCEPT;
            end

            default: state_next = RM_IDLE;
        endcase
    end

    // =========================================================================
    // Output assignments
    // =========================================================================
    always_comb begin
        in_ready = (state == RM_ACCEPT) && enable && !clear;

        out_valid   = (state == RM_EMIT);
        out_xs      = cur_xs;
        out_xe      = cur_xe;
        out_row     = cur_row;
        out_blob_id = cur_blob_id;
        out_is_new  = cur_is_new;

        merge_valid = (state == RM_MERGE) && merge_pending;
        merge_a     = merge_queue_a;
        merge_b     = merge_queue_b;
    end

    // =========================================================================
    // Sequential logic
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= RM_IDLE;
            prev_count    <= '0;
            curr_count    <= '0;
            scan_ptr      <= '0;
            scan_start    <= '0;
            next_blob_id  <= '0;
            overflow      <= 1'b0;
            cur_xs        <= '0;
            cur_xe        <= '0;
            cur_row       <= '0;
            cur_last      <= 1'b0;
            cur_blob_id   <= '0;
            cur_is_new    <= 1'b0;
            cur_has_match <= 1'b0;
            merge_pending <= 1'b0;
            merge_queue_a <= '0;
            merge_queue_b <= '0;
        end else begin
            state <= state_next;

            if (clear) begin
                prev_count    <= '0;
                curr_count    <= '0;
                scan_ptr      <= '0;
                scan_start    <= '0;
                next_blob_id  <= '0;
                overflow      <= 1'b0;
                merge_pending <= 1'b0;
            end else begin
                case (state)
                    RM_ACCEPT: begin
                        if (in_valid) begin
                            cur_xs        <= in_xs;
                            cur_xe        <= in_xe;
                            cur_row       <= in_row;
                            cur_last      <= in_last_in_row;
                            cur_has_match <= 1'b0;
                            cur_is_new    <= 1'b0;
                            merge_pending <= 1'b0;
                            scan_ptr      <= scan_start;
                        end
                    end

                    RM_SCAN: begin
                        if (scan_ptr < prev_count &&
                            !(prev_xs[scan_ptr] > cur_xe + 16'd1)) begin
                            // Check 8-connected overlap
                            if (prev_xe[scan_ptr] + 16'd1 >= cur_xs &&
                                prev_xs[scan_ptr] <= cur_xe + 16'd1) begin
                                if (!cur_has_match) begin
                                    // First overlap: take this blob_id
                                    cur_blob_id   <= prev_bid[scan_ptr];
                                    cur_has_match <= 1'b1;
                                    cur_is_new    <= 1'b0;
                                end else if (prev_bid[scan_ptr] != cur_blob_id) begin
                                    // Additional overlap with different blob: merge
                                    merge_queue_a <= cur_blob_id;
                                    merge_queue_b <= prev_bid[scan_ptr];
                                    merge_pending <= 1'b1;
                                end
                            end

                            // Advance scan_start past runs that can't overlap future runs
                            if (prev_xe[scan_ptr] + 16'd1 < cur_xs) begin
                                scan_start <= scan_ptr + 1'b1;
                            end

                            scan_ptr <= scan_ptr + 1'b1;
                        end
                    end

                    RM_EMIT: begin
                        if (out_valid && out_ready) begin
                            // If no match found, allocate new blob ID
                            if (!cur_has_match) begin
                                if (next_blob_id < 8'(MAX_BLOBS)) begin
                                    cur_blob_id  <= next_blob_id[6:0];
                                    cur_is_new   <= 1'b1;
                                    next_blob_id <= next_blob_id + 8'd1;
                                end else begin
                                    overflow <= 1'b1;
                                end
                            end

                            // Store in current-row buffer
                            if (curr_count < RUN_W'(MAX_RUNS_PER_ROW - 1)) begin
                                curr_xs[curr_count]  <= cur_xs;
                                curr_xe[curr_count]  <= cur_xe;
                                curr_bid[curr_count] <= cur_has_match ? cur_blob_id :
                                                        (next_blob_id < 8'(MAX_BLOBS) ?
                                                         next_blob_id[6:0] - 7'd1 : cur_blob_id);
                                curr_count           <= curr_count + 1'b1;
                            end
                        end
                    end

                    RM_MERGE: begin
                        if (merge_valid && merge_ready) begin
                            merge_pending <= 1'b0;
                        end
                    end

                    RM_ROW_SWAP: begin
                        // Copy current row into previous row
                        for (int i = 0; i < MAX_RUNS_PER_ROW; i++) begin
                            prev_xs[i]  <= curr_xs[i];
                            prev_xe[i]  <= curr_xe[i];
                            prev_bid[i] <= curr_bid[i];
                        end
                        prev_count <= curr_count;
                        curr_count <= '0;
                        scan_start <= '0;
                        scan_ptr   <= '0;
                    end

                    default: ;
                endcase
            end
        end
    end

endmodule
