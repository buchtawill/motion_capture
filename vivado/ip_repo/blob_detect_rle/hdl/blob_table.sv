`timescale 1ns / 1ps

module blob_table #(
    parameter int MAX_BLOBS = 128
)(
    input  logic        clk,
    input  logic        rst_n,

    input  logic        clear,
    input  logic        flatten_start,

    // Labeled run input from row_merger
    input  logic [15:0] in_xs,
    input  logic [15:0] in_xe,
    input  logic [15:0] in_row,
    input  logic [6:0]  in_blob_id,
    input  logic        in_is_new,
    input  logic        in_valid,
    output logic        in_ready,

    // Merge input from row_merger
    input  logic        merge_valid,
    input  logic [6:0]  merge_a,
    input  logic [6:0]  merge_b,
    output logic        merge_ready,

    // Status
    output logic        flatten_done,
    output logic [6:0]  blob_count,

    // Result BRAM read port
    input  logic [6:0]  result_rd_addr,
    output logic [159:0] result_rd_data
);

    localparam int BLOB_W = $clog2(MAX_BLOBS);

    // =========================================================================
    // Union-find parent array (register file)
    // =========================================================================
    logic [6:0] parent [0:MAX_BLOBS-1];
    logic [3:0] rank   [0:MAX_BLOBS-1];

    // =========================================================================
    // Blob descriptor storage (register arrays for RMW in single cycle)
    // =========================================================================
    logic [31:0] blob_count_r [0:MAX_BLOBS-1];
    logic [31:0] blob_sum_x   [0:MAX_BLOBS-1];
    logic [31:0] blob_sum_y   [0:MAX_BLOBS-1];
    logic [15:0] blob_xmin    [0:MAX_BLOBS-1];
    logic [15:0] blob_xmax    [0:MAX_BLOBS-1];
    logic [15:0] blob_ymin    [0:MAX_BLOBS-1];
    logic [15:0] blob_ymax    [0:MAX_BLOBS-1];

    // =========================================================================
    // Result BRAM (written during flatten, read by register interface)
    // =========================================================================
    logic [159:0] result_ram [0:MAX_BLOBS-1];

    always_ff @(posedge clk) begin
        result_rd_data <= result_ram[result_rd_addr];
    end

    // =========================================================================
    // FSM
    // =========================================================================
    typedef enum logic [3:0] {
        BT_IDLE      = 4'd0,
        BT_CLEAR     = 4'd1,
        BT_READY     = 4'd2,
        BT_ACCEPT    = 4'd3,
        BT_FIND      = 4'd4,
        BT_ACCUM     = 4'd5,
        BT_DO_MERGE  = 4'd6,
        BT_FIND_MA   = 4'd7,
        BT_FIND_MB   = 4'd8,
        BT_MERGE_UF  = 4'd9,
        BT_FLATTEN   = 4'd10,
        BT_FLAT_EMIT = 4'd11,
        BT_DONE      = 4'd12
    } bt_state_t;

    bt_state_t state, state_next;

    // =========================================================================
    // Working registers
    // =========================================================================
    logic [7:0]  clr_idx;
    logic [15:0] w_xs, w_xe, w_row;
    logic [6:0]  w_blob_id;
    logic        w_is_new;

    // Find operation
    logic [6:0]  find_x;
    logic [6:0]  find_root;
    logic [3:0]  find_iter;

    // Merge operation
    logic [6:0]  mrg_a, mrg_b;
    logic [6:0]  root_a, root_b;

    // Flatten
    logic [7:0]  flat_idx;
    logic [6:0]  flat_out_id;

    // Run accumulation intermediates
    logic [15:0] run_len;
    logic [31:0] sum_x_add;

    always_comb begin
        run_len   = w_xe - w_xs + 16'd1;
        // (xs + xe) * run_len is always even, so >>1 is exact
        sum_x_add = (({16'h0, w_xs} + {16'h0, w_xe}) * {16'h0, run_len}) >> 1;
    end

    // =========================================================================
    // Next-state logic
    // =========================================================================
    always_comb begin
        state_next = state;
        case (state)
            BT_IDLE: begin
                if (clear)
                    state_next = BT_CLEAR;
            end
            BT_CLEAR: begin
                if (clr_idx == 8'(MAX_BLOBS - 1))
                    state_next = BT_READY;
            end
            BT_READY: begin
                if (in_valid)
                    state_next = BT_ACCEPT;
                else if (merge_valid)
                    state_next = BT_DO_MERGE;
                else if (flatten_start)
                    state_next = BT_FLATTEN;
            end
            BT_ACCEPT: begin
                state_next = BT_FIND;
            end
            BT_FIND: begin
                if (parent[find_x] == find_x || find_iter >= 4'd8)
                    state_next = BT_ACCUM;
            end
            BT_ACCUM: begin
                state_next = BT_READY;
            end
            BT_DO_MERGE: begin
                state_next = BT_FIND_MA;
            end
            BT_FIND_MA: begin
                if (parent[find_x] == find_x || find_iter >= 4'd8)
                    state_next = BT_FIND_MB;
            end
            BT_FIND_MB: begin
                if (parent[find_x] == find_x || find_iter >= 4'd8)
                    state_next = BT_MERGE_UF;
            end
            BT_MERGE_UF: begin
                state_next = BT_READY;
            end
            BT_FLATTEN: begin
                if (flat_idx >= 8'(MAX_BLOBS))
                    state_next = BT_DONE;
            end
            BT_FLAT_EMIT: begin
                state_next = BT_FLATTEN;
            end
            BT_DONE: begin
                state_next = BT_IDLE;
            end
            default: state_next = BT_IDLE;
        endcase
    end

    // =========================================================================
    // Output assignments
    // =========================================================================
    always_comb begin
        in_ready     = (state == BT_READY) && !merge_valid;
        merge_ready  = (state == BT_READY);
        flatten_done = (state == BT_DONE);
    end

    // =========================================================================
    // Sequential logic
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= BT_IDLE;
            clr_idx     <= '0;
            blob_count  <= '0;
            flat_idx    <= '0;
            flat_out_id <= '0;
            find_x      <= '0;
            find_root   <= '0;
            find_iter   <= '0;
            w_xs        <= '0;
            w_xe        <= '0;
            w_row       <= '0;
            w_blob_id   <= '0;
            w_is_new    <= 1'b0;
            mrg_a       <= '0;
            mrg_b       <= '0;
            root_a      <= '0;
            root_b      <= '0;
        end else begin
            state <= state_next;

            case (state)
                BT_IDLE: begin
                    if (clear) begin
                        clr_idx <= '0;
                    end
                end

                BT_CLEAR: begin
                    parent[clr_idx]       <= clr_idx[6:0];
                    rank[clr_idx]         <= '0;
                    blob_count_r[clr_idx] <= '0;
                    blob_sum_x[clr_idx]   <= '0;
                    blob_sum_y[clr_idx]   <= '0;
                    blob_xmin[clr_idx]    <= 16'hFFFF;
                    blob_xmax[clr_idx]    <= '0;
                    blob_ymin[clr_idx]    <= 16'hFFFF;
                    blob_ymax[clr_idx]    <= '0;
                    result_ram[clr_idx]   <= '0;
                    clr_idx <= clr_idx + 8'd1;
                end

                BT_READY: begin
                    if (in_valid && !merge_valid) begin
                        w_xs      <= in_xs;
                        w_xe      <= in_xe;
                        w_row     <= in_row;
                        w_blob_id <= in_blob_id;
                        w_is_new  <= in_is_new;
                    end else if (merge_valid) begin
                        mrg_a <= merge_a;
                        mrg_b <= merge_b;
                    end
                end

                BT_ACCEPT: begin
                    find_x    <= w_blob_id;
                    find_iter <= '0;
                end

                BT_FIND: begin
                    if (parent[find_x] == find_x || find_iter >= 4'd8) begin
                        find_root <= find_x;
                    end else begin
                        // Path compression: point to grandparent
                        parent[find_x] <= parent[parent[find_x]];
                        find_x         <= parent[find_x];
                        find_iter      <= find_iter + 4'd1;
                    end
                end

                BT_ACCUM: begin
                    blob_count_r[find_root] <= blob_count_r[find_root] + {16'h0, run_len};
                    blob_sum_x[find_root]   <= blob_sum_x[find_root] + sum_x_add;
                    blob_sum_y[find_root]   <= blob_sum_y[find_root] + ({16'h0, w_row} * {16'h0, run_len});
                    if (w_xs < blob_xmin[find_root]) blob_xmin[find_root] <= w_xs;
                    if (w_xe > blob_xmax[find_root]) blob_xmax[find_root] <= w_xe;
                    if (w_row < blob_ymin[find_root]) blob_ymin[find_root] <= w_row;
                    if (w_row > blob_ymax[find_root]) blob_ymax[find_root] <= w_row;
                end

                BT_DO_MERGE: begin
                    find_x    <= mrg_a;
                    find_iter <= '0;
                end

                BT_FIND_MA: begin
                    if (parent[find_x] == find_x || find_iter >= 4'd8) begin
                        root_a    <= find_x;
                        find_x    <= mrg_b;
                        find_iter <= '0;
                    end else begin
                        parent[find_x] <= parent[parent[find_x]];
                        find_x         <= parent[find_x];
                        find_iter      <= find_iter + 4'd1;
                    end
                end

                BT_FIND_MB: begin
                    if (parent[find_x] == find_x || find_iter >= 4'd8) begin
                        root_b <= find_x;
                    end else begin
                        parent[find_x] <= parent[parent[find_x]];
                        find_x         <= parent[find_x];
                        find_iter      <= find_iter + 4'd1;
                    end
                end

                BT_MERGE_UF: begin
                    if (root_a != root_b) begin
                        // Union by rank
                        if (rank[root_a] < rank[root_b]) begin
                            parent[root_a] <= root_b;
                            // Merge descriptors into root_b
                            blob_count_r[root_b] <= blob_count_r[root_b] + blob_count_r[root_a];
                            blob_sum_x[root_b]   <= blob_sum_x[root_b] + blob_sum_x[root_a];
                            blob_sum_y[root_b]   <= blob_sum_y[root_b] + blob_sum_y[root_a];
                            if (blob_xmin[root_a] < blob_xmin[root_b])
                                blob_xmin[root_b] <= blob_xmin[root_a];
                            if (blob_xmax[root_a] > blob_xmax[root_b])
                                blob_xmax[root_b] <= blob_xmax[root_a];
                            if (blob_ymin[root_a] < blob_ymin[root_b])
                                blob_ymin[root_b] <= blob_ymin[root_a];
                            if (blob_ymax[root_a] > blob_ymax[root_b])
                                blob_ymax[root_b] <= blob_ymax[root_a];
                            // Clear merged source
                            blob_count_r[root_a] <= '0;
                            blob_sum_x[root_a]   <= '0;
                            blob_sum_y[root_a]   <= '0;
                        end else begin
                            parent[root_b] <= root_a;
                            blob_count_r[root_a] <= blob_count_r[root_a] + blob_count_r[root_b];
                            blob_sum_x[root_a]   <= blob_sum_x[root_a] + blob_sum_x[root_b];
                            blob_sum_y[root_a]   <= blob_sum_y[root_a] + blob_sum_y[root_b];
                            if (blob_xmin[root_b] < blob_xmin[root_a])
                                blob_xmin[root_a] <= blob_xmin[root_b];
                            if (blob_xmax[root_b] > blob_xmax[root_a])
                                blob_xmax[root_a] <= blob_xmax[root_b];
                            if (blob_ymin[root_b] < blob_ymin[root_a])
                                blob_ymin[root_a] <= blob_ymin[root_b];
                            if (blob_ymax[root_b] > blob_ymax[root_a])
                                blob_ymax[root_a] <= blob_ymax[root_b];
                            blob_count_r[root_b] <= '0;
                            blob_sum_x[root_b]   <= '0;
                            blob_sum_y[root_b]   <= '0;
                            if (rank[root_a] == rank[root_b])
                                rank[root_a] <= rank[root_a] + 4'd1;
                        end
                    end
                end

                BT_FLATTEN: begin
                    if (flat_idx < 8'(MAX_BLOBS)) begin
                        // Find root for this entry
                        if (parent[flat_idx[6:0]] == flat_idx[6:0] &&
                            blob_count_r[flat_idx[6:0]] != 32'h0) begin
                            // This is a root with data — emit to result BRAM
                            result_ram[flat_out_id] <= {
                                blob_count_r[flat_idx[6:0]],
                                blob_sum_x[flat_idx[6:0]],
                                blob_sum_y[flat_idx[6:0]],
                                blob_xmin[flat_idx[6:0]],
                                blob_xmax[flat_idx[6:0]],
                                blob_ymin[flat_idx[6:0]],
                                blob_ymax[flat_idx[6:0]]
                            };
                            flat_out_id <= flat_out_id + 7'd1;
                        end
                        flat_idx <= flat_idx + 8'd1;
                    end
                end

                BT_DONE: begin
                    blob_count <= flat_out_id;
                    flat_idx    <= '0;
                    flat_out_id <= '0;
                end

                default: ;
            endcase
        end
    end

endmodule
