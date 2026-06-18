// grid_scanner.sv
// Post-frame flood-fill blob detector.
//
// Scans the cell grid linearly. For each unvisited, non-empty cell,
// performs an iterative 4-connected depth-first search using a small
// hardware stack (register array). Each cell that belongs to a connected
// component is emitted with its blob_id and cell statistics.
//
// Cell BRAM layout (80 bits, 1-cycle read latency):
//   [79:60] count [19:0]
//   [59:28] sum_x [31:0]
//   [27: 0] sum_y [27:0]
//
// Emits one cell record per handshake to blob_emitter.

`timescale 1ns / 1ps

module grid_scanner #(
    parameter int MAX_CELLS   = 2048,
    parameter int MAX_BLOBS   = 128,
    parameter int STACK_DEPTH = 64
)(
    input  logic        clk,
    input  logic        rst_n,

    // Control
    input  logic        start,       // pulse: begin scan
    output logic        done,        // pulse: scan complete

    // Grid dimensions
    input  logic [10:0] grid_cols,
    input  logic [10:0] grid_rows,

    // Cell BRAM read port (1-cycle latency)
    output logic [$clog2(MAX_CELLS)-1:0] cell_addr,
    input  logic [79:0]                  cell_data,

    // Output to blob_emitter (valid/ready handshake)
    output logic        out_valid,
    input  logic        out_ready,
    output logic [$clog2(MAX_CELLS)-1:0] out_cell_idx,
    output logic [10:0] out_cell_col,
    output logic [10:0] out_cell_row,
    output logic [$clog2(MAX_BLOBS)-1:0] out_blob_id,
    output logic [19:0] out_cell_count,
    output logic [31:0] out_cell_sum_x,
    output logic [27:0] out_cell_sum_y,

    // Blob count output
    output logic [$clog2(MAX_BLOBS)-1:0] blob_count,

    // Overflow: too many blobs or stack full
    output logic overflow
);

    localparam int ADDR_W = $clog2(MAX_CELLS);
    localparam int BLOB_W = $clog2(MAX_BLOBS);
    localparam int STCK_W = $clog2(STACK_DEPTH);

    // =========================================================================
    // Visited bit array (distributed RAM, one bit per cell)
    // =========================================================================
    logic visited [0:MAX_CELLS-1];

    // =========================================================================
    // Hardware stack (register array)
    // =========================================================================
    logic [ADDR_W-1:0] stack [0:STACK_DEPTH-1];
    logic [STCK_W:0]   sp;    // 0 = empty

    wire stack_empty = (sp == '0);
    wire stack_full  = (sp == STCK_W+1'(STACK_DEPTH));

    // =========================================================================
    // FSM
    // =========================================================================
    typedef enum logic [3:0] {
        ST_IDLE        = 4'd0,
        ST_CLEAR       = 4'd1,   // clear visited[]
        ST_LINEAR_RD   = 4'd2,   // issue BRAM read for linear scan cell
        ST_LINEAR_WAIT = 4'd3,   // wait 1 cycle for BRAM latency
        ST_LINEAR_CHK  = 4'd4,   // inspect cell; start DFS or advance
        ST_DFS_PUSH    = 4'd5,   // push seed onto stack
        ST_DFS_POP     = 4'd6,   // pop + issue BRAM read (or end component)
        ST_DFS_WAIT    = 4'd7,   // wait 1 cycle for BRAM latency
        ST_DFS_CHK     = 4'd8,   // check if popped cell is non-empty
        ST_DFS_EMIT    = 4'd9,   // emit cell (stall for backpressure)
        ST_DFS_NBRS    = 4'd10,  // push unvisited neighbors (4 sub-steps)
        ST_DONE        = 4'd11
    } state_t;

    state_t state;

    // Linear scan index and total cell count
    logic [ADDR_W-1:0] linear_idx;
    logic [ADDR_W-1:0] total_cells;
    logic [10:0]       linear_col, linear_row;

    // DFS current cell (registered when popped)
    logic [ADDR_W-1:0] dfs_cell;
    logic [10:0]       dfs_col, dfs_row;

    // Stack carries (col, row) alongside flat index to avoid dividers
    logic [10:0] stack_col [0:STACK_DEPTH-1];
    logic [10:0] stack_row [0:STACK_DEPTH-1];

    // Blob tracking (1 extra bit to detect overflow at MAX_BLOBS)
    logic [BLOB_W:0] cur_blob_id;

    // Clear counter
    logic [ADDR_W-1:0] clr_addr;

    // Neighbor push sub-phase (0=N, 1=S, 2=E, 3=W)
    logic [1:0]        nbr_phase;

    assign blob_count = BLOB_W'(cur_blob_id);

    // =========================================================================
    // Cell BRAM read address (combinational mux)
    // =========================================================================
    always_comb begin
        case (state)
            ST_LINEAR_RD: cell_addr = linear_idx;
            ST_DFS_POP:   cell_addr = stack[sp - 1'b1];  // peek top of stack
            default:      cell_addr = '0;
        endcase
    end

    // =========================================================================
    // Neighbor computation (fully combinational, used in ST_DFS_NBRS)
    // =========================================================================
    logic [ADDR_W-1:0] nbr_idx;
    logic [10:0]       nbr_col, nbr_row;
    logic              nbr_geo_valid;  // neighbor is within grid bounds

    always_comb begin
        nbr_idx       = '0;
        nbr_col       = '0;
        nbr_row       = '0;
        nbr_geo_valid = 1'b0;
        case (nbr_phase)
            2'd0: begin // North
                if (dfs_row > 11'h0) begin
                    nbr_col       = dfs_col;
                    nbr_row       = dfs_row - 11'h1;
                    nbr_idx       = ADDR_W'(nbr_row * grid_cols + nbr_col);
                    nbr_geo_valid = 1'b1;
                end
            end
            2'd1: begin // South
                if (dfs_row < grid_rows - 11'h1) begin
                    nbr_col       = dfs_col;
                    nbr_row       = dfs_row + 11'h1;
                    nbr_idx       = ADDR_W'(nbr_row * grid_cols + nbr_col);
                    nbr_geo_valid = 1'b1;
                end
            end
            2'd2: begin // East
                if (dfs_col < grid_cols - 11'h1) begin
                    nbr_col       = dfs_col + 11'h1;
                    nbr_row       = dfs_row;
                    nbr_idx       = ADDR_W'(nbr_row * grid_cols + nbr_col);
                    nbr_geo_valid = 1'b1;
                end
            end
            2'd3: begin // West
                if (dfs_col > 11'h0) begin
                    nbr_col       = dfs_col - 11'h1;
                    nbr_row       = dfs_row;
                    nbr_idx       = ADDR_W'(nbr_row * grid_cols + nbr_col);
                    nbr_geo_valid = 1'b1;
                end
            end
            default: ;
        endcase
    end

    // =========================================================================
    // Main FSM
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= ST_IDLE;
            done        <= 1'b0;
            overflow    <= 1'b0;
            linear_idx  <= '0;
            linear_col  <= '0;
            linear_row  <= '0;
            total_cells <= '0;
            cur_blob_id <= '0;
            sp          <= '0;
            clr_addr    <= '0;
            nbr_phase   <= '0;
            dfs_cell    <= '0;
            dfs_col     <= '0;
            dfs_row     <= '0;
            out_valid   <= 1'b0;
            out_cell_idx   <= '0;
            out_cell_col   <= '0;
            out_cell_row   <= '0;
            out_blob_id    <= '0;
            out_cell_count <= '0;
            out_cell_sum_x <= '0;
            out_cell_sum_y <= '0;
            // visited[] is cleared by ST_CLEAR state before each scan
        end else begin
            done <= 1'b0;

            case (state)
                // -------------------------------------------------------------
                ST_IDLE: begin
                    if (start) begin
                        linear_idx  <= '0;
                        linear_col  <= '0;
                        linear_row  <= '0;
                        total_cells <= ADDR_W'(grid_cols * grid_rows);
                        cur_blob_id <= '0;
                        sp          <= '0;
                        clr_addr    <= '0;
                        overflow    <= 1'b0;
                        out_valid   <= 1'b0;
                        state       <= ST_CLEAR;
                    end
                end

                // -------------------------------------------------------------
                // Clear visited[] (one entry per cycle)
                ST_CLEAR: begin
                    visited[clr_addr] <= 1'b0;
                    if (clr_addr == ADDR_W'(MAX_CELLS - 1)) begin
                        state <= ST_LINEAR_RD;
                    end else begin
                        clr_addr <= clr_addr + 1'b1;
                    end
                end

                // -------------------------------------------------------------
                // Linear scan: issue BRAM read (cell_addr driven combinationally)
                ST_LINEAR_RD: begin
                    state <= ST_LINEAR_WAIT;
                end

                // -------------------------------------------------------------
                // Linear scan: BRAM latency
                ST_LINEAR_WAIT: begin
                    state <= ST_LINEAR_CHK;
                end

                // -------------------------------------------------------------
                // Linear scan: check if this cell is a DFS seed
                ST_LINEAR_CHK: begin
                    if (!visited[linear_idx] && cell_data[79:60] != 20'h0) begin
                        // Non-empty unvisited cell: new connected component
                        if (cur_blob_id >= (BLOB_W+1)'(MAX_BLOBS)) begin
                            overflow <= 1'b1;
                            state    <= ST_DONE;
                        end else begin
                            state <= ST_DFS_PUSH;
                        end
                    end else begin
                        // Empty or already visited: advance linear scan
                        if (linear_idx + 1'b1 >= total_cells) begin
                            state <= ST_DONE;
                        end else begin
                            linear_idx <= linear_idx + 1'b1;
                            if (linear_col + 11'h1 >= grid_cols) begin
                                linear_col <= '0;
                                linear_row <= linear_row + 11'h1;
                            end else begin
                                linear_col <= linear_col + 11'h1;
                            end
                            state      <= ST_LINEAR_RD;
                        end
                    end
                end

                // -------------------------------------------------------------
                // Push seed cell onto stack and mark visited
                ST_DFS_PUSH: begin
                    if (!stack_full) begin
                        stack[sp]           <= linear_idx;
                        stack_col[sp]       <= linear_col;
                        stack_row[sp]       <= linear_row;
                        sp                  <= sp + 1'b1;
                        visited[linear_idx] <= 1'b1;
                        state               <= ST_DFS_POP;
                    end
                    // If stack full (shouldn't happen on seed), overflow and done
                    else begin
                        overflow <= 1'b1;
                        state    <= ST_DONE;
                    end
                end

                // -------------------------------------------------------------
                // Pop from stack; issue BRAM read; or end component
                ST_DFS_POP: begin
                    if (stack_empty) begin
                        // Component done; bump blob_id; continue linear scan
                        cur_blob_id <= cur_blob_id + 1'b1;
                        if (linear_idx + 1'b1 >= total_cells) begin
                            state <= ST_DONE;
                        end else begin
                            linear_idx <= linear_idx + 1'b1;
                            if (linear_col + 11'h1 >= grid_cols) begin
                                linear_col <= '0;
                                linear_row <= linear_row + 11'h1;
                            end else begin
                                linear_col <= linear_col + 11'h1;
                            end
                            state      <= ST_LINEAR_RD;
                        end
                    end else begin
                        // cell_addr is driven from stack[sp-1] combinationally
                        dfs_cell <= stack[sp - 1'b1];
                        dfs_col  <= stack_col[sp - 1'b1];
                        dfs_row  <= stack_row[sp - 1'b1];
                        sp       <= sp - 1'b1;
                        state    <= ST_DFS_WAIT;
                    end
                end

                // -------------------------------------------------------------
                // DFS: wait for BRAM latency (col/row already loaded from stack)
                ST_DFS_WAIT: begin
                    state <= ST_DFS_CHK;
                end

                // -------------------------------------------------------------
                // DFS: skip empty cells (can happen if neighbor was pushed
                // before we knew its count; we check on pop)
                ST_DFS_CHK: begin
                    if (cell_data[79:60] == 20'h0) begin
                        // Empty cell: skip; pop next
                        state <= ST_DFS_POP;
                    end else begin
                        state <= ST_DFS_EMIT;
                    end
                end

                // -------------------------------------------------------------
                // Emit cell to blob_emitter; stall if back-pressure
                ST_DFS_EMIT: begin
                    if (!out_valid) begin
                        // First presentation
                        out_valid      <= 1'b1;
                        out_cell_idx   <= dfs_cell;
                        out_cell_col   <= dfs_col;
                        out_cell_row   <= dfs_row;
                        out_blob_id    <= cur_blob_id;
                        out_cell_count <= cell_data[79:60];
                        out_cell_sum_x <= cell_data[59:28];
                        out_cell_sum_y <= cell_data[27: 0];
                    end
                    if (out_valid && out_ready) begin
                        out_valid  <= 1'b0;
                        nbr_phase  <= 2'd0;
                        state      <= ST_DFS_NBRS;
                    end
                end

                // -------------------------------------------------------------
                // Push 4-connected unvisited neighbors (one direction per cycle)
                ST_DFS_NBRS: begin
                    // Push neighbor if geo-valid and unvisited
                    if (nbr_geo_valid && !visited[nbr_idx]) begin
                        if (!stack_full) begin
                            stack[sp]        <= nbr_idx;
                            stack_col[sp]    <= nbr_col;
                            stack_row[sp]    <= nbr_row;
                            sp               <= sp + 1'b1;
                            visited[nbr_idx] <= 1'b1;
                        end else begin
                            overflow <= 1'b1;
                        end
                    end

                    if (nbr_phase == 2'd3) begin
                        // All four directions done; pop next cell
                        nbr_phase <= 2'd0;
                        state     <= ST_DFS_POP;
                    end else begin
                        nbr_phase <= nbr_phase + 1'b1;
                    end
                end

                // -------------------------------------------------------------
                ST_DONE: begin
                    done  <= 1'b1;
                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
