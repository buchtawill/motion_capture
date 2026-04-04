// frame_rate_counter.v
// AXI Stream sniffer IP for measuring camera frame rate.
//
// Sits passively between the MIPI CSI2 Rx subsystem output and AXI VDMA.
// All AXI-Stream signals are wired through transparently.
//
// Measurement procedure (when enabled):
//   1. Wait for first TUSER strobe (TVALID & TREADY & TUSER[0] all high) —
//      this is the AXI4-Stream Video start-of-frame marker.
//   2. Latch cycle counter = 0 at that strobe; begin incrementing each clock.
//   3. Count 100 subsequent TUSER strobes.
//   4. On the 100th strobe, stop the counter and assert done.
//
// AXI-Lite Register Map (word-addressed, 32-bit wide):
//   0x00  Control  [0]=enable  [1]=sw_reset (always reads 0; write 1 to reset)
//   0x04  Status   [0]=idle    [1]=busy     [2]=done
//   0x08  CycleCount — 32-bit elapsed clock cycles across 100 frames
//
// Both the AXI-Lite and AXI-Stream interfaces share a single clock (aclk).
// If the two datapaths run on different clocks, add CDC registers between
// the control/status registers and the measurement logic.

`timescale 1ns / 1ps

module frame_rate_counter #(
    parameter integer AXIS_DATA_WIDTH  = 32,
    parameter integer AXIS_TUSER_WIDTH = 1,
    parameter integer AXIS_TKEEP_WIDTH = AXIS_DATA_WIDTH / 8
)(
    // -------------------------------------------------------------------------
    // AXI-Lite clock / reset (also clocks the AXI-Stream sniffer logic)
    // -------------------------------------------------------------------------
    input  wire                           aclk,
    input  wire                           aresetn,

    // -------------------------------------------------------------------------
    // AXI-Lite Slave
    // -------------------------------------------------------------------------
    // Write address channel
    input  wire [3:0]                     s_axi_awaddr,
    input  wire                           s_axi_awvalid,
    output reg                            s_axi_awready,

    // Write data channel
    input  wire [31:0]                    s_axi_wdata,
    input  wire [3:0]                     s_axi_wstrb,
    input  wire                           s_axi_wvalid,
    output reg                            s_axi_wready,

    // Write response channel
    output reg  [1:0]                     s_axi_bresp,
    output reg                            s_axi_bvalid,
    input  wire                           s_axi_bready,

    // Read address channel
    input  wire [3:0]                     s_axi_araddr,
    input  wire                           s_axi_arvalid,
    output reg                            s_axi_arready,

    // Read data channel
    output reg  [31:0]                    s_axi_rdata,
    output reg  [1:0]                     s_axi_rresp,
    output reg                            s_axi_rvalid,
    input  wire                           s_axi_rready,

    // -------------------------------------------------------------------------
    // AXI-Stream Slave  (from MIPI CSI2 Rx subsystem)
    // -------------------------------------------------------------------------
    input  wire [AXIS_DATA_WIDTH-1:0]     s_axis_tdata,
    input  wire [9:0]                     s_axis_tdest,
    // input  wire [AXIS_TKEEP_WIDTH-1:0]    s_axis_tkeep, // Not present from CSI2 RX subsystem
    input  wire [AXIS_TUSER_WIDTH-1:0]    s_axis_tuser,
    input  wire                           s_axis_tlast,
    input  wire                           s_axis_tvalid,
    output wire                           s_axis_tready,

    // -------------------------------------------------------------------------
    // AXI-Stream Master  (to AXI VDMA)
    // -------------------------------------------------------------------------
    output wire [AXIS_DATA_WIDTH-1:0]     m_axis_tdata,
    output wire [AXIS_TKEEP_WIDTH-1:0]    m_axis_tkeep,
    output wire [AXIS_TUSER_WIDTH-1:0]    m_axis_tuser,
    output wire                           m_axis_tlast,
    output wire                           m_axis_tvalid,
    input  wire                           m_axis_tready
);

    // =========================================================================
    // AXI-Stream passthrough (pure wire — zero latency, zero backpressure)
    // =========================================================================
    assign m_axis_tdata  = s_axis_tdata;
    // assign m_axis_tkeep  = s_axis_tkeep;
    assign m_axis_tkeep  = 4'hF;
    assign m_axis_tuser  = s_axis_tuser;
    assign m_axis_tlast  = s_axis_tlast;
    assign m_axis_tvalid = s_axis_tvalid;
    assign s_axis_tready = m_axis_tready;

    // =========================================================================
    // Register addresses
    // =========================================================================
    localparam ADDR_CTRL        = 4'h0;   // 0x00
    localparam ADDR_STATUS      = 4'h4;   // 0x04
    localparam ADDR_CYCLE_COUNT = 4'h8;   // 0x08

    // =========================================================================
    // Control register fields
    // =========================================================================
    reg  reg_enable;     // bit 0 of control register
    reg  sw_reset_latch; // one-cycle pulse set by AXI write block, read by state machine

    // =========================================================================
    // Measurement state machine
    // =========================================================================
    localparam [1:0]
        S_IDLE     = 2'd0,   // not enabled
        S_WAIT_SOF = 2'd1,   // enabled, awaiting first TUSER strobe
        S_COUNTING = 2'd2,   // counting clock cycles
        S_DONE     = 2'd3;   // measurement complete

    reg  [1:0]  state;
    reg  [31:0] cycle_counter;      // free-running while in S_COUNTING
    reg  [6:0]  frame_counter;      // counts TUSER strobes (0..100)
    reg  [31:0] reg_cycle_count;    // latched result exposed to AXI-Lite

    // TUSER strobe: start-of-frame detected on the stream
    wire sof_strobe = s_axis_tvalid & m_axis_tready & s_axis_tuser[0];

    always @(posedge aclk) begin
        if (!aresetn || sw_reset_latch) begin
            state           <= S_IDLE;
            cycle_counter   <= 32'd0;
            frame_counter   <= 7'd0;
            reg_cycle_count <= 32'd0;
        end else begin
            case (state)
                // ----------------------------------------------------------
                S_IDLE: begin
                    cycle_counter <= 32'd0;
                    frame_counter <= 7'd0;
                    if (reg_enable)
                        state <= S_WAIT_SOF;
                end

                // ----------------------------------------------------------
                S_WAIT_SOF: begin
                    if (!reg_enable) begin
                        state <= S_IDLE;
                    end else if (sof_strobe) begin
                        // First frame boundary — start counting from this edge
                        cycle_counter <= 32'd0;
                        frame_counter <= 7'd0;
                        state         <= S_COUNTING;
                    end
                end

                // ----------------------------------------------------------
                S_COUNTING: begin
                    if (!reg_enable) begin
                        state <= S_IDLE;
                    end else begin
                        cycle_counter <= cycle_counter + 1'b1;
                        if (sof_strobe) begin
                            if (frame_counter == 7'd99) begin
                                // 100th strobe reached — latch and finish
                                reg_cycle_count <= cycle_counter + 1'b1;
                                state           <= S_DONE;
                            end else begin
                                frame_counter <= frame_counter + 1'b1;
                            end
                        end
                    end
                end

                // ----------------------------------------------------------
                S_DONE: begin
                    // Hold result until reset or re-enable
                    if (!reg_enable)
                        state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // Status bits derived from state
    wire status_idle = (state == S_IDLE) || (state == S_WAIT_SOF);
    wire status_busy = (state == S_COUNTING);
    wire status_done = (state == S_DONE);

    // =========================================================================
    // AXI-Lite write logic
    // =========================================================================
    // Latch write address / data together (simplest approach: accept both
    // channels independently, respond once both have arrived).
    reg        aw_active;
    reg [3:0]  aw_addr_lat;
    reg        w_active;
    reg [31:0] w_data_lat;
    reg [3:0]  w_strb_lat;

    always @(posedge aclk) begin
        if (!aresetn) begin
            s_axi_awready <= 1'b1;
            s_axi_wready  <= 1'b1;
            s_axi_bvalid  <= 1'b0;
            s_axi_bresp   <= 2'b00;
            aw_active     <= 1'b0;
            w_active      <= 1'b0;
            aw_addr_lat   <= 4'd0;
            w_data_lat    <= 32'd0;
            w_strb_lat    <= 4'd0;
            reg_enable     <= 1'b0;
            sw_reset_latch <= 1'b0;
        end else begin
            sw_reset_latch <= 1'b0; // default; overridden below when sw_reset written

            // Accept write address
            if (s_axi_awvalid && s_axi_awready) begin
                aw_addr_lat   <= s_axi_awaddr;
                aw_active     <= 1'b1;
                s_axi_awready <= 1'b0;
            end

            // Accept write data
            if (s_axi_wvalid && s_axi_wready) begin
                w_data_lat   <= s_axi_wdata;
                w_strb_lat   <= s_axi_wstrb;
                w_active     <= 1'b1;
                s_axi_wready <= 1'b0;
            end

            // Issue response once both address and data are captured
            if (aw_active && w_active && !s_axi_bvalid) begin
                s_axi_bresp  <= 2'b00; // OKAY
                s_axi_bvalid <= 1'b1;
                aw_active    <= 1'b0;
                w_active     <= 1'b0;

                // Perform register write
                if (aw_addr_lat == ADDR_CTRL) begin
                    // bit 0: enable
                    if (w_strb_lat[0]) begin
                        reg_enable <= w_data_lat[0];
                        // bit 1: sw_reset — reset entire state machine
                        if (w_data_lat[1]) begin
                            reg_enable <= 1'b0;
                            // state machine reset is handled below
                        end
                    end
                end
                // Status and CycleCount are read-only; writes are silently ignored
            end

            // sw_reset: pulse latch for one cycle so the state machine block
            // (the sole owner of state/counters) can reset itself cleanly.
            if (aw_active && w_active && !s_axi_bvalid &&
                (aw_addr_lat == ADDR_CTRL) && w_strb_lat[0] && w_data_lat[1]) begin
                sw_reset_latch <= 1'b1;
            end

            // Deassert bvalid after handshake
            if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid  <= 1'b0;
                s_axi_awready <= 1'b1;
                s_axi_wready  <= 1'b1;
            end
        end
    end

    // =========================================================================
    // AXI-Lite read logic
    // =========================================================================
    always @(posedge aclk) begin
        if (!aresetn) begin
            s_axi_arready <= 1'b1;
            s_axi_rvalid  <= 1'b0;
            s_axi_rresp   <= 2'b00;
            s_axi_rdata   <= 32'd0;
        end else begin
            if (s_axi_arvalid && s_axi_arready) begin
                s_axi_arready <= 1'b0;
                s_axi_rresp   <= 2'b00;
                s_axi_rvalid  <= 1'b1;

                case (s_axi_araddr)
                    ADDR_CTRL: begin
                        // bit 1 (sw_reset) always reads 0
                        s_axi_rdata <= {30'd0, 1'b0, reg_enable};
                    end
                    ADDR_STATUS: begin
                        s_axi_rdata <= {29'd0, status_done, status_busy, status_idle};
                    end
                    ADDR_CYCLE_COUNT: begin
                        s_axi_rdata <= reg_cycle_count;
                    end
                    default: begin
                        s_axi_rdata <= 32'hDEAD_BEEF;
                        s_axi_rresp <= 2'b10; // SLVERR
                    end
                endcase
            end

            if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid  <= 1'b0;
                s_axi_arready <= 1'b1;
            end
        end
    end

endmodule
