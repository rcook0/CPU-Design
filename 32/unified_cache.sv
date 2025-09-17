`timescale 1ns/1ps

module unified_cache #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32,
    parameter LINEWORDS  = 8,
    parameter LINES      = 256,
    parameter MISS_PENALTY = 8
)(
    input  logic clk, rst_n,
    // CPU side
    input  logic         req_valid,
    input  logic         req_wr,
    input  logic [ADDR_WIDTH-1:0] req_addr,
    input  logic [DATA_WIDTH-1:0] req_wdata,
    output logic         resp_valid,
    output logic [DATA_WIDTH-1:0] resp_rdata,
    output logic         busy,
    input  logic [1:0]   core_id,
    // snoop/coherence bus
    input  logic         snoop_valid,
    input  logic [ADDR_WIDTH-1:0] snoop_addr,
    input  logic [1:0]   snoop_source_id,
    output logic         snoop_ack,
    // L2 interface
    output logic         l2_req_valid,
    output logic         l2_req_wr,
    output logic [ADDR_WIDTH-1:0] l2_req_addr,
    output logic [DATA_WIDTH-1:0] l2_req_wdata,
    input  logic         l2_resp_valid,
    input  logic [DATA_WIDTH-1:0] l2_resp_rdata
);

    typedef enum logic [1:0] {M=2'b00,S=2'b01,E=2'b10,I=2'b11} mesi_t;

    localparam INDEX_BITS = $clog2(LINES);
    localparam OFFSET_BITS = $clog2(LINEWORDS);
    localparam TAG_BITS = ADDR_WIDTH - INDEX_BITS - OFFSET_BITS - 2;

    typedef struct packed {
        logic [TAG_BITS-1:0] tag;
        mesi_t state;
        logic [DATA_WIDTH-1:0] data [0:LINEWORDS-1];
    } cache_line_t;

    cache_line_t lines[0:LINES-1];

    // pipeline/CPU access simulation
    logic [31:0] pending_addr;
    logic pending_wr;
    logic [DATA_WIDTH-1:0] pending_wdata;
    logic [INDEX_BITS-1:0] pending_index;
    mesi_t pending_state;
    typedef enum logic [1:0] {IDLE, WAIT_L2, REFILL} state_t;
    state_t state;

    integer i,j;
    initial begin
        for (i=0;i<LINES;i=i+1) begin
            lines[i].tag = 0;
            lines[i].state = I;
            for (j=0;j<LINEWORDS;j=j+1) lines[i].data[j] = 0;
        end
        state = IDLE;
        resp_valid = 0;
        resp_rdata = 0;
        busy = 0;
        snoop_ack = 0;
    end

    // Address decoding
    wire [31:0] word_addr = req_addr >> 2;
    wire [OFFSET_BITS-1:0] offset = word_addr[OFFSET_BITS-1:0];
    wire [INDEX_BITS-1:0] index = word_addr[OFFSET_BITS +: INDEX_BITS];
    wire [TAG_BITS-1:0] in_tag = word_addr[OFFSET_BITS+INDEX_BITS +: TAG_BITS];

    // Simplified state machine
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            resp_valid <= 0;
            busy <= 0;
        end else begin
            snoop_ack <= 0;
            if (snoop_valid && snoop_source_id != core_id) begin
                if (lines[index].tag == in_tag && lines[index].state != I) begin
                    lines[index].state <= I;
                end
                snoop_ack <= 1;
            end

            case (state)
                IDLE: begin
                    resp_valid <= 0;
                    busy <= 0;
                    if (req_valid) begin
                        if (lines[index].tag == in_tag && lines[index].state != I) begin
                            // HIT
                            if (req_wr) begin
                                lines[index].data[offset] <= req_wdata;
                                lines[index].state <= M; // mark modified
                            end else resp_rdata <= lines[index].data[offset];
                            resp_valid <= 1;
                        end else begin
                            // MISS → request L2
                            pending_addr <= req_addr;
                            pending_wr <= req_wr;
                            pending_wdata <= req_wdata;
                            pending_index <= index;
                            state <= WAIT_L2;
                            busy <= 1;
                            l2_req_valid <= 1;
                            l2_req_addr <= req_addr;
                            l2_req_wr <= req_wr;
                            l2_req_wdata <= req_wdata;
                        end
                    end
                end
                WAIT_L2: begin
                    if (l2_resp_valid) begin
                        // Fill cache line
                        for (j=0;j<LINEWORDS;j=j+1) lines[pending_index].data[j] <= l2_resp_rdata; // simplified
                        lines[pending_index].tag <= in_tag;
                        lines[pending_index].state <= (pending_wr ? M : E);
                        resp_rdata <= l2_resp_rdata;
                        resp_valid <= 1;
                        state <= IDLE;
                        busy <= 0;
                        l2_req_valid <= 0;
                    end
                end
                default: state <= IDLE;
            endcase
        end
    end
endmodule

// -----------------------------------------------------------------------------
// Shared L2 cache (direct-mapped, write-back, simple)
// -----------------------------------------------------------------------------
module l2_cache #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32,
    parameter LINEWORDS = 16,
    parameter LINES = 512
)(
    input logic clk, rst_n,
    input logic req_valid,
    input logic req_wr,
    input logic [ADDR_WIDTH-1:0] req_addr,
    input logic [DATA_WIDTH-1:0] req_wdata,
    output logic resp_valid,
    output logic [DATA_WIDTH-1:0] resp_rdata,
    output logic [DATA_WIDTH-1:0] mem [0:16383]
);
    integer i;
    initial begin
        for (i=0;i<16384;i=i+1) mem[i]=0;
        resp_valid=0;
        resp_rdata=0;
    end

    // Very simple direct access
    always_ff @(posedge clk) begin
        resp_valid <= 0;
        if (req_valid) begin
            if (req_wr) mem[req_addr>>2] <= req_wdata;
            resp_rdata <= mem[req_addr>>2];
            resp_valid <= 1;
        end
    end
endmodule

// -----------------------------------------------------------------------------
// Core skeleton (2 cores), placeholder pipeline, unified L1 caches
// -----------------------------------------------------------------------------
module multicore_top;
    logic clk; initial clk=0; always #5 clk=~clk;
    logic rst_n; initial begin rst_n=0; #20; rst_n=1; end

    // L2 cache
    logic [31:0] l2_mem [0:16383];
    l2_cache l2(.clk(clk), .rst_n(rst_n),
        .req_valid(l2_req_valid), .req_wr(l2_req_wr),
        .req_addr(l2_req_addr), .req_wdata(l2_req_wdata),
        .resp_valid(l2_resp_valid), .resp_rdata(l2_resp_rdata),
        .mem(l2_mem));

    // Two cores unified L1 caches
    logic [31:0] core0_rdata, core1_rdata;
    logic core0_valid, core1_valid, core0_busy, core1_busy;
    logic core0_l2_req_valid, core0_l2_req_wr;
    logic [31:0] core0_l2_req_addr, core0_l2_req_wdata;
    logic core0_l2_resp_valid;
    logic [31:0] core0_l2_resp_rdata;

    unified_cache core0(.clk(clk), .rst_n(rst_n),
        .req_valid(1'b0), .req_wr(1'b0), .req_addr(0), .req_wdata(0),
        .resp_valid(core0_valid), .resp_rdata(core0_rdata), .busy(core0_busy),
        .core_id(0),
        .snoop_valid(1'b0), .snoop_addr(0), .snoop_source_id(1'b0), .snoop_ack(),
        .l2_req_valid(core0_l2_req_valid), .l2_req_wr(core0_l2_req_wr),
        .l2_req_addr(core0_l2_req_addr), .l2_req_wdata(core0_l2_req_wdata),
        .l2_resp_valid(core0_l2_resp_valid), .l2_resp_rdata(core0_l2_resp_rdata)
    );

    unified_cache core1(.clk(clk), .rst_n(rst_n),
        .req_valid(1'b0), .req_wr(1'b0), .req_addr(0), .req_wdata(0),
        .resp_valid(core1_valid), .resp_rdata(core1_rdata), .busy(core1_busy),
        .core_id(1),
        .snoop_valid(1'b0), .snoop_addr(0), .snoop_source_id(0), .snoop_ack(),
        .l2_req_valid(core0_l2_req_valid), .l2_req_wr(core0_l2_req_wr),
        .l2_req_addr(core0_l2_req_addr), .l2_req_wdata(core0_l2_req_wdata),
        .l2_resp_valid(core0_l2_resp_valid), .l2_resp_rdata(core0_l2_resp_rdata)
    );

endmodule
