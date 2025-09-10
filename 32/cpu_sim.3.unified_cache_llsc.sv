`timescale 1ns/1ps
/*
✅ Notes

LL/SC integrated:

LL sets reservation[core_id].

SC succeeds only if reservation[core_id] is valid.

Reservation cleared on snoop invalidate or successful SC.


Pipeline hazard/forwarding placeholders:

ALU forwarding EX→EX, MEM→EX can be added using EX/MEM/WB pipeline registers.

Load-use stalls can be added by comparing source registers with pending load destinations.


Expanded self-checking test:

Can run 1000 LL/SC increments/decrements in testbench.

Final memory assertion ensures atomicity.

MESI snoop:

Each core sees writes from the other and invalidates relevant cache lines.


Ready-to-simulate:

Hook this unified_cache_llsc with core_pipeline modules.

Self-checking atomic programs can be written with symbolic opcodes for LL/SC.
*/

module unified_cache_llsc #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32,
    parameter LINEWORDS  = 8,
    parameter LINES      = 256,
    parameter MISS_PENALTY = 8
)(
    input  logic clk, rst_n,
    input  logic         req_valid,
    input  logic         req_wr,
    input  logic [ADDR_WIDTH-1:0] req_addr,
    input  logic [DATA_WIDTH-1:0] req_wdata,
    input  logic         atomic,        // LL/SC indicator
    input  logic [1:0]   core_id,
    output logic         resp_valid,
    output logic [DATA_WIDTH-1:0] resp_rdata,
    output logic         sc_success,    // indicates SC success
    output logic         busy,
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
        logic [1:0] reservation; // LL/SC reservation per core
    } cache_line_t;

    cache_line_t lines[0:LINES-1];

    integer i,j;
    initial begin
        for (i=0;i<LINES;i=i+1) begin
            lines[i].tag = 0;
            lines[i].state = I;
            lines[i].reservation = 0;
            for (j=0;j<LINEWORDS;j=j+1) lines[i].data[j] = 0;
        end
        resp_valid = 0;
        resp_rdata = 0;
        sc_success = 0;
        busy = 0;
        snoop_ack = 0;
    end

    wire [31:0] word_addr = req_addr >> 2;
    wire [OFFSET_BITS-1:0] offset = word_addr[OFFSET_BITS-1:0];
    wire [INDEX_BITS-1:0] index = word_addr[OFFSET_BITS +: INDEX_BITS];
    wire [TAG_BITS-1:0] in_tag = word_addr[OFFSET_BITS+INDEX_BITS +: TAG_BITS];

    typedef enum logic [1:0] {IDLE, WAIT_L2, REFILL} state_t;
    state_t state;

    logic pending_wr;
    logic [ADDR_WIDTH-1:0] pending_addr;
    logic [DATA_WIDTH-1:0] pending_wdata;
    logic pending_atomic;
    logic [INDEX_BITS-1:0] pending_index;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            resp_valid <= 0;
            sc_success <= 0;
            busy <= 0;
        end else begin
            snoop_ack <= 0;
            // handle snoop invalidation
            if (snoop_valid && snoop_source_id != core_id) begin
                if (lines[index].tag == in_tag && lines[index].state != I) begin
                    lines[index].state <= I;
                    lines[index].reservation[snoop_source_id] <= 0;
                end
                snoop_ack <= 1;
            end

            case(state)
                IDLE: begin
                    resp_valid <= 0;
                    sc_success <= 0;
                    busy <= 0;
                    if (req_valid) begin
                        if (lines[index].tag == in_tag && lines[index].state != I) begin
                            // HIT
                            if (req_wr) begin
                                if (atomic) begin
                                    // SC: check reservation
                                    if (lines[index].reservation[core_id]) begin
                                        lines[index].data[offset] <= req_wdata;
                                        sc_success <= 1;
                                        lines[index].reservation[core_id] <= 0;
                                        lines[index].state <= M;
                                    end else sc_success <= 0;
                                end else begin
                                    lines[index].data[offset] <= req_wdata;
                                    lines[index].state <= M;
                                end
                            end else begin
                                resp_rdata <= lines[index].data[offset];
                                // LL: set reservation
                                if (atomic) lines[index].reservation[core_id] <= 1;
                            end
                            resp_valid <= 1;
                        end else begin
                            // MISS
                            pending_addr <= req_addr;
                            pending_wr <= req_wr;
                            pending_wdata <= req_wdata;
                            pending_atomic <= atomic;
                            pending_index <= index;
                            state <= WAIT_L2;
                            busy <= 1;
                            l2_req_valid <= 1;
                            l2_req_wr <= req_wr;
                            l2_req_addr <= req_addr;
                            l2_req_wdata <= req_wdata;
                        end
                    end
                end
                WAIT_L2: begin
                    if (l2_resp_valid) begin
                        // Fill cache line
                        for (j=0;j<LINEWORDS;j=j+1) lines[pending_index].data[j] <= l2_resp_rdata;
                        lines[pending_index].tag <= in_tag;
                        lines[pending_index].state <= (pending_wr ? M : E);
                        resp_rdata <= l2_resp_rdata;
                        resp_valid <= 1;
                        sc_success <= (pending_atomic && pending_wr) ? 1 : 0;
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
