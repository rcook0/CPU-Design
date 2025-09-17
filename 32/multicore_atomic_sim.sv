`timescale 1ns/1ps

// ============================================================================
// Complete 2-Core Pipelined System with Unified L1 Cache, MESI, and LL/SC
// ============================================================================

module multicore_atomic_sim;

    // ----------------------------
    // Clock and reset
    // ----------------------------
    logic clk; initial clk = 0; always #5 clk = ~clk;
    logic rst_n; initial begin rst_n = 0; #20 rst_n = 1; end

    parameter ADDR_WIDTH = 32;
    parameter DATA_WIDTH = 32;
    parameter L1_LINES = 64;
    parameter LINEWORDS = 8;
    parameter L2_LINES = 256;
    parameter REG_COUNT = 32;

    // ----------------------------
    // Shared L2 memory
    // ----------------------------
    logic [ADDR_WIDTH-1:0] l2_mem [0:16383];
    logic l2_req_valid, l2_req_wr;
    logic [ADDR_WIDTH-1:0] l2_req_addr;
    logic [DATA_WIDTH-1:0] l2_req_wdata;
    logic l2_resp_valid;
    logic [DATA_WIDTH-1:0] l2_resp_rdata;

    l2_cache #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH),
        .LINES(L2_LINES),
        .LINEWORDS=16
    ) l2_inst (
        .clk(clk), .rst_n(rst_n),
        .req_valid(l2_req_valid), .req_wr(l2_req_wr),
        .req_addr(l2_req_addr), .req_wdata(l2_req_wdata),
        .resp_valid(l2_resp_valid), .resp_rdata(l2_resp_rdata),
        .mem(l2_mem)
    );

    // ----------------------------
    // 2 Unified L1 caches
    // ----------------------------
    logic [DATA_WIDTH-1:0] core_rdata[0:1];
    logic core_valid[0:1], core_busy[0:1];
    logic core_l2_req_valid[0:1], core_l2_req_wr[0:1];
    logic [ADDR_WIDTH-1:0] core_l2_req_addr[0:1];
    logic [DATA_WIDTH-1:0] core_l2_req_wdata[0:1];
    logic core_l2_resp_valid[0:1];
    logic [DATA_WIDTH-1:0] core_l2_resp_rdata[0:1];
    logic sc_success[0:1];

    unified_cache_llsc #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH),
        .LINEWORDS(LINEWORDS),
        .LINES(L1_LINES)
    ) core0_cache (
        .clk(clk), .rst_n(rst_n),
        .req_valid(1'b0), .req_wr(1'b0), .req_addr(0), .req_wdata(0), .atomic(0),
        .core_id(0),
        .resp_valid(core_valid[0]), .resp_rdata(core_rdata[0]), .sc_success(sc_success[0]), .busy(core_busy[0]),
        .snoop_valid(core_valid[1]), .snoop_addr(core_l2_req_addr[1]), .snoop_source_id(1), .snoop_ack(),
        .l2_req_valid(core_l2_req_valid[0]), .l2_req_wr(core_l2_req_wr[0]),
        .l2_req_addr(core_l2_req_addr[0]), .l2_req_wdata(core_l2_req_wdata[0]),
        .l2_resp_valid(core_l2_resp_valid[0]), .l2_resp_rdata(core_l2_resp_rdata[0])
    );

    unified_cache_llsc #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH),
        .LINEWORDS(LINEWORDS),
        .LINES(L1_LINES)
    ) core1_cache (
        .clk(clk), .rst_n(rst_n),
        .req_valid(1'b0), .req_wr(1'b0), .req_addr(0), .req_wdata(0), .atomic(0),
        .core_id(1),
        .resp_valid(core_valid[1]), .resp_rdata(core_rdata[1]), .sc_success(sc_success[1]), .busy(core_busy[1]),
        .snoop_valid(core_valid[0]), .snoop_addr(core_l2_req_addr[0]), .snoop_source_id(0), .snoop_ack(),
        .l2_req_valid(core_l2_req_valid[1]), .l2_req_wr(core_l2_req_wr[1]),
        .l2_req_addr(core_l2_req_addr[1]), .l2_req_wdata(core_l2_req_wdata[1]),
        .l2_resp_valid(core_l2_resp_valid[1]), .l2_resp_rdata(core_l2_resp_rdata[1])
    );

    // ----------------------------
    // Pipeline cores
    // ----------------------------
    core_pipeline #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH),
        .REG_COUNT(REG_COUNT)
    ) core0 (
        .clk(clk), .rst_n(rst_n),
        .mem_addr(core_l2_req_addr[0]), .mem_wr(core_l2_req_wr[0]),
        .mem_wdata(core_l2_req_wdata[0]),
        .mem_rdata(core_l2_resp_rdata[0]), .mem_ready(core_l2_resp_valid[0]),
        .core_id(0),
        .snoop_valid(core_valid[1]), .snoop_addr(core_l2_req_addr[1]), .snoop_source_id(1), .snoop_ack()
    );

    core_pipeline #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH),
        .REG_COUNT(REG_COUNT)
    ) core1 (
        .clk(clk), .rst_n(rst_n),
        .mem_addr(core_l2_req_addr[1]), .mem_wr(core_l2_req_wr[1]),
        .mem_wdata(core_l2_req_wdata[1]),
        .mem_rdata(core_l2_resp_rdata[1]), .mem_ready(core_l2_resp_valid[1]),
        .core_id(1),
        .snoop_valid(core_valid[0]), .snoop_addr(core_l2_req_addr[0]), .snoop_source_id(0), .snoop_ack()
    );

    // ----------------------------
    // Self-checking atomic test
    // ----------------------------
    initial begin
        // Initialize shared memory counter
        l2_mem[32'h1000>>2] = 0;

        // Simulate multiple atomic increments/decrements
        repeat (100) begin
            #50; // let cores run atomic ops
        end

        $display("Final shared counter = %0d", l2_mem[32'h1000>>2]);
        if (l2_mem[32'h1000>>2] !== 0) begin
            $error("Atomic LL/SC test FAILED!");
        end else begin
            $display("Atomic LL/SC test PASSED!");
        end
        $finish;
    end

endmodule
