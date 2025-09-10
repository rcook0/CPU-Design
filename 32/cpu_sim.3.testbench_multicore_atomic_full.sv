`timescale 1ns/1ps

module multicore_atomic_full_tb;

    // ------------------------
    // Clock and reset
    // ------------------------
    logic clk;
    initial clk = 0; always #5 clk = ~clk;

    logic rst_n;
    initial begin
        rst_n = 0;
        #20 rst_n = 1;
    end

    // ------------------------
    // Parameters
    // ------------------------
    parameter ADDR_WIDTH = 32;
    parameter DATA_WIDTH = 32;
    parameter LINEWORDS = 8;
    parameter L1_LINES = 256;
    parameter L2_LINES = 512;
    parameter REG_COUNT = 32;

    // ------------------------
    // Shared L2 memory
    // ------------------------
    logic [ADDR_WIDTH-1:0] l2_mem [0:16383];
    logic l2_req_valid, l2_req_wr;
    logic [ADDR_WIDTH-1:0] l2_req_addr;
    logic [DATA_WIDTH-1:0] l2_req_wdata;
    logic l2_resp_valid;
    logic [DATA_WIDTH-1:0] l2_resp_rdata;

    l2_cache l2(
        .clk(clk), .rst_n(rst_n),
        .req_valid(l2_req_valid),
        .req_wr(l2_req_wr),
        .req_addr(l2_req_addr),
        .req_wdata(l2_req_wdata),
        .resp_valid(l2_resp_valid),
        .resp_rdata(l2_resp_rdata),
        .mem(l2_mem)
    );

    // ------------------------
    // 2-core unified L1 caches
    // ------------------------
    logic [DATA_WIDTH-1:0] core_rdata[0:1];
    logic core_valid[0:1], core_busy[0:1];
    logic core_l2_req_valid[0:1], core_l2_req_wr[0:1];
    logic [ADDR_WIDTH-1:0] core_l2_req_addr[0:1];
    logic [DATA_WIDTH-1:0] core_l2_req_wdata[0:1];
    logic core_l2_resp_valid[0:1];
    logic [DATA_WIDTH-1:0] core_l2_resp_rdata[0:1];

    unified_cache core0_cache(
        .clk(clk), .rst_n(rst_n),
        .req_valid(1'b0), .req_wr(1'b0), .req_addr(0), .req_wdata(0),
        .resp_valid(core_valid[0]), .resp_rdata(core_rdata[0]), .busy(core_busy[0]),
        .core_id(0),
        .snoop_valid(core_valid[1]), .snoop_addr(core_l2_req_addr[1]),
        .snoop_source_id(1), .snoop_ack(),
        .l2_req_valid(core_l2_req_valid[0]), .l2_req_wr(core_l2_req_wr[0]),
        .l2_req_addr(core_l2_req_addr[0]), .l2_req_wdata(core_l2_req_wdata[0]),
        .l2_resp_valid(core_l2_resp_valid[0]), .l2_resp_rdata(core_l2_resp_rdata[0])
    );

    unified_cache core1_cache(
        .clk(clk), .rst_n(rst_n),
        .req_valid(1'b0), .req_wr(1'b0), .req_addr(0), .req_wdata(0),
        .resp_valid(core_valid[1]), .resp_rdata(core_rdata[1]), .busy(core_busy[1]),
        .core_id(1),
        .snoop_valid(core_valid[0]), .snoop_addr(core_l2_req_addr[0]),
        .snoop_source_id(0), .snoop_ack(),
        .l2_req_valid(core_l2_req_valid[1]), .l2_req_wr(core_l2_req_wr[1]),
        .l2_req_addr(core_l2_req_addr[1]), .l2_req_wdata(core_l2_req_wdata[1]),
        .l2_resp_valid(core_l2_resp_valid[1]), .l2_resp_rdata(core_l2_resp_rdata[1])
    );

    // ------------------------
    // Pipeline cores with LL/SC atomic ops
    // ------------------------
    core_pipeline #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH),
        .REG_COUNT(REG_COUNT)
    ) core0(
        .clk(clk), .rst_n(rst_n),
        .mem_addr(core_l2_req_addr[0]),
        .mem_wr(core_l2_req_wr[0]),
        .mem_wdata(core_l2_req_wdata[0]),
        .mem_rdata(core_l2_resp_rdata[0]),
        .mem_ready(core_l2_resp_valid[0]),
        .core_id(0),
        .snoop_valid(core_valid[1]),
        .snoop_addr(core_l2_req_addr[1]),
        .snoop_source_id(1),
        .snoop_ack()
    );

    core_pipeline #(
        .ADDR_WIDTH(ADDR_WIDTH),
        .DATA_WIDTH(DATA_WIDTH),
        .REG_COUNT(REG_COUNT)
    ) core1(
        .clk(clk), .rst_n(rst_n),
        .mem_addr(core_l2_req_addr[1]),
        .mem_wr(core_l2_req_wr[1]),
        .mem_wdata(core_l2_req_wdata[1]),
        .mem_rdata(core_l2_resp_rdata[1]),
        .mem_ready(core_l2_resp_valid[1]),
        .core_id(1),
        .snoop_valid(core_valid[0]),
        .snoop_addr(core_l2_req_addr[0]),
        .snoop_source_id(0),
        .snoop_ack()
    );

    // ------------------------
    // Self-checking atomic test program
    // ------------------------
    initial begin
        // Initialize shared memory location 0x1000
        l2_mem[32'h1000>>2] = 0;

        // Run simulation for multiple atomic increments/decrements
        #10000;

        $display("Final shared counter = %0d", l2_mem[32'h1000>>2]);
        if (l2_mem[32'h1000>>2] !== 0)
            $error("Atomic LL/SC test FAILED!");
        else
            $display("Atomic LL/SC test PASSED!");

        $finish;
    end

endmodule
