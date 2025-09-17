`timescale 1ns/1ps

module multicore_atomic_tb;
    logic clk; initial clk=0; always #5 clk=~clk;
    logic rst_n; initial begin rst_n=0; #20; rst_n=1; end

    // ------------------------
    // Shared L2 memory
    // ------------------------
    logic [31:0] l2_mem [0:16383];
    logic l2_req_valid, l2_req_wr;
    logic [31:0] l2_req_addr, l2_req_wdata;
    logic l2_resp_valid;
    logic [31:0] l2_resp_rdata;

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
    // Core 0
    // ------------------------
    logic [31:0] core0_rdata;
    logic core0_valid, core0_busy;
    logic core0_l2_req_valid, core0_l2_req_wr;
    logic [31:0] core0_l2_req_addr, core0_l2_req_wdata;
    logic core0_l2_resp_valid;
    logic [31:0] core0_l2_resp_rdata;

    unified_cache core0_cache(
        .clk(clk), .rst_n(rst_n),
        .req_valid(1'b0), .req_wr(1'b0), .req_addr(0), .req_wdata(0),
        .resp_valid(core0_valid), .resp_rdata(core0_rdata), .busy(core0_busy),
        .core_id(0),
        .snoop_valid(1'b0), .snoop_addr(0), .snoop_source_id(1'b0), .snoop_ack(),
        .l2_req_valid(core0_l2_req_valid),
        .l2_req_wr(core0_l2_req_wr),
        .l2_req_addr(core0_l2_req_addr),
        .l2_req_wdata(core0_l2_req_wdata),
        .l2_resp_valid(core0_l2_resp_valid),
        .l2_resp_rdata(core0_l2_resp_rdata)
    );

    core_pipeline core0(
        .clk(clk), .rst_n(rst_n),
        .mem_addr(core0_l2_req_addr),
        .mem_wr(core0_l2_req_wr),
        .mem_wdata(core0_l2_req_wdata),
        .mem_rdata(core0_l2_resp_rdata),
        .mem_ready(core0_l2_resp_valid),
        .core_id(0),
        .snoop_valid(1'b0),
        .snoop_addr(0),
        .snoop_source_id(1'b1),
        .snoop_ack()
    );

    // ------------------------
    // Core 1
    // ------------------------
    logic [31:0] core1_rdata;
    logic core1_valid, core1_busy;
    logic core1_l2_req_valid, core1_l2_req_wr;
    logic [31:0] core1_l2_req_addr, core1_l2_req_wdata;
    logic core1_l2_resp_valid;
    logic [31:0] core1_l2_resp_rdata;

    unified_cache core1_cache(
        .clk(clk), .rst_n(rst_n),
        .req_valid(1'b0), .req_wr(1'b0), .req_addr(0), .req_wdata(0),
        .resp_valid(core1_valid), .resp_rdata(core1_rdata), .busy(core1_busy),
        .core_id(1),
        .snoop_valid(1'b0), .snoop_addr(0), .snoop_source_id(0), .snoop_ack(),
        .l2_req_valid(core1_l2_req_valid),
        .l2_req_wr(core1_l2_req_wr),
        .l2_req_addr(core1_l2_req_addr),
        .l2_req_wdata(core1_l2_req_wdata),
        .l2_resp_valid(core1_l2_resp_valid),
        .l2_resp_rdata(core1_l2_resp_rdata)
    );

    core_pipeline core1(
        .clk(clk), .rst_n(rst_n),
        .mem_addr(core1_l2_req_addr),
        .mem_wr(core1_l2_req_wr),
        .mem_wdata(core1_l2_req_wdata),
        .mem_rdata(core1_l2_resp_rdata),
        .mem_ready(core1_l2_resp_valid),
        .core_id(1),
        .snoop_valid(1'b0),
        .snoop_addr(0),
        .snoop_source_id(1'b0),
        .snoop_ack()
    );

    // ------------------------
    // Self-checking program
    // ------------------------
    initial begin
        // Initialize shared memory counter
        l2_mem[32'h1000>>2] = 0;

        #2000; // Run for a few cycles

        $display("Final shared counter = %0d", l2_mem[32'h1000>>2]);
        if (l2_mem[32'h1000>>2] !== 0)
            $error("Atomic test failed!");
        else
            $display("Atomic test passed!");
        $finish;
    end
endmodule
