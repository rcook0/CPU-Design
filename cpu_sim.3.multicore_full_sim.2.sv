`timescale 1ns/1ps

module multicore_full_sim;

    // ----------------------------
    // Clock & Reset
    // ----------------------------
    logic clk;
    logic rst_n;

    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 100 MHz simulation clock
    end

    initial begin
        rst_n = 0;
        #20 rst_n = 1;
    end

    // ----------------------------
    // Program Counters & Instructions
    // ----------------------------
    logic [31:0] pc_core0, pc_core1;
    logic [31:0] instr_core0, instr_core1;

    // ----------------------------
    // Instruction Memories
    // ----------------------------
    instruction_memory #(
        .PROGRAM_FILE("program.hex")
    ) instr_mem0 (
        .addr(pc_core0),
        .data(instr_core0)
    );

    instruction_memory #(
        .PROGRAM_FILE("program.hex")
    ) instr_mem1 (
        .addr(pc_core1),
        .data(instr_core1)
    );

    // ----------------------------
    // L1 Caches per Core
    // ----------------------------
    logic [31:0] l1_rdata0, l1_rdata1;
    logic l1_resp_valid0, l1_resp_valid1;
    logic l1_sc_success0, l1_sc_success1;

    unified_cache_llsc l1_cache0 (
        .clk(clk),
        .rst_n(rst_n),
        .req_valid(...),     // connected from core_pipeline
        .req_wr(...),
        .req_addr(...),
        .req_wdata(...),
        .atomic(...),
        .core_id(2'b00),
        .resp_valid(l1_resp_valid0),
        .resp_rdata(l1_rdata0),
        .sc_success(l1_sc_success0),
        .snoop_if(...)       // connect to coherence bus
    );

    unified_cache_llsc l1_cache1 (
        .clk(clk),
        .rst_n(rst_n),
        .req_valid(...),     // connected from core_pipeline
        .req_wr(...),
        .req_addr(...),
        .req_wdata(...),
        .atomic(...),
        .core_id(2'b01),
        .resp_valid(l1_resp_valid1),
        .resp_rdata(l1_rdata1),
        .sc_success(l1_sc_success1),
        .snoop_if(...)       // connect to coherence bus
    );

    // ----------------------------
    // Core Pipelines
    // ----------------------------
    core_pipeline core0 (
        .clk(clk),
        .rst_n(rst_n),
        .pc_in(pc_core0),
        .instr(instr_core0),
        .l1_cache_if(...)    // connect to l1_cache0
    );

    core_pipeline core1 (
        .clk(clk),
        .rst_n(rst_n),
        .pc_in(pc_core1),
        .instr(instr_core1),
        .l1_cache_if(...)    // connect to l1_cache1
    );

    // ----------------------------
    // Shared L2 Cache
    // ----------------------------
    l2_cache l2 (
        .clk(clk),
        .rst_n(rst_n),
        .req_valid(...),     // from L1 caches
        .req_wr(...),
        .req_addr(...),
        .req_wdata(...),
        .resp_valid(...),
        .resp_rdata(...),
        .mem(...)            // backing memory array
    );

    // ----------------------------
    // Optional: self-checking logic
    // ----------------------------
    // Can monitor the shared counter in L2 memory and assert correctness
    // Example:
    // always_ff @(posedge clk) begin
    //     if (l2.mem[0] > NUM_ITER) $fatal("Counter exceeded expected value");
    // end

endmodule
