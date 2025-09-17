cpu_sim.3.multicore_full_sim.sv`timescale 1ns/1ps

module multicore_full_sim;

    // ----------------------------
    // Clock & Reset
    // ----------------------------
    logic clk;
    logic rst_n;

    initial begin
        clk = 0;
        forever #5 clk = ~clk;  // 100 MHz simulation clock
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
    // Define memory request/response structs
    // ----------------------------
    typedef struct packed {
        logic valid;
        logic wr;
        logic [31:0] addr;
        logic [31:0] wdata;
        logic atomic;
    } mem_req_t;

    typedef struct packed {
        logic valid;
        logic [31:0] rdata;
        logic sc_success;
    } mem_resp_t;

    // ----------------------------
    // Memory interfaces per core
    // ----------------------------
    mem_req_t  mem_req0, mem_req1;
    mem_resp_t mem_resp0, mem_resp1;

    // ----------------------------
    // L1 Caches per Core
    // ----------------------------
    unified_cache_llsc l1_cache0 (
        .clk(clk),
        .rst_n(rst_n),
        .mem_req(mem_req0),
        .mem_resp(mem_resp0),
        .core_id(2'b00)
    );

    unified_cache_llsc l1_cache1 (
        .clk(clk),
        .rst_n(rst_n),
        .mem_req(mem_req1),
        .mem_resp(mem_resp1),
        .core_id(2'b01)
    );

    // ----------------------------
    // Core Pipelines
    // ----------------------------
    core_pipeline core0 (
        .clk(clk),
        .rst_n(rst_n),
        .pc_in(pc_core0),
        .instr(instr_core0),
        .mem_if(mem_req0, mem_resp0)
    );

    core_pipeline core1 (
        .clk(clk),
        .rst_n(rst_n),
        .pc_in(pc_core1),
        .instr(instr_core1),
        .mem_if(mem_req1, mem_resp1)
    );

    // ----------------------------
    // Shared L2 Cache
    // ----------------------------
    l2_cache l2 (
        .clk(clk),
        .rst_n(rst_n),
        .mem_req0(mem_req0),
        .mem_resp0(mem_resp0),
        .mem_req1(mem_req1),
        .mem_resp1(mem_resp1)
    );

endmodule
