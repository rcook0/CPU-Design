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

// -----------------------------------------------------------------------------
// Self-check: after N cycles, the shared counter at 0x1000 must be 0.
// Adjust CYCLES as needed to comfortably exceed your program length.
// -----------------------------------------------------------------------------
localparam int CYCLES = 200_000;  // sim budget

initial begin
  // VCD (optional)
  $dumpfile("multicore_full_sim.vcd");
  $dumpvars(0, multicore_full_sim);

  // Wait for reset deassert
  @(negedge rst_n);
  @(posedge rst_n);

  // Let the program run
  repeat (CYCLES) @(posedge clk);

  int final_val = l2.mem[32'h1000 >> 2];  // uses l2_cache's exposed mem[]
  $display("[SELF-CHECK] Final shared counter = %0d", final_val);

  if (final_val !== 0) begin
    $error("[FAIL] Atomic LL/SC test failed: expected 0, got %0d", final_val);
    $fatal(1);
  end else begin
    $display("[PASS] Atomic LL/SC test passed: counter == 0");
  end

    // Never allow absurd values (quick sanity while running)
    always @(posedge clk) begin
      assert (l2.mem[32'h1000 >> 2] < 1_000_000)
        else $fatal(2, "[ASSERT] Counter exploded: %0d", l2.mem[32'h1000 >> 2]);
    end
    
  $finish;
end


endmodule
