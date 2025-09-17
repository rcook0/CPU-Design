// Core 0 instruction fetch
instr_mem0 imem0(.addr(pc_core0), .data(instr_core0));

// Core 1 instruction fetch
instr_mem1 imem1(.addr(pc_core1), .data(instr_core1));

// Core 0 -> L1
core_pipeline core0(
    .clk(clk), .rst_n(rst_n),
    .pc_in(pc_core0), .instr(instr_core0),
    .mem_req_valid(mem_req_valid0),
    .mem_req_addr(mem_req_addr0),
    .mem_req_wdata(mem_req_wdata0),
    .mem_req_wr(mem_req_wr0),
    .mem_req_atomic(mem_req_atomic0),
    .mem_rdata(resp_rdata0),
    .mem_valid(resp_valid0),
    .sc_success(sc_success0)
);

unified_cache_llsc l1_cache0(
    .clk(clk), .rst_n(rst_n),
    .req_valid(mem_req_valid0),
    .req_addr(mem_req_addr0),
    .req_wdata(mem_req_wdata0),
    .req_wr(mem_req_wr0),
    .atomic(mem_req_atomic0),
    .resp_rdata(resp_rdata0),
    .resp_valid(resp_valid0),
    .sc_success(sc_success0),
    .snoop_if(...)
);

// Repeat similarly for Core 1 -> L1
