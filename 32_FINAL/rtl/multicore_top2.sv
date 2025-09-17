`timescale 1ns/1ps
import mc_common_pkg::*;
module multicore_top2 #(parameter string PROGRAM_HEX="program.hex")(input logic clk,rst_n, output logic[NCORES-1:0] halted);
  localparam int N=NCORES;
  mem_req_t if_req[N]; mem_resp_t if_resp[N];
  mem_req_t d_req[N];  mem_resp_t d_resp[N];
  mem_req_t l1req[N];  mem_resp_t l1rsp[N];
  mem_req_t l2_req;    mem_resp_t l2_resp;
  logic [31:0] instr[N];
  for (genvar c=0;c<N;c++) begin: G_IM
    instruction_memory #(.PROGRAM_FILE(PROGRAM_HEX)) instr_mem(.addr(if_req[c].addr), .data(instr[c]));
  end
  for (genvar c=0;c<N;c++) begin: G_L1
    unified_l1_llsc u_l1(.clk, .rst_n, .if_req(if_req[c]), .if_resp(if_resp[c]), .d_req(d_req[c]), .d_resp(d_resp[c]), .l2_req(l1req[c]), .l2_resp(l1rsp[c]));
  end
  for (genvar c=0;c<N;c++) begin: G_CORE
    core7 u_core(.clk, .rst_n,
      .if_req(if_req[c]), .if_resp('{valid:1'b1, rdata:instr[c], sc_success:1'b0}),
      .d_req(d_req[c]), .d_resp(d_resp[c]), .halted(halted[c]));
  end
  rr_arbiter #(.N(N)) u_rr(.clk, .rst_n, .req(l1req), .rsp(l1rsp), .l2_req(l2_req), .l2_resp(l2_resp));
  l2_cache_simple u_l2(.clk, .rst_n, .req(l2_req), .resp(l2_resp));
endmodule
