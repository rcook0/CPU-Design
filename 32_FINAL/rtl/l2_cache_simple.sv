`timescale 1ns/1ps
import mc_common_pkg::*;
module l2_cache_simple #(parameter int WORDS=1<<16, parameter string PROGRAM_HEX="program.hex", parameter string DATA_HEX="data_init.hex")
( input logic clk,rst_n, input mem_req_t req, output mem_resp_t resp );
  xlen_t mem[0:WORDS-1];
  initial begin if(PROGRAM_HEX!="") $readmemh(PROGRAM_HEX, mem); if(DATA_HEX!="") $readmemh(DATA_HEX, mem); end
  xlen_t r_q; logic v_q;
  always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin v_q<=0; r_q<='0; end
    else begin v_q<=req.valid; if(req.valid) begin if(req.wr) mem[req.addr[31:2]]<=req.wdata; r_q<=mem[req.addr[31:2]]; end end
  end
  assign resp = '{valid:v_q, rdata:r_q, sc_success:(req.atomic && req.wr)};
endmodule
