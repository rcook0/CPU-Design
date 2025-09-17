`timescale 1ns/1ps
import mc_common_pkg::*;
module rr_arbiter #(parameter int N=2)(input logic clk,rst_n, input mem_req_t req[N], output mem_resp_t rsp[N],
  output mem_req_t l2_req, input mem_resp_t l2_resp);
  logic [$clog2(N)-1:0] ptr, grant;
  always_comb begin grant=ptr; for (int i=0;i<N;i++) begin int k=(ptr+i)%N; if(req[k].valid) begin grant=k[$clog2(N)-1:0]; break; end end end
  assign l2_req = req[grant];
  for (genvar i=0;i<N;i++) begin : G assign rsp[i] = (i==grant)? l2_resp : '{default:'0}; end
  always_ff @(posedge clk or negedge rst_n) begin if(!rst_n) ptr<='0; else if(l2_req.valid) ptr <= (grant==N-1)?'0:(grant+1); end
endmodule
