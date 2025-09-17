`timescale 1ns/1ps
import mc_common_pkg::*;
module btb_dm #(parameter ENTRIES=64)(input logic clk,rst_n,input logic[31:0] pc,output logic hit,output logic[31:0] target,
input logic update,input logic[31:0] pc_u,input logic[31:0] target_u);
  localparam IDXW=$clog2(ENTRIES);
  logic [31:0] tag [0:ENTRIES-1]; logic [31:0] tgt [0:ENTRIES-1]; logic vld[0:ENTRIES-1];
  function automatic [IDXW-1:0] idx(input logic[31:0] a); return a[IDXW+2-1:2]; endfunction
  wire [IDXW-1:0] i_rd=idx(pc); assign hit=vld[i_rd]&&(tag[i_rd]==pc); assign target=tgt[i_rd];
  always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) for (int i=0;i<ENTRIES;i++) begin vld[i]<=0; tag[i]<=0; tgt[i]<=0; end
    else if(update) begin automatic [IDXW-1:0] i=idx(pc_u); vld[i]<=1; tag[i]<=pc_u; tgt[i]<=target_u; end
  end
endmodule
