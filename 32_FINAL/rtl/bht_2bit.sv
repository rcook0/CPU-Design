`timescale 1ns/1ps
module bht_2bit #(parameter ENTRIES=64)(input logic clk,rst_n,input logic[31:0] pc,output logic predict_taken,
input logic resolve_valid,input logic[31:0] resolve_pc,input logic resolve_taken);
  localparam IDXW=$clog2(ENTRIES); logic[1:0] table[0:ENTRIES-1];
  function automatic [IDXW-1:0] idx(input logic[31:0] a); return a[IDXW+2-1:2]; endfunction
  assign predict_taken=(table[idx(pc)]>=2);
  always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) for (int i=0;i<ENTRIES;i++) table[i]<=2;
    else if(resolve_valid) begin automatic [IDXW-1:0] i=idx(resolve_pc);
      if(resolve_taken) begin if(table[i]!=3) table[i]<=table[i]+1; end
      else begin if(table[i]!=0) table[i]<=table[i]-1; end
    end
  end
endmodule
