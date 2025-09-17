`timescale 1ns/1ps
module interlock_unit #(parameter REGW=5)(input logic v_mem,input logic[REGW-1:0] rd_mem,input logic we_mem,
input logic[REGW-1:0] rs1_id,input logic[REGW-1:0] rs2_id,output logic stall_id);
  assign stall_id = we_mem && v_mem && (rd_mem!='0) && ((rd_mem==rs1_id)||(rd_mem==rs2_id));
endmodule
