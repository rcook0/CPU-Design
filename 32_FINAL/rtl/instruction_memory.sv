`timescale 1ns/1ps
import mc_common_pkg::*;
module instruction_memory #(parameter string PROGRAM_FILE="program.hex", parameter int WORDS=4096)
( input xlen_t addr, output logic[31:0] data );
  logic[31:0] mem[0:WORDS-1];
  initial if(PROGRAM_FILE!="") $readmemh(PROGRAM_FILE, mem);
  assign data = mem[addr[31:2]];
endmodule
