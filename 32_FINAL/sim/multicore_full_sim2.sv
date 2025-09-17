`timescale 1ns/1ps
import mc_common_pkg::*;
module multicore_full_sim2;
  logic clk=0, rst_n=0; always #5 clk=~clk;
  logic [NCORES-1:0] halted;
  multicore_top2 #(.PROGRAM_HEX("program.hex")) dut(.clk, .rst_n, .halted);
  initial begin
    $dumpfile("waves.vcd"); $dumpvars(0, multicore_full_sim2);
    repeat(10) @(posedge clk); rst_n=1;
    wait(&halted); $display("All cores halted."); $finish(0);
  end
endmodule
