`timescale 1ns/1ps
import mc_common_pkg::*;
module unified_l1_llsc #(parameter int LINES=64)(input logic clk,rst_n,
  input mem_req_t if_req, output mem_resp_t if_resp,
  input mem_req_t d_req,  output mem_resp_t d_resp,
  output mem_req_t l2_req, input mem_resp_t l2_resp);
  typedef struct packed { logic valid; xlen_t addr; } res_t;
  res_t reservation;
  mem_req_t act; assign act = d_req.valid ? d_req : (if_req.valid ? if_req : '{default:'0});
  assign l2_req = act;
  mem_req_t act_q;
  always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) act_q<='0; else if(act.valid) act_q<=act;
  end
  always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) reservation<='{default:'0};
    else begin
      if(d_req.valid && d_req.atomic && !d_req.wr) begin reservation.valid<=1; reservation.addr<=d_req.addr; end
      if(l2_req.valid && l2_req.wr) if(reservation.valid && reservation.addr==l2_req.addr) reservation.valid<=0;
    end
  end
  assign if_resp = '{valid:(l2_resp.valid && act_q.is_ifetch), rdata:l2_resp.rdata, sc_success:1'b0};
  wire sc_ok = (act_q.atomic && act_q.wr && reservation.valid && reservation.addr==act_q.addr);
  assign d_resp = '{valid:(l2_resp.valid && !act_q.is_ifetch), rdata:l2_resp.rdata, sc_success:sc_ok};
endmodule
