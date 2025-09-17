`timescale 1ns/1ps
import mc_common_pkg::*;
module core7 #(parameter bit ENABLE_BHT=1'b1, parameter bit ENABLE_BTB=1'b1)
( input logic clk,rst_n, output mem_req_t if_req, input mem_resp_t if_resp, output mem_req_t d_req, input mem_resp_t d_resp, output logic halted );
  typedef struct packed { logic is_ll,is_sc,is_add,is_sub,is_br,is_j,is_halt; logic[4:0] rs1,rs2,rd; logic rd_we; xlen_t imm; } dec_t;
  function automatic dec_t decode32(input logic[31:0] ins);
    dec_t d; d='0;
    unique case(ins[31:28])
      4'h1: d.is_ll=1;
      4'h2: d.is_sc=1;
      4'h3: begin d.is_add=1; d.rd_we=1; end
      4'h4: begin d.is_sub=1; d.rd_we=1; end
      4'h5: d.is_br=1;
      4'h6: d.is_j=1;
      4'hF: d.is_halt=1;
      default: ;
    endcase
    d.rd=ins[27:24]; d.rs1=ins[23:20]; d.rs2=ins[19:16]; d.imm={{16{ins[15]}},ins[15:0]};
    return d;
  endfunction
  typedef struct packed { logic v; xlen_t pc; } if_t;
  typedef struct packed { logic v; xlen_t pc; logic[31:0] instr; dec_t dec; } id_t;
  typedef struct packed { logic v; xlen_t pc; dec_t dec; xlen_t a,b; } ex_t;
  typedef struct packed { logic v; xlen_t pc; dec_t dec; xlen_t addr,alu; } m1_t;
  typedef struct packed { logic v; xlen_t pc; dec_t dec; xlen_t rdata; } m2_t;
  typedef struct packed { logic v; xlen_t pc; dec_t dec; xlen_t wb; } wb_t;
  if_t IF,IF_n; id_t ID,ID_n; ex_t EX,EX_n; m1_t M1,M1_n; m2_t M2,M2_n; wb_t WB,WB_n;
  xlen_t rf[0:31];
  always_ff @(posedge clk or negedge rst_n) begin if(!rst_n) for(int i=0;i<32;i++) rf[i]<='0; else if(WB.v && WB.dec.rd_we && WB.dec.rd!=0) rf[WB.dec.rd]<=WB.wb; end
  logic btb_hit; logic[31:0] btb_tgt; logic bht_tk;
  assign btb_hit = 1'b0; assign btb_tgt='0; assign bht_tk=1'b0; // simplified
  wire pred_take = btb_hit && bht_tk;
  wire [31:0] next_pc = pred_take ? btb_tgt : (IF.pc + 32'd4);
  assign if_req = '{valid:1'b1, wr:1'b0, addr:IF.pc, wdata:'0, atomic:1'b0, is_ifetch:1'b1, size:2'b10};
  always_comb begin
    IF_n=IF; ID_n='0; ID_n.v=if_resp.valid; ID_n.pc=IF.pc; ID_n.instr=if_resp.rdata; ID_n.dec=decode32(if_resp.rdata);
    IF_n.v=1'b1; IF_n.pc=next_pc;
  end
  function automatic xlen_t fwd(input xlen_t d, input logic[4:0] rs); if (WB.v && WB.dec.rd_we && WB.dec.rd==rs && rs!=0) return WB.wb; else return d; endfunction
  always_comb begin EX_n='0; EX_n.v=ID.v; EX_n.pc=ID.pc; EX_n.dec=ID.dec; EX_n.a=fwd(rf[ID.dec.rs1],ID.dec.rs1);
    EX_n.b = (ID.dec.is_add||ID.dec.is_sub||ID.dec.is_br)? ID.dec.imm : fwd(rf[ID.dec.rs2],ID.dec.rs2);
  end
  always_comb begin M1_n='0; M1_n.v=EX.v; M1_n.pc=EX.pc; M1_n.dec=EX.dec; xlen_t add=EX.a+EX.b; xlen_t sub=EX.a-EX.b; M1_n.alu=(EX.dec.is_sub)?sub:add; M1_n.addr=EX.a+EX.b; end
  assign d_req = '{valid:(M1.v&&(M1.dec.is_ll||M1.dec.is_sc)), wr:M1.dec.is_sc, addr:M1.addr, wdata:M1.alu, atomic:(M1.dec.is_ll||M1.dec.is_sc), is_ifetch:1'b0, size:2'b10};
  always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin IF<='{v:0,pc:32'h0}; ID<='0; EX<='0; M1<='0; M2<='0; WB<='0; halted<=0; end
    else begin
      IF<=IF_n; ID<=ID_n; EX<=EX_n; M1<=M1_n;
      M2<='{v:d_resp.valid, pc:M1.pc, dec:M1.dec, rdata:d_resp.rdata};
      WB<='{v:M2.v, pc:M2.pc, dec:M2.dec, wb: (M2.dec.is_add||M2.dec.is_sub)? M1.alu : M2.rdata};
      if(WB.v && WB.dec.is_halt) halted<=1;
    end
  end
endmodule
