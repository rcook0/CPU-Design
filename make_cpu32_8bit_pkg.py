#!/usr/bin/env python3
# make_cpu32_8bit_pkg.py
import os, zipfile, textwrap, struct

ROOT = "cpu32_8bit_pkg_full"
DIRS = ["rtl","sim","sw","scripts","views","docs","sw/tests"]
for d in DIRS:
    os.makedirs(os.path.join(ROOT,d), exist_ok=True)

def W(relpath, content, mode="w", binary=False):
    p = os.path.join(ROOT, relpath)
    with open(p, "wb" if binary else mode) as f:
        f.write(content if binary else content)
    return p

# ------------- RTL -------------
W("rtl/mc_common_pkg.sv", """\
package mc_common_pkg;
  parameter int XLEN   = 32;
  parameter int NCORES = 2;
  typedef logic [XLEN-1:0] xlen_t;
  typedef struct packed { logic valid, wr, atomic, is_ifetch; logic [1:0] size; xlen_t addr, wdata; } mem_req_t;
  typedef struct packed { logic valid, sc_success; xlen_t rdata; } mem_resp_t;
endpackage
""")

W("rtl/instruction_memory.sv", """\
`timescale 1ns/1ps
import mc_common_pkg::*;
module instruction_memory #(parameter string PROGRAM_FILE="program.hex", parameter int WORDS=4096)
( input xlen_t addr, output logic [31:0] data );
  logic [31:0] mem [0:WORDS-1];
  initial if (PROGRAM_FILE!="") $readmemh(PROGRAM_FILE, mem);
  assign data = mem[addr[31:2]];
endmodule
""")

W("rtl/unified_l1_llsc.sv", """\
`timescale 1ns/1ps
import mc_common_pkg::*;
module unified_l1_llsc (input logic clk,rst_n,
  input mem_req_t if_req, output mem_resp_t if_resp,
  input mem_req_t d_req,  output mem_resp_t d_resp,
  output mem_req_t l2_req, input mem_resp_t l2_resp);
  typedef struct packed { logic valid; xlen_t addr; } res_t;
  res_t reservation; mem_req_t act, act_q;
  assign act = d_req.valid ? d_req : (if_req.valid ? if_req : '{default:'0});
  assign l2_req = act;
  always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) act_q <= '0; else if (act.valid) act_q <= act;
  end
  always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) reservation <= '{default:'0};
    else begin
      if (d_req.valid && d_req.atomic && !d_req.wr) begin reservation.valid<=1'b1; reservation.addr<=d_req.addr; end
      if (l2_req.valid && l2_req.wr && reservation.valid && reservation.addr==l2_req.addr) reservation.valid<=1'b0;
    end
  end
  assign if_resp = '{valid:l2_resp.valid && act_q.is_ifetch, rdata:l2_resp.rdata, sc_success:1'b0};
  wire sc_ok = (act_q.atomic && act_q.wr && reservation.valid && reservation.addr==act_q.addr);
  assign d_resp = '{valid:l2_resp.valid && !act_q.is_ifetch, rdata:l2_resp.rdata, sc_success:sc_ok};
endmodule
""")

W("rtl/l2_cache_simple.sv", """\
`timescale 1ns/1ps
import mc_common_pkg::*;
module l2_cache_simple #(parameter int WORDS=1<<16, parameter string PROGRAM_HEX="program.hex", parameter string DATA_HEX="data_init.hex")
( input logic clk,rst_n, input mem_req_t req, output mem_resp_t resp );
  xlen_t mem [0:WORDS-1]; xlen_t r_q; logic v_q;
  initial begin if(PROGRAM_HEX!="") $readmemh(PROGRAM_HEX, mem); if(DATA_HEX!="") $readmemh(DATA_HEX, mem); end
  always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin v_q<=1'b0; r_q<='0; end
    else begin
      v_q <= req.valid;
      if (req.valid) begin if (req.wr) mem[req.addr[31:2]] <= req.wdata; r_q <= mem[req.addr[31:2]]; end
    end
  end
  assign resp = '{valid:v_q, rdata:r_q, sc_success:(req.atomic && req.wr)};
endmodule
""")

W("rtl/rr_arbiter.sv", """\
`timescale 1ns/1ps
import mc_common_pkg::*;
module rr_arbiter #(parameter int N=2)
( input logic clk,rst_n, input mem_req_t req[N], output mem_resp_t rsp[N], output mem_req_t l2_req, input mem_resp_t l2_resp );
  logic[$clog2(N)-1:0] ptr, grant;
  always_comb begin
    grant = ptr;
    for (int i=0;i<N;i++) begin int k=(ptr+i)%N; if (req[k].valid) begin grant=k[$clog2(N)-1:0]; break; end end
  end
  assign l2_req = req[grant];
  for (genvar i=0;i<N;i++) begin : G assign rsp[i] = (i==grant)? l2_resp : '{default:'0}; end
  always_ff @(posedge clk or negedge rst_n) begin if(!rst_n) ptr<='0; else if(l2_req.valid) ptr <= (grant==N-1)? '0 : (grant+1); end
endmodule
""")

W("rtl/core8.sv", """\
// 32-bit core with 8-bit opcodes and simple R/I/S/B/J formats.
`timescale 1ns/1ps
import mc_common_pkg::*;
module core8 (input logic clk,rst_n, output mem_req_t if_req, input mem_resp_t if_resp, output mem_req_t d_req, input mem_resp_t d_resp, output logic halted);
  typedef struct packed {logic is_nop,is_halt,is_add,is_sub,is_and,is_or,is_xor,is_sll,is_srl,is_sra,is_addi,is_lw,is_sw,is_ll,is_sc,is_beq,is_bne,is_blt,is_bge,is_jal; logic[4:0] rd,rs1,rs2; xlen_t imm; logic rd_we;} dec_t;
  typedef struct packed {logic v; xlen_t pc;} if_t;
  typedef struct packed {logic v; xlen_t pc; logic[31:0] instr; dec_t dec;} id_t;
  typedef struct packed {logic v; xlen_t pc; dec_t dec; xlen_t a,b;} ex_t;
  typedef struct packed {logic v; xlen_t pc; dec_t dec; xlen_t addr,alu;} m1_t;
  typedef struct packed {logic v; xlen_t pc; dec_t dec; xlen_t rdata;} m2_t;
  typedef struct packed {logic v; xlen_t pc; dec_t dec; xlen_t wb;} wb_t;
  if_t IF,IF_n; id_t ID,ID_n; ex_t EX,EX_n; m1_t M1,M1_n; m2_t M2,M2_n; wb_t WB,WB_n;
  xlen_t rf[0:31];
  always_ff @(posedge clk or negedge rst_n) begin if(!rst_n) for(int i=0;i<32;i++) rf[i]<='0; else if(WB.v && WB.dec.rd_we && WB.dec.rd!=5'd0) rf[WB.dec.rd]<=WB.wb; end
  function automatic dec_t decode(input logic[31:0] ins);
    dec_t d; d='0; logic[7:0] op=ins[31:24]; logic[4:0] rd=ins[23:19], rs1=ins[18:14], rs2=ins[13:9]; logic[13:0] imm14=ins[13:0]; logic[18:0] imm19=ins[18:0]; logic[8:0] funct9=ins[8:0];
    xlen_t se14={{18{imm14[13]}},imm14}; xlen_t se19={{13{imm19[18]}},imm19}; d.rd=rd; d.rs1=rs1; d.rs2=rs2;
    unique case(op)
      8'h00: d.is_nop=1;
      8'h01: begin unique case(funct9)
        9'h000:d.is_add=1; 9'h001:d.is_sub=1; 9'h002:d.is_and=1; 9'h003:d.is_or=1;
        9'h004:d.is_xor=1; 9'h005:d.is_sll=1; 9'h006:d.is_srl=1; 9'h007:d.is_sra=1; endcase d.rd_we=1; end
      8'h02: begin d.is_addi=1; d.rd_we=1; d.imm=se14; end
      8'h30: begin d.is_lw=1;   d.rd_we=1; d.imm=se14; end
      8'h31: begin d.is_sw=1;   d.imm=se14; end
      8'h32: begin d.is_ll=1;   d.rd_we=1; d.imm=se14; end
      8'h33: begin d.is_sc=1;   d.imm=se14; end
      8'h40: begin d.is_beq=1;  d.imm=se14; end
      8'h41: begin d.is_bne=1;  d.imm=se14; end
      8'h42: begin d.is_blt=1;  d.imm=se14; end
      8'h43: begin d.is_bge=1;  d.imm=se14; end
      8'h50: begin d.is_jal=1;  d.rd_we=1; d.imm=se19; end
      8'hFF: d.is_halt=1;
    endcase
    return d;
  endfunction
  assign if_req = '{valid:1'b1, wr:1'b0, addr:IF.pc, wdata:'0, atomic:1'b0, is_ifetch:1'b1, size:2'b10};
  wire [31:0] pc_next = IF.pc + 32'd4;
  always_comb begin IF_n=IF; ID_n='0; ID_n.v=if_resp.valid; ID_n.pc=IF.pc; ID_n.instr=if_resp.rdata; ID_n.dec=decode(if_resp.rdata); IF_n.v=1'b1; IF_n.pc=pc_next; end
  function automatic xlen_t R(input [4:0] r); return (r==5'd0)?'0:rf[r]; endfunction
  always_comb begin EX_n='0; EX_n.v=ID.v; EX_n.pc=ID.pc; EX_n.dec=ID.dec; EX_n.a=R(ID.dec.rs1); EX_n.b= ID.dec.is_addi? ID.dec.imm : R(ID.dec.rs2); end
  wire take_beq = EX.dec.is_beq && (EX.a==R(EX.dec.rs2));
  wire take_bne = EX.dec.is_bne && (EX.a!=R(EX.dec.rs2));
  wire take_blt = EX.dec.is_blt && ($signed(EX.a) <  $signed(R(EX.dec.rs2)));
  wire take_bge = EX.dec.is_bge && ($signed(EX.a) >= $signed(R(EX.dec.rs2)));
  wire do_br = take_beq||take_bne||take_blt||take_bge;
  always_comb begin
    M1_n='0; M1_n.v=EX.v; M1_n.pc=EX.pc; M1_n.dec=EX.dec;
    xlen_t add=EX.a+EX.b, sub=EX.a-EX.b, sll=EX.a<<(EX.b[4:0]), srl=EX.a>>(EX.b[4:0]), sra=$signed(EX.a)>>> (EX.b[4:0]);
    unique case(1'b1)
      EX.dec.is_add,EX.dec.is_addi: M1_n.alu=add;
      EX.dec.is_sub: M1_n.alu=sub;
      EX.dec.is_and: M1_n.alu=(EX.a & EX.b);
      EX.dec.is_or : M1_n.alu=(EX.a | EX.b);
      EX.dec.is_xor: M1_n.alu=(EX.a ^ EX.b);
      EX.dec.is_sll: M1_n.alu=sll;
      EX.dec.is_srl: M1_n.alu=srl;
      EX.dec.is_sra: M1_n.alu=sra;
      default: M1_n.alu=add;
    endcase
    M1_n.addr = EX.a + EX.dec.imm;
  end
  assign d_req = '{valid:(M1.v && (M1.dec.is_ll||M1.dec.is_sc||M1.dec.is_lw||M1.dec.is_sw)), wr:(M1.dec.is_sc||M1.dec.is_sw), addr:M1.addr, wdata:M1.alu, atomic:(M1.dec.is_ll||M1.dec.is_sc), is_ifetch:1'b0, size:2'b10};
  always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin IF<='{v:1'b0,pc:32'h0}; ID<='0; EX<='0; M1<='0; M2<='0; WB<='0; halted<=1'b0; end
    else begin
      IF<=IF_n; ID<=ID_n; EX<=EX_n;
      if(EX.v && do_br) IF.pc <= EX.pc + EX.dec.imm;
      else if(EX.v && EX.dec.is_jal) IF.pc <= EX.pc + EX.dec.imm;
      M2<='{v:d_resp.valid, pc:M1.pc, dec:M1.dec, rdata:d_resp.rdata};
      WB<='{v:M2.v, pc:M2.pc, dec:M2.dec, wb:(M2.dec.is_lw||M2.dec.is_ll)? M2.rdata : M1.alu};
      if(WB.v && WB.dec.is_halt) halted<=1'b1;
    end
  end
endmodule
""")

W("rtl/multicore_top3.sv", """\
`timescale 1ns/1ps
import mc_common_pkg::*;
module multicore_top3 #(parameter string PROGRAM_HEX="program.hex") (input logic clk,rst_n, output logic [NCORES-1:0] halted);
  localparam int N=NCORES;
  mem_req_t if_req[N]; mem_resp_t if_resp[N];
  mem_req_t d_req[N];  mem_resp_t d_resp[N];
  mem_req_t l1req[N];  mem_resp_t l1rsp[N];
  mem_req_t l2_req;    mem_resp_t l2_resp;
  logic [31:0] instr[N];
  for (genvar c=0;c<N;c++) begin: G_IM
    instruction_memory #(.PROGRAM_FILE(PROGRAM_HEX)) imem(.addr(if_req[c].addr), .data(instr[c]));
  end
  for (genvar c=0;c<N;c++) begin: G_L1
    unified_l1_llsc u_l1(.clk,.rst_n,.if_req(if_req[c]),.if_resp(if_resp[c]),.d_req(d_req[c]),.d_resp(d_resp[c]),.l2_req(l1req[c]),.l2_resp(l1rsp[c]));
  end
  for (genvar c=0;c<N;c++) begin: G_CORE
    core8 u_core(.clk,.rst_n,.if_req(if_req[c]),.if_resp('{valid:1'b1, rdata:instr[c], sc_success:1'b0}),.d_req(d_req[c]),.d_resp(d_resp[c]),.halted(halted[c]));
  end
  rr_arbiter #(.N(N)) u_rr(.clk,.rst_n,.req(l1req),.rsp(l1rsp),.l2_req(l2_req),.l2_resp(l2_resp));
  l2_cache_simple u_l2(.clk,.rst_n,.req(l2_req),.resp(l2_resp));
endmodule
""")

# ------------- SIM -------------
W("sim/multicore_full_sim3.sv", """\
`timescale 1ns/1ps
import mc_common_pkg::*;
module multicore_full_sim3;
  logic clk=0, rst_n=0; always #5 clk = ~clk;
  logic [NCORES-1:0] halted;
  multicore_top3 #(.PROGRAM_HEX("program.hex")) dut(.clk,.rst_n,.halted);
  initial begin
    $dumpfile("waves.vcd"); $dumpvars(0, multicore_full_sim3);
    repeat (10) @(posedge clk); rst_n=1;
    wait(&halted); $display("All cores halted."); $finish(0);
  end
endmodule
""")

# ------------- SW: assembler / disassembler / program -------------
W("sw/asm8.py", """\
#!/usr/bin/env python3
import sys, argparse, struct, re
OPC={'NOP':0x00,'ALU':0x01,'ADDI':0x02,'LW':0x30,'SW':0x31,'LL':0x32,'SC':0x33,'BEQ':0x40,'BNE':0x41,'BLT':0x42,'BGE':0x43,'JAL':0x50,'HALT':0xFF}
FUNCT9={'ADD':0x000,'SUB':0x001,'AND':0x002,'OR':0x003,'XOR':0x004,'SLL':0x005,'SRL':0x006,'SRA':0x007}
REG={f"r{i}":i for i in range(32)}
def R(tok):
    t=tok.strip().lower().rstrip(',')
    if t not in REG: raise SystemExit(f"Unknown register {tok}")
    return REG[t]
def enc_R(op,rd,rs1,rs2,funct9): return (op<<24)|((rd&31)<<19)|((rs1&31)<<14)|((rs2&31)<<9)|(funct9&0x1FF)
def enc_I(op,rd,rs1,imm14): return (op<<24)|((rd&31)<<19)|((rs1&31)<<14)|(imm14 & 0x3FFF)
def enc_S(op,rs1,rs2,imm14): return (op<<24)|((rs1&31)<<14)|((rs2&31)<<9)|(imm14 & 0x3FFF)
def enc_B(op,rs1,rs2,imm14): return (op<<24)|((rs1&31)<<14)|((rs2&31)<<9)|(imm14 & 0x3FFF)
def enc_J(op,rd,imm19):     return (op<<24)|((rd&31)<<19)|(imm19 & 0x7FFFF)
def assemble(lines):
    labels={}; pc=0; cleaned=[]
    for raw in lines:
        line=raw.split(';')[0].strip()
        if not line: continue
        if line.endswith(':'): labels[line[:-1]]=pc; continue
        cleaned.append(line); pc+=1
    words=[]; pc=0
    for line in cleaned:
        toks=[t.strip() for t in re.split(r'[\\s,()]+', line) if t.strip()]
        m=toks[0].upper()
        if m=='NOP':
            w=enc_R(OPC['NOP'],0,0,0,0)
        elif m in FUNCT9:
            rd=R(toks[1]); rs1=R(toks[2]); rs2=R(toks[3])
            w=enc_R(OPC['ALU'],rd,rs1,rs2,FUNCT9[m])
        elif m=='ADDI':
            rd=R(toks[1]); rs1=R(toks[2]); imm=int(toks[3],0)
            w=enc_I(OPC['ADDI'],rd,rs1,imm)
        elif m in ('LW','LL'):
            rd=R(toks[1]); imm=int(toks[2],0); rs1=R(toks[3])
            w=enc_I(OPC[m],rd,rs1,imm)
        elif m in ('SW','SC'):
            rs2=R(toks[1]); imm=int(toks[2],0); rs1=R(toks[3])
            w=enc_S(OPC[m],rs1,rs2,imm)
        elif m in ('BEQ','BNE','BLT','BGE'):
            rs1=R(toks[1]); rs2=R(toks[2]); tgt=toks[3]
            if tgt in labels: rel=labels[tgt]-(pc+1); imm=rel*4
            else: imm=int(tgt,0)
            w=enc_B(OPC[m],rs1,rs2,imm)
        elif m=='JAL':
            rd=R(toks[1]); tgt=toks[2]
            if tgt in labels: rel=labels[tgt]-(pc+1); imm=rel*4
            else: imm=int(tgt,0)
            w=enc_J(OPC['JAL'],rd,imm)
        elif m=='HALT':
            w=enc_R(OPC['HALT'],0,0,0,0)
        else:
            raise SystemExit(f"Unknown mnemonic: {m} at '{line}'")
        words.append(w); pc+=1
    return words
def main():
    ap=argparse.ArgumentParser(description='8-bit opcode assembler')
    ap.add_argument('input'); ap.add_argument('-o','--out',default='program')
    a=ap.parse_args()
    lines=open(a.input).read().splitlines()
    words=assemble(lines)
    with open(a.out+'.hex','w') as f:
        for w in words: f.write(f"{w:08X}\\n")
    with open(a.out+'.bin','wb') as f:
        for w in words: f.write(struct.pack('<I', w))
    print(f"[OK] Wrote {a.out}.hex and {a.out}.bin ({len(words)} words)")
if __name__=='__main__': main()
"""); os.chmod(os.path.join(ROOT,"sw/asm8.py"),0o755)

W("sw/disasm8.py", """\
#!/usr/bin/env python3
import sys, struct
OPC_R=0x01
MNEM={0x00:'NOP',0x02:'ADDI',0x30:'LW',0x31:'SW',0x32:'LL',0x33:'SC',0x40:'BEQ',0x41:'BNE',0x42:'BLT',0x43:'BGE',0x50:'JAL',0xFF:'HALT'}
FUN9={0x000:'ADD',0x001:'SUB',0x002:'AND',0x003:'OR',0x004:'XOR',0x005:'SLL',0x006:'SRL',0x007:'SRA'}
def words(path):
    if path.lower().endswith('.hex'):
        return [int(l.strip(),16) for l in open(path) if l.strip()]
    b=open(path,'rb').read()
    return [struct.unpack('<I', b[i:i+4])[0] for i in range(0,len(b),4)]
def dis(w,pc):
    opc=(w>>24)&0xFF; rd=(w>>19)&0x1F; rs1=(w>>14)&0x1F; rs2=(w>>9)&0x1F
    imm14=w&0x3FFF; imm14-=1<<14 if (imm14&0x2000) else 0
    imm19=w&0x7FFFF; imm19-=1<<19 if (imm19&0x40000) else 0
    f9=w&0x1FF
    if opc==OPC_R: return f"{FUN9.get(f9,'ALU?%03x'%f9)} r{rd}, r{rs1}, r{rs2}"
    if opc in (0x30,0x32): return f"{MNEM[opc]} r{rd}, {imm14}(r{rs1})"
    if opc in (0x31,0x33): return f"{MNEM[opc]} r{rs2}, {imm14}(r{rs1})"
    if opc in (0x40,0x41,0x42,0x43): return f"{MNEM[opc]} r{rs1}, r{rs2}, 0x{pc+4+imm14:08X}"
    if opc==0x50: return f"JAL r{rd}, 0x{pc+4+imm19:08X}"
    if opc==0x02: return f"ADDI r{rd}, r{rs1}, {imm14}"
    return MNEM.get(opc, f"OPC?{opc:02X}")
def main():
    if len(sys.argv)<2: print("Usage: disasm8.py program.hex|program.bin"); sys.exit(1)
    pc=0
    for w in words(sys.argv[1]):
        print(f"{pc:08X}: {w:08X}   {dis(w,pc)}"); pc+=4
if __name__=='__main__': main()
"""); os.chmod(os.path.join(ROOT,"sw/disasm8.py"),0o755)

W("sw/program.s", """\
; sample program
ADDI r1, r0, 10
ADDI r2, r0, 20
ADD  r3, r1, r2
SW   r3, 0, r0
HALT
""")

# Pre-generate program.hex/bin for convenience (inline minimal assembler)
def assemble_inline(lines):
    OPC={'NOP':0x00,'ALU':0x01,'ADDI':0x02,'LW':0x30,'SW':0x31,'LL':0x32,'SC':0x33,'BEQ':0x40,'BNE':0x41,'BLT':0x42,'BGE':0x43,'JAL':0x50,'HALT':0xFF}
    FUN={'ADD':0x000,'SUB':0x001,'AND':0x002,'OR':0x003,'XOR':0x004,'SLL':0x005,'SRL':0x006,'SRA':0x007}
    REG={f"r{i}":i for i in range(32)}
    def R(t): t=t.strip().lower().rstrip(','); 
    ...
