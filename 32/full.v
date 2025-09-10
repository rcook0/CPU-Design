// regfile.v - 32 x 32-bit register file, reg0 is zero
module regfile (
    input           clk,
    input           rst_n,

    input  [4:0]    ra1,
    input  [4:0]    ra2,
    output [31:0]   rd1,
    output [31:0]   rd2,

    input  [4:0]    wa,
    input  [31:0]   wd,
    input           we
);
    reg [31:0] regs [31:0];
    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i=0;i<32;i=i+1) regs[i] <= 32'b0;
        end else begin
            if (we && wa != 5'b0) regs[wa] <= wd;
        end
    end

    assign rd1 = (ra1==5'b0) ? 32'b0 : regs[ra1];
    assign rd2 = (ra2==5'b0) ? 32'b0 : regs[ra2];
endmodule

// alu.v - simple ALU for R-type and I-type ops
module alu (
    input  [31:0] a,
    input  [31:0] b,
    input  [3:0]  op,   // see funct table in top comment
    output reg [31:0] y,
    output zero
);
    always @(*) begin
        case (op)
            4'b0000: y = a + b; // ADD
            4'b0001: y = a - b; // SUB
            4'b0010: y = a & b; // AND
            4'b0011: y = a | b; // OR
            4'b0100: y = a ^ b; // XOR
            4'b0101: y = a << b[4:0]; // SLL
            4'b0110: y = a >> b[4:0]; // SRL (logical)
            4'b0111: y = $signed(a) >>> b[4:0]; // SRA
            default: y = 32'b0;
        endcase
    end
    assign zero = (y == 32'b0);
endmodule

// imem.v - simple instruction memory (behavioral)
// Size: 1024 words (change DEPTH)
module imem #(
    parameter DEPTH = 1024
) (
    input  [31:0] addr, // byte address
    output [31:0] data
);
    reg [31:0] mem [0:DEPTH-1];
    initial begin
        // Default: set all NOPs; testbench may override using $readmemh
        integer i;
        for (i=0;i<DEPTH;i=i+1) mem[i] = 32'h00000000;
    end

    // word-address from byte address
    wire [31:0] waddr = addr >> 2;
    assign data = mem[waddr];

    // Provide a task for testbench to write directly (not synthesizable)
    // But in simulation you can use: imem_inst.mem[0] = 32'hXXXXXXXX;
endmodule

// dmem.v - main data memory (behavioral)
module dmem #(
    parameter DEPTH = 4096
) (
    input             clk,
    input             rst_n,
    input             en,
    input             wr,   // 1=write, 0=read
    input  [31:0]     addr,
    input  [31:0]     wdata,
    output reg [31:0] rdata
);
    reg [31:0] mem [0:DEPTH-1];
    integer i;
    initial begin
        for (i=0;i<DEPTH;i=i+1) mem[i] = 32'h0;
    end

    always @(posedge clk) begin
        if (en) begin
            if (wr) begin
                mem[addr>>2] <= wdata;
            end else begin
                rdata <= mem[addr>>2];
            end
        end
    end
endmodule

// l1cache.v - simple direct-mapped L1 cache (write-through, no fill buffer complexity).
// - Parameter LINEWORDS: words per cacheline (power of two)
// - Parameter LINES: number of lines
// - On a miss, a configurable miss_penalty cycles stall is returned (the CPU will stall IF stage).
module l1cache #(
    parameter LINEWORDS = 4, // 4 words per line = 16 bytes
    parameter LINES = 64,
    parameter HIT_LATENCY = 1, // cycles
    parameter MISS_PENALTY = 10 // cycles to service miss from main mem
) (
    input           clk,
    input           rst_n,

    // CPU side
    input           req_valid,
    input           req_wr,      // 1 = store, 0 = load / inst fetch
    input  [31:0]   req_addr,
    input  [31:0]   req_wdata,
    output reg      resp_valid,
    output reg [31:0] resp_rdata,
    output reg      busy        // high while miss is being serviced
);
    localparam INDEX_BITS = $clog2(LINES);
    localparam OFFSET_BITS = $clog2(LINEWORDS);
    localparam TAG_BITS = 32 - INDEX_BITS - OFFSET_BITS - 2; // -2 for byte offset inside word-addressed

    typedef struct packed {
        reg valid;
        reg [TAG_BITS-1:0] tag;
        reg [31:0] data [LINEWORDS-1:0];
    } cache_line_t;

    // Behavioral arrays (simulation only)
    reg valid_array [0:LINES-1];
    reg [TAG_BITS-1:0] tag_array [0:LINES-1];
    reg [31:0] data_array [0:LINES-1][0:LINEWORDS-1];

    // Miss handling
    reg [31:0] pending_addr;
    reg pending_wr;
    reg [31:0] pending_wdata;
    reg [3:0] miss_ctr;

    integer i,j;
    initial begin
        for (i=0;i<LINES;i=i+1) begin
            valid_array[i] = 0;
            tag_array[i] = 0;
            for (j=0;j<LINEWORDS;j=j+1) data_array[i][j] = 32'h0;
        end
        resp_valid = 0;
        resp_rdata = 32'h0;
        busy = 0;
        miss_ctr = 0;
    end

    // address breaking
    wire [31:0] word_addr = req_addr >> 2;
    wire [OFFSET_BITS-1:0] offset = word_addr[OFFSET_BITS-1:0];
    wire [INDEX_BITS-1:0] index = word_addr[OFFSET_BITS +: INDEX_BITS];
    wire [TAG_BITS-1:0] tag = word_addr[OFFSET_BITS+INDEX_BITS +: TAG_BITS];

    // For simulation we emulate main memory read with delay when MISS occurs and then fill line
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            resp_valid <= 0;
            busy <= 0;
            miss_ctr <= 0;
        end else begin
            resp_valid <= 0; // clear unless set below

            if (busy) begin
                if (miss_ctr > 0) begin
                    miss_ctr <= miss_ctr - 1;
                end else begin
                    // finish miss: emulate fill from main memory (set data to some deterministic value = addr)
                    // Fill line with word values equal to address base + offset
                    integer k;
                    reg [31:0] base_word;
                    base_word = (req_addr & ~((LINEWORDS*4)-1));
                    for (k=0;k<LINEWORDS;k=k+1) begin
                        data_array[index][k] <= base_word + (k*4);
                    end
                    valid_array[index] <= 1;
                    tag_array[index] <= tag;

                    // Now respond
                    if (pending_wr) begin
                        // perform write-through: also write the cache line and main mem (we emulate by writing cache)
                        data_array[index][offset] <= pending_wdata;
                        resp_rdata <= 32'b0;
                    end else begin
                        resp_rdata <= data_array[index][offset];
                    end
                    resp_valid <= 1;
                    busy <= 0;
                end
            end else begin
                if (req_valid) begin
                    if (valid_array[index] && (tag_array[index] == tag)) begin
                        // HIT
                        if (req_wr) begin
                            data_array[index][offset] <= req_wdata; // write-through to cache line
                            resp_valid <= 1;
                        end else begin
                            resp_rdata <= data_array[index][offset];
                            resp_valid <= 1;
                        end
                    end else begin
                        // MISS: start miss service
                        busy <= 1;
                        miss_ctr <= MISS_PENALTY - 1; // count down
                        pending_addr <= req_addr;
                        pending_wr <= req_wr;
                        pending_wdata <= req_wdata;
                        resp_valid <= 0;
                    end
                end
            end
        end
    end

endmodule

// cpu.v - 5-stage pipelined CPU top connecting the modules above
`timescale 1ns/1ps
module cpu_top (
    input clk,
    input rst_n,

    // hook to imem/dmem/l1
    output [31:0] imem_addr,
    input  [31:0] imem_data,

    // Data memory interface via L1 cache
    output        d_req_valid,
    output        d_req_wr,
    output [31:0] d_req_addr,
    output [31:0] d_req_wdata,
    input         d_resp_valid,
    input  [31:0] d_resp_rdata,
    input         d_busy
);
    // Program counter and IF stage
    reg [31:0] pc, pc_next;
    reg stall; // pipeline stall due to load-use or cache miss

    // Pipeline registers
    // IF/ID
    reg [31:0] if_id_instr;
    reg [31:0] if_id_pc;

    // ID/EX
    reg [31:0] id_ex_pc;
    reg [31:0] id_ex_rs1;
    reg [31:0] id_ex_rs2;
    reg [31:0] id_ex_imm;
    reg [4:0]  id_ex_rd;
    reg [4:0]  id_ex_rs1_idx, id_ex_rs2_idx;
    reg [5:0]  id_ex_opcode;
    reg [10:0] id_ex_funct;
    reg        id_ex_regwrite;
    reg        id_ex_memwrite;
    reg        id_ex_memread;
    reg [3:0]  id_ex_aluop;
    reg        id_ex_aluimm;

    // EX/MEM
    reg [31:0] ex_mem_aluout;
    reg [31:0] ex_mem_rs2;
    reg [4:0]  ex_mem_rd;
    reg        ex_mem_regwrite;
    reg        ex_mem_memwrite;
    reg        ex_mem_memread;

    // MEM/WB
    reg [31:0] mem_wb_memdata;
    reg [31:0] mem_wb_aluout;
    reg [4:0]  mem_wb_rd;
    reg        mem_wb_regwrite;
    reg        mem_wb_memread;

    // Register file instance
    wire [31:0] rf_rd1, rf_rd2;
    regfile rf (
        .clk(clk),
        .rst_n(rst_n),
        .ra1(if_id_instr[20:16]), // rs1 field for ID stage reads (assuming layout)
        .ra2(if_id_instr[15:11]),
        .rd1(rf_rd1),
        .rd2(rf_rd2),
        .wa(mem_wb_rd),
        .wd(mem_wb_memread ? mem_wb_memdata : mem_wb_aluout),
        .we(mem_wb_regwrite)
    );

    // ALU
    wire [31:0] alu_a, alu_b, alu_y;
    wire alu_zero;
    alu alu0(
        .a(alu_a),
        .b(alu_b),
        .op(id_ex_aluop),
        .y(alu_y),
        .zero(alu_zero)
    );

    // Control decode (simple)
    // We'll decode FROM if_id_instr in ID stage into ID/EX regs
    // Opcode mapping per top comment

    // Sign extension helpers
    function [31:0] signext16;
        input [15:0] x;
        signext16 = {{16{x[15]}}, x};
    endfunction

    // PC logic
    assign imem_addr = pc;

    // Simple hazard detection: load-use -> stall one cycle
    wire id_ex_is_load = id_ex_memread;

    // Setup data memory request signals
    reg d_req_valid_r;
    reg d_req_wr_r;
    reg [31:0] d_req_addr_r;
    reg [31:0] d_req_wdata_r;
    assign d_req_valid = d_req_valid_r;
    assign d_req_wr = d_req_wr_r;
    assign d_req_addr = d_req_addr_r;
    assign d_req_wdata = d_req_wdata_r;

    // CPU main sequential
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pc <= 0;
            if_id_instr <= 32'h0;
            if_id_pc <= 32'h0;
            id_ex_pc <= 0;
            id_ex_rs1 <= 0;
            id_ex_rs2 <= 0;
            id_ex_imm <= 0;
            id_ex_regwrite <= 0;
            id_ex_memwrite <= 0;
            id_ex_memread <= 0;
            ex_mem_regwrite <= 0;
            ex_mem_memwrite <= 0;
            ex_mem_memread <= 0;
            mem_wb_regwrite <= 0;
            d_req_valid_r <= 0;
            d_req_wr_r <= 0;
            d_req_addr_r <= 0;
            d_req_wdata_r <= 0;
            stall <= 0;
        end else begin
            // default: clear memory request unless set in MEM stage
            d_req_valid_r <= 0;

            // If there is a cache miss busy, stall fetch/ID
            stall <= (d_busy);

            // Writeback stage already applied via regfile instance since it's synchronous.

            // MEM stage: interact with L1 cache
            if (ex_mem_memread || ex_mem_memwrite) begin
                // issue request to cache and wait for response
                if (!d_busy && !d_req_valid_r) begin
                    d_req_valid_r <= 1;
                    d_req_wr_r <= ex_mem_memwrite;
                    d_req_addr_r <= ex_mem_aluout;
                    d_req_wdata_r <= ex_mem_rs2;
                end
            end

            // collect cache response for load
            if (d_resp_valid) begin
                mem_wb_memdata <= d_resp_rdata;
            end

            // MEM/WB pipeline regs update
            mem_wb_aluout <= ex_mem_aluout;
            mem_wb_rd <= ex_mem_rd;
            mem_wb_regwrite <= ex_mem_regwrite;
            mem_wb_memread <= ex_mem_memread;
            // mem_wb_memdata set from cache response above

            // EX stage -> MEM stage regs
            ex_mem_aluout <= alu_y;
            ex_mem_rs2 <= id_ex_rs2;
            ex_mem_rd <= id_ex_rd;
            ex_mem_regwrite <= id_ex_regwrite;
            ex_mem_memwrite <= id_ex_memwrite;
            ex_mem_memread <= id_ex_memread;

            // ID/EX -> EX: nothing to do here; we used id_ex_* directly in combinational ALU wiring

            // ID stage: decode if no stall
            if (!stall) begin
                // Transfer IF/ID to ID/EX
                id_ex_pc <= if_id_pc;
                id_ex_rs1_idx <= if_id_instr[20:16];
                id_ex_rs2_idx <= if_id_instr[15:11];
                id_ex_rs1 <= rf_rd1;
                id_ex_rs2 <= rf_rd2;
                id_ex_opcode <= if_id_instr[31:26];
                id_ex_funct <= if_id_instr[10:0];
                id_ex_imm <= signext16(if_id_instr[15:0]);
                id_ex_rd <= if_id_instr[25:21];

                // Decode simple controls
                id_ex_regwrite <= 0;
                id_ex_memwrite <= 0;
                id_ex_memread <= 0;
                id_ex_aluimm <= 0;
                id_ex_aluop <= 4'b0000;
                case (if_id_instr[31:26])
                    6'b000000: begin // R-type
                        id_ex_regwrite <= 1;
                        id_ex_aluimm <= 0;
                        id_ex_aluop <= if_id_instr[3:0];
                    end
                    6'b001000: begin // I-type ALU imm
                        id_ex_regwrite <= 1;
                        id_ex_aluimm <= 1;
                        id_ex_aluop <= 4'b0000; // ADD immediate default; could be extended via funct bits
                    end
                    6'b010000: begin // LD
                        id_ex_regwrite <= 1;
                        id_ex_memread <= 1;
                        id_ex_aluimm <= 1;
                        id_ex_aluop <= 4'b0000; // base+imm
                    end
                    6'b010001: begin // ST
                        id_ex_memwrite <= 1;
                        id_ex_aluimm <= 1;
                        id_ex_aluop <= 4'b0000;
                    end
                    6'b011000: begin // BEQ
                        // handled in EX stage via zero flag; no register write
                        id_ex_aluimm <= 0;
                        id_ex_aluop <= 4'b0001; // SUB to set zero
                    end
                    6'b011001: begin // JAL
                        id_ex_regwrite <= 1; // write link
                        id_ex_aluimm <= 1;
                        id_ex_aluop <= 4'b0000;
                    end
                    default: begin
                        // NOP or unknown
                    end
                endcase
            end else begin
                // freeze IF/ID -> bubble insertion: convert ID/EX to NOP when stalled
                id_ex_regwrite <= 0;
                id_ex_memwrite <= 0;
                id_ex_memread <= 0;
                id_ex_aluimm <= 0;
                id_ex_aluop <= 4'b0000;
            end

            // IF stage: fetch next instruction if not stalled
            if (!stall) begin
                if_id_instr <= imem_data;
                if_id_pc <= pc;
                pc <= pc + 4;
            end else begin
                // hold IF/ID (stall)
                if_id_instr <= if_id_instr;
                if_id_pc <= if_id_pc;
                pc <= pc; // hold PC
            end
        end
    end

    // ALU inputs with forwarding basic: EX stage reads from id_ex_rs1/rs2, but we forward from ex_mem and mem_wb
    reg [31:0] forward_a, forward_b;
    always @(*) begin
        // default
        forward_a = id_ex_rs1;
        forward_b = id_ex_aluimm ? id_ex_imm : id_ex_rs2;

        // Forward from EX/MEM if destination matches source
        if (ex_mem_regwrite && (ex_mem_rd != 5'b0) && (ex_mem_rd == id_ex_rs1_idx)) forward_a = ex_mem_aluout;
        if (ex_mem_regwrite && (ex_mem_rd != 5'b0) && (ex_mem_rd == id_ex_rs2_idx) && !id_ex_aluimm) forward_b = ex_mem_aluout;

        // Forward from MEM/WB
        if (mem_wb_regwrite && (mem_wb_rd != 5'b0) && (mem_wb_rd == id_ex_rs1_idx)) begin
            forward_a = mem_wb_memread ? mem_wb_memdata : mem_wb_aluout;
        end
        if (mem_wb_regwrite && (mem_wb_rd != 5'b0) && (mem_wb_rd == id_ex_rs2_idx) && !id_ex_aluimm) begin
            forward_b = mem_wb_memread ? mem_wb_memdata : mem_wb_aluout;
        end
    end

    assign alu_a = forward_a;
    assign alu_b = forward_b;

endmodule

// tb.v - testbench to instantiate cpu_top, imem, l1cache, and a behavioral dmem
`timescale 1ns/1ps
module tb;
    reg clk;
    reg rst_n;

    // instantiate imem
    wire [31:0] imem_addr;
    wire [31:0] imem_data;
    imem #(1024) imem_inst (.addr(imem_addr), .data(imem_data));

    // l1 cache
    wire d_req_valid, d_req_wr;
    wire [31:0] d_req_addr, d_req_wdata;
    wire d_resp_valid;
    wire [31:0] d_resp_rdata;
    wire d_busy;
    l1cache #(.LINEWORDS(4), .LINES(64), .HIT_LATENCY(1), .MISS_PENALTY(5)) l1 (
        .clk(clk),
        .rst_n(rst_n),
        .req_valid(d_req_valid),
        .req_wr(d_req_wr),
        .req_addr(d_req_addr),
        .req_wdata(d_req_wdata),
        .resp_valid(d_resp_valid),
        .resp_rdata(d_resp_rdata),
        .busy(d_busy)
    );

    // main cpu
    cpu_top cpu (
        .clk(clk),
        .rst_n(rst_n),
        .imem_addr(imem_addr),
        .imem_data(imem_data),
        .d_req_valid(d_req_valid),
        .d_req_wr(d_req_wr),
        .d_req_addr(d_req_addr),
        .d_req_wdata(d_req_wdata),
        .d_resp_valid(d_resp_valid),
        .d_resp_rdata(d_resp_rdata),
        .d_busy(d_busy)
    );

    // simple clock
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 100 MHz simulation tick = 10ns period
    end

    // preload a program into imem_inst.mem for test
    initial begin
        rst_n = 0;
        #20;
        rst_n = 1;

        // Simple hand-assembled program (pseudo-assembly shown below)
        // For simplicity we'll put some NOPs and examine register writes in waveform.
        // Example program (conceptual):
        // 0: ADD r1, r0, r0     ; r1 = 0
        // 4: ADDI r2, r0, 8     ; r2 = 8
        // 8: SW  r2, 0(r0)      ; mem[0] = 8
        //12: LW  r3, 0(r0)      ; r3 = mem[0]
        //16: ADD r4, r3, r2     ; r4 = r3 + r2  (should be 16)
        //20: NOP
        // We will use simple encoding defined earlier (not a full assembler).
        // Let's write them directly into the memory array for simulation.

        // Instruction opcodes from spec:
        // R-type: opcode=0, funct low bits define ADD=0000
        // I-type imm ALU: opcode = 6'b001000 (0x08)
        // ST: opcode = 6'b010001 (0x11)
        // LD: opcode = 6'b010000 (0x10)

        // Build instructions manually:
        // ADD r1, r0, r0  => opcode=0, rd=1, rs1=0, rs2=0, funct=0000
        imem_inst.mem[0] = 32'h00004200; // crafted: [31:26]=0, rd=1(00001)<<21, rs1=0<<16, rs2=0<<11, funct=0
        // ADDI r2, r0, 8 => opcode 0x08, rd=2, rs1=0, imm=8
        imem_inst.mem[1] = {6'b001000, 5'd2, 5'd0, 16'h0008};
        // ST r2, 0(r0) => opcode 0x11, rs2=2 in bits[25:21], rs1=0, imm=0
        imem_inst.mem[2] = {6'b010001, 5'd2, 5'd0, 16'h0000};
        // LD r3, 0(r0) => opcode 0x10, rd=3, rs1=0, imm=0
        imem_inst.mem[3] = {6'b010000, 5'd3, 5'd0, 16'h0000};
        // ADD r4, r3, r2 => R-type rd=4, rs1=3, rs2=2, funct=ADD
        imem_inst.mem[4] = {6'b000000, 5'd4, 5'd3, 5'd2, 11'b0};
        // NOPs
        imem_inst.mem[5] = 32'h00000000;
        imem_inst.mem[6] = 32'h00000000;
        imem_inst.mem[7] = 32'h00000000;

        // Let it run for some cycles
        #1000;
        $display("Simulation finished.");
        $finish;
    end

    // optional waveform dump
    initial begin
        $dumpfile("cpu_tb.vcd");
        $dumpvars(0,tb);
    end
endmodule

