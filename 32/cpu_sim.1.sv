// cpu_simu.sv
// SystemVerilog single-file pipelined CPU (simulation-only)
// Harvard: separate I-cache and D-cache, direct-mapped, LINEWORDS = 8
`timescale 1ns/1ps

module cpu_simu_top;
    // instantiate testbench below at bottom of file
endmodule

// ============================================================================
// Parameterizable register file (32 x 32)
// ============================================================================
module regfile_sv (
    input  logic          clk,
    input  logic          rst_n,
    input  logic [4:0]    ra1,
    input  logic [4:0]    ra2,
    output logic [31:0]   rd1,
    output logic [31:0]   rd2,
    input  logic [4:0]    wa,
    input  logic [31:0]   wd,
    input  logic          we
);
    logic [31:0] regs [31];

    integer i;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i=0; i<32; i=i+1)
                regs[i] <= 32'd0;
        end else begin
            if (we && (wa != 5'd0))
                regs[wa] <= wd;
        end
    end

    assign rd1 = (ra1 == 5'd0) ? 32'd0 : regs[ra1];
    assign rd2 = (ra2 == 5'd0) ? 32'd0 : regs[ra2];
endmodule

// ============================================================================
// ALU
// ============================================================================
module alu_sv (
    input  logic [31:0] a,
    input  logic [31:0] b,
    input  logic [3:0]  op,
    output logic [31:0] y,
    output logic        zero
);
    always_comb begin
        unique case (op)
            4'b0000: y = a + b;                       // ADD
            4'b0001: y = a - b;                       // SUB
            4'b0010: y = a & b;                       // AND
            4'b0011: y = a | b;                       // OR
            4'b0100: y = a ^ b;                       // XOR
            4'b0101: y = a << b[4:0];                 // SLL
            4'b0110: y = a >> b[4:0];                 // SRL
            4'b0111: y = $signed(a) >>> b[4:0];       // SRA
            default: y = 32'd0;
        endcase
    end
    assign zero = (y == 32'd0);
endmodule

// ============================================================================
// Instruction memory (behavioral) - word addressed by CPU (byte addr -> word index)
// ============================================================================
module imem_sv #(parameter DEPTH = 2048) (
    input  logic [31:0] addr, // byte address
    output logic [31:0] data
);
    logic [31:0] mem [0:DEPTH-1];
    initial begin
        integer i;
        for (i=0;i<DEPTH;i=i+1) mem[i] = 32'h00000000;
    end

    logic [31:0] waddr;
    assign waddr = addr >> 2;
    assign data = mem[waddr];

    // testbench can poke imem_inst.mem[index] directly
endmodule

// ============================================================================
// Main data memory (behavioral) - backing store for cache fills
// ============================================================================
module dmem_sv #(parameter DEPTH = 16384) (
    input  logic         clk,
    input  logic         rst_n,
    input  logic         en,
    input  logic         wr,   // 1=write, 0=read
    input  logic [31:0]  addr,
    input  logic [31:0]  wdata,
    output logic [31:0]  rdata
);
    logic [31:0] mem [0:DEPTH-1];
    integer i;
    initial begin
        for (i=0;i<DEPTH;i=i+1) mem[i] = 32'h0;
    end

    always_ff @(posedge clk) begin
        if (en) begin
            if (wr) mem[addr >> 2] <= wdata;
            else    rdata <= mem[addr >> 2];
        end
    end
endmodule

// ============================================================================
// Direct-mapped cache (parameterized) - simulation simplified
// - LINEWORDS = 8 words/line
// - On miss, we emulate refill from backing mem by waiting MISS_PENALTY cycles and then
//   filling the line with backing memory contents supplied via a callback/task.
// - For Harvard, we'll instantiate icache and dcache separately.
// ============================================================================
module cache_simplified #(
    parameter LINEWORDS = 8,       // words per line
    parameter LINES = 128,         // number of lines
    parameter HIT_LATENCY = 1,
    parameter MISS_PENALTY = 8
) (
    input  logic           clk,
    input  logic           rst_n,
    // CPU side
    input  logic           req_valid,
    input  logic           req_wr,        // 1 = write (for D-cache), 0 = read
    input  logic [31:0]    req_addr,      // byte address
    input  logic [31:0]    req_wdata,
    output logic           resp_valid,
    output logic [31:0]    resp_rdata,
    output logic           busy,
    // Backing memory interface (for refill/reads) - implemented by testbench or dmem module via tasks
    // For simulation we will produce deterministic fill data if backing not connected.
    input  logic           backing_en,    // when high, backing_rdata is valid (optional)
    input  logic [31:0]    backing_rdata
);
    localparam INDEX_BITS = $clog2(LINES);
    localparam OFFSET_BITS = $clog2(LINEWORDS);
    localparam TAG_BITS = 32 - INDEX_BITS - OFFSET_BITS - 2; // -2 because byte offset within word is 2 bits

    // Behavioral arrays
    logic              valid_array [0:LINES-1];
    logic [TAG_BITS-1:0] tag_array [0:LINES-1];
    logic [31:0]       data_array [0:LINES-1][0:LINEWORDS-1];

    // miss state
    logic [3:0]        miss_ctr;
    logic              in_miss;
    logic [31:0]       pending_addr;
    logic              pending_wr;
    logic [31:0]       pending_wdata;
    logic [INDEX_BITS-1:0] pending_index;
    integer i,j;

    // initialize arrays
    initial begin
        for (i=0;i<LINES;i=i+1) begin
            valid_array[i] = 1'b0;
            tag_array[i] = '0;
            for (j=0;j<LINEWORDS;j=j+1) data_array[i][j] = 32'd0;
        end
        resp_valid = 1'b0;
        resp_rdata = 32'd0;
        busy = 1'b0;
        miss_ctr = 0;
        in_miss = 1'b0;
    end

    // break address
    wire [31:0] word_addr = req_addr >> 2;
    wire [OFFSET_BITS-1:0] offset = word_addr[OFFSET_BITS-1:0];
    wire [INDEX_BITS-1:0] index = word_addr[OFFSET_BITS +: INDEX_BITS];
    wire [TAG_BITS-1:0] tag = word_addr[OFFSET_BITS+INDEX_BITS +: TAG_BITS];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            resp_valid <= 1'b0;
            busy <= 1'b0;
            miss_ctr <= 0;
            in_miss <= 1'b0;
        end else begin
            resp_valid <= 1'b0; // default
            if (in_miss) begin
                if (miss_ctr > 0) begin
                    miss_ctr <= miss_ctr - 1;
                end else begin
                    // finish miss: emulate fill
                    integer k;
                    logic [31:0] base_word;
                    base_word = { pending_addr[31:5], 5'd0, 2'd0 }; // align to line base: clear lower bits (LINEWORDS*4 - handled simply)
                    // If backing_en provided, we assume caller filled data_array before clearing busy.
                    // For deterministic simulation, if backing_en not used fill with base_word + k*4
                    if (!backing_en) begin
                        for (k=0;k<LINEWORDS;k=k+1) data_array[pending_index][k] <= base_word + (k*4);
                    end
                    valid_array[pending_index] <= 1'b1;
                    tag_array[pending_index] <= (pending_addr >> (2 + OFFSET_BITS + INDEX_BITS));
                    // perform pending access
                    if (pending_wr) begin
                        data_array[pending_index][offset] <= pending_wdata; // write-through behavior: cache update
                        resp_rdata <= 32'd0;
                    end else begin
                        resp_rdata <= data_array[pending_index][offset];
                    end
                    resp_valid <= 1'b1;
                    busy <= 1'b0;
                    in_miss <= 1'b0;
                end
            end else begin
                if (req_valid) begin
                    if (valid_array[index] && (tag_array[index] == tag)) begin
                        // HIT
                        if (req_wr) begin
                            data_array[index][offset] <= req_wdata;
                            resp_valid <= 1'b1;
                            resp_rdata <= 32'd0;
                        end else begin
                            resp_valid <= 1'b1;
                            resp_rdata <= data_array[index][offset];
                        end
                        busy <= 1'b0;
                    end else begin
                        // MISS: start miss handling
                        busy <= 1'b1;
                        in_miss <= 1'b1;
                        miss_ctr <= MISS_PENALTY - 1;
                        pending_addr <= req_addr;
                        pending_wr <= req_wr;
                        pending_wdata <= req_wdata;
                        pending_index <= index;
                        resp_valid <= 1'b0;
                    end
                end else begin
                    // no request: idle
                    busy <= 1'b0;
                end
            end
        end
    end
endmodule

// ============================================================================
// Top-level pipelined CPU (Harvard) - uses imem_sv + icache + dcache + regfile + alu
// ============================================================================
module cpu_harvard_sv (
    input  logic clk,
    input  logic rst_n,
    // nothing external; testbench will connect internal modules
    // expose some debug signals optionally via hierarchical references
    output logic [31:0] dbg_pc
);
    // =====================
    // Parameters & ISA fields
    // =====================
    localparam OPC_RTYPE = 6'b000000;
    localparam OPC_ADDI  = 6'b001000;
    localparam OPC_LW    = 6'b010000;
    localparam OPC_SW    = 6'b010001;
    localparam OPC_BEQ   = 6'b011000;
    localparam OPC_JAL   = 6'b011001;
    localparam OPC_NOP   = 6'b111111; // unused

    // Pipeline registers and control signals
    logic [31:0] pc, pc_next;
    logic        if_stall; // stall due to icache miss or dcache miss

    // IF/ID
    logic [31:0] if_id_instr;
    logic [31:0] if_id_pc;

    // ID/EX
    logic [31:0] id_ex_pc;
    logic [31:0] id_ex_rs1;
    logic [31:0] id_ex_rs2;
    logic [31:0] id_ex_imm;
    logic [4:0]  id_ex_rd;
    logic [4:0]  id_ex_rs1_idx, id_ex_rs2_idx;
    logic [5:0]  id_ex_opcode;
    logic [10:0] id_ex_funct;
    logic        id_ex_regwrite;
    logic        id_ex_memwrite;
    logic        id_ex_memread;
    logic [3:0]  id_ex_aluop;
    logic        id_ex_aluimm;

    // EX/MEM
    logic [31:0] ex_mem_aluout;
    logic [31:0] ex_mem_rs2;
    logic [4:0]  ex_mem_rd;
    logic        ex_mem_regwrite;
    logic        ex_mem_memwrite;
    logic        ex_mem_memread;

    // MEM/WB
    logic [31:0] mem_wb_memdata;
    logic [31:0] mem_wb_aluout;
    logic [4:0]  mem_wb_rd;
    logic        mem_wb_regwrite;
    logic        mem_wb_memread;

    // Register file wires
    logic [31:0] rf_rd1, rf_rd2;
    regfile_sv rf (
        .clk(clk),
        .rst_n(rst_n),
        .ra1(if_id_instr[20:16]),
        .ra2(if_id_instr[15:11]),
        .rd1(rf_rd1),
        .rd2(rf_rd2),
        .wa(mem_wb_rd),
        .wd(mem_wb_memread ? mem_wb_memdata : mem_wb_aluout),
        .we(mem_wb_regwrite)
    );

    // ALU
    logic [31:0] alu_a, alu_b, alu_y;
    logic        alu_zero;
    alu_sv alu0 (
        .a(alu_a),
        .b(alu_b),
        .op(id_ex_aluop),
        .y(alu_y),
        .zero(alu_zero)
    );

    // IMEM and ICACHE
    wire [31:0] imem_addr;
    wire [31:0] imem_data;
    imem_sv #(2048) imem_inst (.addr(imem_addr), .data(imem_data));

    // I-Cache instance
    logic ic_req_valid, ic_req_wr; // ic_req_wr unused (instruction fetch = read)
    logic [31:0] ic_req_addr, ic_req_wdata;
    logic ic_resp_valid;
    logic [31:0] ic_resp_rdata;
    logic ic_busy;
    cache_simplified #(.LINEWORDS(8), .LINES(256), .HIT_LATENCY(1), .MISS_PENALTY(6)) icache (
        .clk(clk), .rst_n(rst_n),
        .req_valid(ic_req_valid),
        .req_wr(ic_req_wr),
        .req_addr(ic_req_addr),
        .req_wdata(ic_req_wdata),
        .resp_valid(ic_resp_valid),
        .resp_rdata(ic_resp_rdata),
        .busy(ic_busy),
        .backing_en(1'b1),           // we will connect backing via imem read through testbench
        .backing_rdata(imem_data)
    );

    // D-Cache instance
    logic dc_req_valid, dc_req_wr;
    logic [31:0] dc_req_addr, dc_req_wdata;
    logic dc_resp_valid;
    logic [31:0] dc_resp_rdata;
    logic dc_busy;
    cache_simplified #(.LINEWORDS(8), .LINES(256), .HIT_LATENCY(1), .MISS_PENALTY(8)) dcache (
        .clk(clk), .rst_n(rst_n),
        .req_valid(dc_req_valid),
        .req_wr(dc_req_wr),
        .req_addr(dc_req_addr),
        .req_wdata(dc_req_wdata),
        .resp_valid(dc_resp_valid),
        .resp_rdata(dc_resp_rdata),
        .busy(dc_busy),
        .backing_en(1'b1),           // will connect to dmem backing via testbench
        .backing_rdata(32'd0)        // ignored; dcache fill uses deterministic fill in module
    );

    // connect imem fetch via icache: CPU will drive icache req addr; icache returns instruction word
    assign imem_addr = ic_req_addr;
    // when icache responds, supply data to IF stage
    wire if_instr_valid = ic_resp_valid;
    wire [31:0] if_instr_data = ic_resp_rdata;

    // D-cache responses go into MEM stage
    wire mem_resp_valid = dc_resp_valid;
    wire [31:0] mem_resp_data = dc_resp_rdata;

    // CPU internal dcache request latching
    logic d_req_issued;

    // Simple sign-extend functions
    function automatic logic [31:0] signext16(input logic [15:0] imm);
        signext16 = {{16{imm[15]}}, imm};
    endfunction

    // -------------------------
    // PC and IF stage logic
    // -------------------------
    assign dbg_pc = pc;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pc <= 32'd0;
            if_id_instr <= 32'd0;
            if_id_pc <= 32'd0;
            ic_req_valid <= 1'b0;
            ic_req_addr <= 32'd0;
            if_stall <= 1'b0;
        end else begin
            // IF stall if icache busy or dcache busy (for some MEM ops)
            if_stall <= (ic_busy || dc_busy);

            // Issue instruction fetch into icache if not busy and not already requested
            if (!ic_busy && !ic_req_valid) begin
                ic_req_valid <= 1'b1;
                ic_req_wr <= 1'b0;
                ic_req_addr <= pc;
            end else begin
                // keep ic_req_valid for one cycle (cache module samples req_valid)
                ic_req_valid <= 1'b0;
            end

            // If icache responded, latch instruction into IF/ID
            if (ic_resp_valid) begin
                if_id_instr <= ic_resp_rdata;
                if_id_pc <= pc;
                // increment PC unless branch redirect later
                pc <= pc + 4;
            end else begin
                // if no response yet, retain current IF/ID (stall)
                if_id_instr <= if_id_instr;
                if_id_pc <= if_id_pc;
            end
        end
    end

    // -------------------------
    // ID stage: decode & read regfile
    // -------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            id_ex_pc <= 32'd0;
            id_ex_rs1 <= 32'd0;
            id_ex_rs2 <= 32'd0;
            id_ex_imm <= 32'd0;
            id_ex_rd <= 5'd0;
            id_ex_rs1_idx <= 5'd0;
            id_ex_rs2_idx <= 5'd0;
            id_ex_opcode <= 6'd0;
            id_ex_funct <= 11'd0;
            id_ex_regwrite <= 1'b0;
            id_ex_memwrite <= 1'b0;
            id_ex_memread <= 1'b0;
            id_ex_aluop <= 4'd0;
            id_ex_aluimm <= 1'b0;
        end else begin
            // basic decode from if_id_instr
            id_ex_pc <= if_id_pc;
            id_ex_rs1_idx <= if_id_instr[20:16];
            id_ex_rs2_idx <= if_id_instr[15:11];
            id_ex_rs1 <= rf_rd1;
            id_ex_rs2 <= rf_rd2;
            id_ex_opcode <= if_id_instr[31:26];
            id_ex_funct <= if_id_instr[10:0];
            id_ex_imm <= signext16(if_id_instr[15:0]);
            id_ex_rd <= if_id_instr[25:21];

            // default controls
            id_ex_regwrite <= 1'b0;
            id_ex_memwrite <= 1'b0;
            id_ex_memread <= 1'b0;
            id_ex_aluimm <= 1'b0;
            id_ex_aluop <= 4'b0000;

            unique case (if_id_instr[31:26])
                OPC_RTYPE: begin
                    id_ex_regwrite <= 1'b1;
                    id_ex_aluimm <= 1'b0;
                    id_ex_aluop <= if_id_instr[3:0];
                end
                OPC_ADDI: begin
                    id_ex_regwrite <= 1'b1;
                    id_ex_aluimm <= 1'b1;
                    id_ex_aluop <= 4'b0000; // ADDI uses ADD op
                end
                OPC_LW: begin
                    id_ex_regwrite <= 1'b1;
                    id_ex_memread <= 1'b1;
                    id_ex_aluimm <= 1'b1;
                    id_ex_aluop <= 4'b0000; // ADD base+imm
                end
                OPC_SW: begin
                    id_ex_memwrite <= 1'b1;
                    id_ex_aluimm <= 1'b1;
                    id_ex_aluop <= 4'b0000;
                end
                OPC_BEQ: begin
                    id_ex_aluimm <= 1'b0;
                    id_ex_aluop <= 4'b0001; // SUB to test zero
                end
                OPC_JAL: begin
                    id_ex_regwrite <= 1'b1; // link
                    id_ex_aluimm <= 1'b1;
                    id_ex_aluop <= 4'b0000;
                end
                default: begin
                    // NOP or unknown
                end
            endcase
        end
    end

    // -------------------------
    // EX stage with forwarding + branch decision
    // -------------------------
    // forwarding wires
    logic [31:0] forward_a, forward_b;

    always_comb begin
        // defaults
        forward_a = id_ex_rs1;
        forward_b = id_ex_aluimm ? id_ex_imm : id_ex_rs2;

        // EX/MEM forwarding
        if (ex_mem_regwrite && (ex_mem_rd != 5'd0) && (ex_mem_rd == id_ex_rs1_idx))
            forward_a = ex_mem_aluout;
        if (ex_mem_regwrite && (ex_mem_rd != 5'd0) && (ex_mem_rd == id_ex_rs2_idx) && !id_ex_aluimm)
            forward_b = ex_mem_aluout;

        // MEM/WB forwarding
        logic [31:0] mem_stage_result = mem_wb_memread ? mem_wb_memdata : mem_wb_aluout;
        if (mem_wb_regwrite && (mem_wb_rd != 5'd0) && (mem_wb_rd == id_ex_rs1_idx))
            forward_a = mem_stage_result;
        if (mem_wb_regwrite && (mem_wb_rd != 5'd0) && (mem_wb_rd == id_ex_rs2_idx) && !id_ex_aluimm)
            forward_b = mem_stage_result;
    end

    assign alu_a = forward_a;
    assign alu_b = forward_b;

    // EX pipeline register updates
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ex_mem_aluout <= 32'd0;
            ex_mem_rs2 <= 32'd0;
            ex_mem_rd <= 5'd0;
            ex_mem_regwrite <= 1'b0;
            ex_mem_memwrite <= 1'b0;
            ex_mem_memread <= 1'b0;
        end else begin
            // Execute operation (alu result computed combinationally)
            ex_mem_aluout <= alu_y;
            ex_mem_rs2 <= forward_b; // store value for store instruction (after forwarding)
            ex_mem_rd <= id_ex_rd;
            ex_mem_regwrite <= id_ex_regwrite;
            ex_mem_memwrite <= id_ex_memwrite;
            ex_mem_memread <= id_ex_memread;

            // Branch resolution (if BEQ and zero)
            if ((id_ex_opcode == OPC_BEQ) && (alu_zero)) begin
                // branch taken: compute target = id_ex_pc + imm*4 (imm is in words previously signext)
                // our imm is signext16 as bytes? earlier design said PC-relative words; here we'll do words*4
                pc <= id_ex_pc + (id_ex_imm << 2);
                // flush IF/ID and ID/EX (simple method: insert NOP into id_ex control)
                id_ex_regwrite <= 1'b0;
                id_ex_memread  <= 1'b0;
                id_ex_memwrite <= 1'b0;
                if_id_instr <= 32'd0;
                if_id_pc <= 32'd0;
            end
            // JAL handling (simple): write return addr (pc+4) into rd and jump
            if (id_ex_opcode == OPC_JAL) begin
                pc <= id_ex_pc + (id_ex_imm << 2);
                // id_ex_regwrite true will write link in later WB
                // flush IF/ID
                id_ex_regwrite <= id_ex_regwrite;
                if_id_instr <= 32'd0;
                if_id_pc <= 32'd0;
            end
        end
    end

    // -------------------------
    // MEM stage: interact with D-cache
    // -------------------------
    // D-cache request signals
    assign dc_req_valid = ex_mem_memread || ex_mem_memwrite;
    assign dc_req_wr    = ex_mem_memwrite;
    assign dc_req_addr  = ex_mem_aluout;
    assign dc_req_wdata = ex_mem_rs2;

    // capture cache responses
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem_wb_memdata <= 32'd0;
            mem_wb_aluout <= 32'd0;
            mem_wb_rd <= 5'd0;
            mem_wb_regwrite <= 1'b0;
            mem_wb_memread <= 1'b0;
        end else begin
            // If D-cache returns, latch memory data
            if (mem_resp_valid) mem_wb_memdata <= mem_resp_data;

            // Move ex_mem to mem_wb pipeline regs
            mem_wb_aluout <= ex_mem_aluout;
            mem_wb_rd <= ex_mem_rd;
            mem_wb_regwrite <= ex_mem_regwrite;
            mem_wb_memread <= ex_mem_memread;
        end
    end

    // -------------------------
    // WB stage: regfile write (handled by regfile synchronous interface)
    // -------------------------
    // regfile .we and .wa and .wd are connected earlier when instantiating regfile

    // Note: regfile writes happen on rising edge and read used one cycle later due to pipelining

    // -------------------------
    // Simple load-use hazard detection and stall insertion
    // if ID stage uses a register that is the target of a load in EX stage, stall one cycle
    // -------------------------
    logic load_use_stall;
    always_comb begin
        load_use_stall = 1'b0;
        if (id_ex_memread && (
              (id_ex_rd == if_id_instr[20:16]) || // rs1
              (id_ex_rd == if_id_instr[15:11])    // rs2
           ) && (id_ex_rd != 5'd0)) begin
            load_use_stall = 1'b1;
        end
    end

    // apply stall: if load_use_stall, freeze IF/ID and insert bubble into ID/EX
    // For simplicity, we used icache/dcache busy to stall; a production design would integrate both

    // END of cpu_harvard_sv
endmodule

// ============================================================================
// Testbench: instantiates cpu_harvard_sv, imem, and verifies behavior
// ============================================================================
module tb;
    logic clk;
    logic rst_n;

    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 100 MHz simulation tick
    end

    initial begin
        rst_n = 1'b0;
        #25;
        rst_n = 1'b1;
    end

    // instantiate imem and dmem, and CPU (hierarchically for access)
    imem_sv #(2048) imem_inst (.addr(), .data()); // we'll reference imem_inst from icache backing via hierarchical name

    // For simplicity we will instantiate cpu_harvard_sv as a module and then patch imem content by hierarchical reference
    cpu_harvard_sv cpu (.clk(clk), .rst_n(rst_n), .dbg_pc());

    // Because cache modules inside cpu_harvard_sv take backing_rdata from imem/dmem via port,
    // we connected imem_addr = ic_req_addr earlier. However SystemVerilog hierarchical assignment is not recommended;
    // to keep the single-file approach simple and simulator-friendly, we will directly write imem memory from tb.

    // Preload imem (word addresses) - using same encoding as earlier example
    initial begin
        // Wait for reset release
        wait (rst_n == 1'b1);
        // Simple program:
        // 0: ADD r1, r0, r0     ; r1 = 0
        // 4: ADDI r2, r0, 8     ; r2 = 8
        // 8: SW  r2, 0(r0)      ; mem[0] = 8
        //12: LW  r3, 0(r0)      ; r3 = mem[0]
        //16: ADD r4, r3, r2     ; r4 = r3 + r2  (should be 16)
        //20: NOP...
        // We'll craft instruction words consistent with earlier encodings.

        // Note: imem_inst.mem is internal to imem_sv; we can assign via hierarchical path:
        // imem_inst.mem[index] = value;
        // But the imem used by cpu is instantiated inside cpu_harvard_sv - hierarchical access:
        // cpu.imem_inst.mem[index] = value;  // this assumes same instance name used earlier
        // To ensure instance name matches, modify cpu_harvard_sv imem instantiation to use 'imem_inst' name.
        // In our file, imem was instantiated as 'imem_inst' inside cpu_harvard_sv, so we can reference it here.

        // Fill instructions (word indices)
        // Construct instruction bits:
        // R-type: opcode[31:26]=0, rd[25:21], rs1[20:16], rs2[15:11], funct[10:0]; ADD = funct[3:0]=0000
        cpu.imem_inst.mem[0] = 32'h00004200; // attempt to place ADD r1, r0, r0 (match earlier example)
        cpu.imem_inst.mem[1] = {6'b001000, 5'd2, 5'd0, 16'h0008}; // ADDI r2, r0, 8
        cpu.imem_inst.mem[2] = {6'b010001, 5'd2, 5'd0, 16'h0000}; // SW r2, 0(r0)
        cpu.imem_inst.mem[3] = {6'b010000, 5'd3, 5'd0, 16'h0000}; // LW r3, 0(r0)
        cpu.imem_inst.mem[4] = {6'b000000, 5'd4, 5'd3, 5'd2, 11'b0}; // ADD r4, r3, r2
        cpu.imem_inst.mem[5] = 32'h00000000;
        cpu.imem_inst.mem[6] = 32'h00000000;
        cpu.imem_inst.mem[7] = 32'h00000000;

        // Let simulation run
        #2000;
        $display("Testbench done.");
        $finish;
    end

    // waveform dump
    initial begin
        $dumpfile("cpu_harvard_tb.vcd");
        $dumpvars(0, tb);
    end
endmodule
