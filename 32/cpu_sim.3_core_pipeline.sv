module core_pipeline #(
    parameter ADDR_WIDTH=32,
    parameter DATA_WIDTH=32,
    parameter REG_COUNT=32
)(
    input logic clk, rst_n,
    output logic [ADDR_WIDTH-1:0] mem_addr,
    output logic mem_wr,
    output logic [DATA_WIDTH-1:0] mem_wdata,
    input  logic [DATA_WIDTH-1:0] mem_rdata,
    input  logic mem_ready,
    input  logic [1:0] core_id,
    // snoop bus
    input  logic snoop_valid,
    input  logic [ADDR_WIDTH-1:0] snoop_addr,
    input  logic [1:0] snoop_source_id,
    output logic snoop_ack
);

    // -------------------
    // Registers
    // -------------------
    logic [DATA_WIDTH-1:0] regs[0:REG_COUNT-1];
    logic [ADDR_WIDTH-1:0] pc;

    // -------------------
    // Pipeline registers
    // -------------------
    typedef struct packed {
        logic [31:0] instr;
        logic [ADDR_WIDTH-1:0] pc;
        logic valid;
    } if_id_t;
    if_id_t if_id;

    typedef struct packed {
        logic [31:0] instr;
        logic [ADDR_WIDTH-1:0] pc;
        logic [DATA_WIDTH-1:0] rs1_val, rs2_val;
        logic [4:0] rd;
        logic mem_rd, mem_wr;
        logic atomic;
        logic valid;
    } id_ex_t;
    id_ex_t id_ex;

    typedef struct packed {
        logic [31:0] instr;
        logic [ADDR_WIDTH-1:0] pc;
        logic [DATA_WIDTH-1:0] alu_out, rs2_val;
        logic [4:0] rd;
        logic mem_rd, mem_wr;
        logic atomic;
        logic valid;
    } ex_mem_t;
    ex_mem_t ex_mem;

    typedef struct packed {
        logic [31:0] instr;
        logic [ADDR_WIDTH-1:0] pc;
        logic [DATA_WIDTH-1:0] mem_data, alu_out;
        logic [4:0] rd;
        logic valid;
    } mem_wb_t;
    mem_wb_t mem_wb;

    // -------------------
    // Instruction Fetch (IF)
    // -------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pc <= 0;
            if_id <= '{instr:0, pc:0, valid:0};
        end else begin
            // simplified: fetch 32-bit instr from memory via unified cache
            if_id.instr <= mem_rdata; // assume mem_ready=1
            if_id.pc <= pc;
            if_id.valid <= 1;
            pc <= pc + 4;
        end
    end

    // -------------------
    // Instruction Decode (ID)
    // -------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) id_ex <= '0;
        else begin
            id_ex.instr <= if_id.instr;
            id_ex.pc <= if_id.pc;
            id_ex.valid <= if_id.valid;
            id_ex.rd <= if_id.instr[11:7];
            id_ex.rs1_val <= regs[if_id.instr[19:15]];
            id_ex.rs2_val <= regs[if_id.instr[24:20]];
            id_ex.mem_rd <= (if_id.instr[6:0]==7'b0000011);
            id_ex.mem_wr <= (if_id.instr[6:0]==7'b0100011);
            id_ex.atomic <= (if_id.instr[6:0]==7'b1111111); // custom atomic opcode
        end
    end

    // -------------------
    // Execute (EX)
    // -------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) ex_mem <= '0;
        else begin
            ex_mem <= '0;
            ex_mem.instr <= id_ex.instr;
            ex_mem.pc <= id_ex.pc;
            ex_mem.rd <= id_ex.rd;
            ex_mem.mem_rd <= id_ex.mem_rd;
            ex_mem.mem_wr <= id_ex.mem_wr;
            ex_mem.atomic <= id_ex.atomic;
            ex_mem.valid <= id_ex.valid;
            // simplified ALU: ADD
            ex_mem.alu_out <= id_ex.rs1_val + id_ex.rs2_val;
            ex_mem.rs2_val <= id_ex.rs2_val;
        end
    end

    // -------------------
    // Memory (MEM)
    // -------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) mem_wb <= '0;
        else begin
            mem_wb <= '0;
            mem_wb.instr <= ex_mem.instr;
            mem_wb.pc <= ex_mem.pc;
            mem_wb.rd <= ex_mem.rd;
            mem_wb.valid <= ex_mem.valid;
            mem_wb.alu_out <= ex_mem.alu_out;

            if (ex_mem.mem_rd) begin
                mem_addr <= ex_mem.alu_out;
                mem_wr <= 0;
                mem_data <= 0;
            end
            if (ex_mem.mem_wr || ex_mem.atomic) begin
                mem_addr <= ex_mem.alu_out;
                mem_wr <= 1;
                mem_data <= ex_mem.rs2_val;
            end
            mem_wb.mem_data <= mem_rdata;
        end
    end

    // -------------------
    // Writeback (WB)
    // -------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i=0;i<REG_COUNT;i++) regs[i]=0;
        end else begin
            if (mem_wb.valid && mem_wb.rd != 0)
                regs[mem_wb.rd] <= mem_wb.mem_data; // simplified: load result
        end
    end

endmodule
