`timescale 1ns/1ps

module instruction_memory #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32,
    parameter SIZE = 256,
    parameter PROGRAM_FILE = "program.hex"
)(
    input  logic [ADDR_WIDTH-1:0] addr,
    output logic [DATA_WIDTH-1:0] data
);

    // ----------------------------
    // Memory array
    // ----------------------------
    logic [DATA_WIDTH-1:0] mem [0:SIZE-1];

    // ----------------------------
    // Load program from file
    // ----------------------------
    initial begin
        $display("Loading program from %s", PROGRAM_FILE);
        $readmemh(PROGRAM_FILE, mem);
    end

    // ----------------------------
    // Instruction fetch
    // ----------------------------
    assign data = mem[addr[7:0]];

endmodule
