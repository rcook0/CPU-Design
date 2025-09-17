// regfile_dualport.v
// Parameterized 32-bit, 32-entry register file with one write port, two read ports.
// Synchronous write, asynchronous read (common for FPGA BRAM inference).
module regfile_dualport #(
    parameter DATA_W = 32,
    parameter ADDR_W = 5
) (
    input  wire                      clk,
    input  wire                      rst_n,

    // Write port
    input  wire                      we,
    input  wire [ADDR_W-1:0]         waddr,
    input  wire [DATA_W-1:0]         wdata,

    // Read port A
    input  wire [ADDR_W-1:0]         raddr_a,
    output wire [DATA_W-1:0]         rdata_a,

    // Read port B
    input  wire [ADDR_W-1:0]         raddr_b,
    output wire [DATA_W-1:0]         rdata_b
);

    // Simple reg array
    reg [DATA_W-1:0] mem [0:(1<<ADDR_W)-1];

    // Optional: zero-register behavior (e.g., index 0 reads as zero and writes ignored)
    // assign rdata_a = (raddr_a == 0) ? {DATA_W{1'b0}} : mem[raddr_a];
    // assign rdata_b = (raddr_b == 0) ? {DATA_W{1'b0}} : mem[raddr_b];

    // Asynchronous read (tool will infer block RAM or LUT RAM depending on indexing)
    assign rdata_a = mem[raddr_a];
    assign rdata_b = mem[raddr_b];

    // Synchronous write
    always @(posedge clk) begin
        if (!rst_n) begin
            // Optional: initialize to zero (synthesis tool dependent)
            // integer i; for (i = 0; i < (1<<ADDR_W); i = i + 1) mem[i] <= {DATA_W{1'b0}};
        end else begin
            if (we) begin
                mem[waddr] <= wdata;
            end
        end
    end

endmodule
