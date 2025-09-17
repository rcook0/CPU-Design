`timescale 1ns/1ps

module l2_two_master_arbiter (
    input  logic       clk, rst_n,

    // Core0 L1 → L2
    input  mem_req_t   req0,
    output mem_resp_t  resp0,

    // Core1 L1 → L2
    input  mem_req_t   req1,
    output mem_resp_t  resp1,

    // Downstream L2 interface
    output mem_req_t   l2_req,
    input  mem_resp_t  l2_resp
);

    logic turn;  // round-robin toggle

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) turn <= 0;
        else if (req0.valid && req1.valid)
            turn <= ~turn;  // alternate only when both request
    end

    always_comb begin
        l2_req   = '0;
        resp0    = '0;
        resp1    = '0;

        if (req0.valid && (!req1.valid || !turn)) begin
            l2_req = req0;
            resp0  = l2_resp;
        end else if (req1.valid) begin
            l2_req = req1;
            resp1  = l2_resp;
        end
    end
endmodule
