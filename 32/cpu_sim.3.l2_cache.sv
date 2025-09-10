`timescale 1ns/1ps

module l2_cache #(
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 32,
    parameter LINES      = 256,
    parameter LINEWORDS  = 8,
    parameter LATENCY    = 4
)(
    input  logic clk,
    input  logic rst_n,
    // Request from L1
    input  logic         req_valid,
    input  logic         req_wr,
    input  logic [ADDR_WIDTH-1:0] req_addr,
    input  logic [DATA_WIDTH-1:0] req_wdata,
    output logic         resp_valid,
    output logic [DATA_WIDTH-1:0] resp_rdata,
    // Backing memory
    inout  logic [DATA_WIDTH-1:0] mem [0:16383]
);

    // ----------------------------
    // Internal registers
    // ----------------------------
    logic [LATENCY-1:0] pipeline_valid;
    logic [DATA_WIDTH-1:0] pipeline_rdata [0:LATENCY-1];
    logic [ADDR_WIDTH-1:0] pipeline_addr [0:LATENCY-1];
    logic pipeline_wr [0:LATENCY-1];

    integer i;
    initial begin
        pipeline_valid = 0;
        resp_valid = 0;
        resp_rdata = 0;
    end

    // ----------------------------
    // Simple L2 access
    // ----------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            pipeline_valid <= 0;
            resp_valid <= 0;
            resp_rdata <= 0;
        end else begin
            // Shift pipeline
            for(i=LATENCY-1;i>0;i=i-1) begin
                pipeline_valid[i] <= pipeline_valid[i-1];
                pipeline_rdata[i] <= pipeline_rdata[i-1];
                pipeline_addr[i] <= pipeline_addr[i-1];
                pipeline_wr[i] <= pipeline_wr[i-1];
            end
            pipeline_valid[0] <= req_valid;
            pipeline_addr[0] <= req_addr;
            pipeline_wr[0] <= req_wr;
            if(req_valid) begin
                if(req_wr) mem[req_addr>>2] <= req_wdata;
                pipeline_rdata[0] <= mem[req_addr>>2];
            end

            // Output
            resp_valid <= pipeline_valid[LATENCY-1];
            resp_rdata <= pipeline_rdata[LATENCY-1];
        end
    end

endmodule
