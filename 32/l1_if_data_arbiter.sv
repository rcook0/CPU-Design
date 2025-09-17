`timescale 1ns/1ps

module l1_if_data_arbiter (
    input  mem_req_t  if_req,
    output mem_resp_t if_resp,

    input  mem_req_t  data_req,
    output mem_resp_t data_resp,

    // Combined request to L1 core
    output mem_req_t  active_req,
    input  mem_resp_t active_resp
);
    always_comb begin
        // Defaults
        active_req = '0;
        if_resp    = '0;
        data_resp  = '0;

        if (data_req.valid) begin
            active_req = data_req;
            data_resp  = active_resp;
        end else if (if_req.valid) begin
            active_req = if_req;
            if_resp    = active_resp;
        end
    end
endmodule
