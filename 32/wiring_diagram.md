                ┌─────────────────────────┐
                │       instr_mem0        │
                │  (program.hex)          │
                └─────────┬───────────────┘
                          │ instr_core0
                          ▼
                ┌─────────────────────────┐
                │    core_pipeline0       │
                │ pc_in=pc_core0          │
                │ mem_req_valid0, addr0,  │
                │ wdata0, wr0, atomic0   │
                │ mem_rdata0, resp_valid0│
                │ sc_success0             │
                └─────────┬───────────────┘
                          │ memory request
                          ▼
                ┌─────────────────────────┐
                │      L1_cache0          │
                │ req_valid, addr, wdata  │
                │ wr, atomic               │
                │ resp_rdata, resp_valid   │
                │ sc_success               │
                │ snoop_if                 │
                └─────────┬───────────────┘
                          │ L1 → L2 request
                          ▼
                ┌─────────────────────────┐
                │        L2 Cache          │
                │ shared backing memory    │
                │ resp_rdata, resp_valid   │
                └─────────┬───────────────┘
                          │
                          │ L2 → L1 response
                          ▲
                ┌─────────┴───────────────┐
                │      L1_cache1          │
                │ req_valid, addr, wdata  │
                │ wr, atomic               │
                │ resp_rdata, resp_valid   │
                │ sc_success               │
                │ snoop_if                 │
                └─────────┬───────────────┘
                          │ memory request
                          ▼
                ┌─────────────────────────┐
                │    core_pipeline1       │
                │ pc_in=pc_core1          │
                │ mem_req_valid1, addr1,  │
                │ wdata1, wr1, atomic1    │
                │ mem_rdata1, resp_valid1 │
                │ sc_success1             │
                └─────────┬───────────────┘
                          │ instr_core1
                          ▲
                ┌─────────────────────────┐
                │       instr_mem1        │
                │  (program.hex)          │
                └─────────────────────────┘
