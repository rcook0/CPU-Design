package mc_common_pkg;
  parameter int XLEN   = 32;
  parameter int NCORES = 2;
  typedef logic [XLEN-1:0] xlen_t;
  typedef struct packed { logic valid, wr, atomic, is_ifetch; logic [1:0] size; xlen_t addr, wdata; } mem_req_t;
  typedef struct packed { logic valid, sc_success; xlen_t rdata; } mem_resp_t;
endpackage
