                   ┌───────────────────────────────┐
                   │            Core 0              │
                   │  - IF stage → if_req0          │
                   │  - MEM stage → data_req0       │
                   └───────────┬───────────────────┘
                               │
                               ▼
                   ┌───────────────────────────────┐
                   │   L1 Cache Unified (Core 0)   │
                   │ ┌───────────────────────────┐ │
                   │ │ IF/DATA Arbiter           │ │
                   │ │ - Priority: DATA > IF     │ │
                   │ └───────────┬───────────────┘ │
                   │             │ active_req0      │
                   │     Tag/Data Arrays + MESI    │
                   │     LL/SC reservations        │
                   └───────────┬───────────────────┘
                               │
                               ▼
                               │ req0/resp0
                               │
                   ┌───────────┴───────────────┐
                   │                           │
                   ▼                           ▼
    ┌───────────────────────────────┐   ┌───────────────────────────────┐
    │  L2 Two-Master Arbiter        │   │  Shared L2 Cache              │
    │  - Round robin between cores  │──▶│  Backing memory array (mem[]) │
    └───────────────────────────────┘   └───────────────────────────────┘
                   ▲
                   │ req1/resp1
                               │
                               ▼
                   ┌───────────────────────────────┐
                   │   L1 Cache Unified (Core 1)   │
                   │ ┌───────────────────────────┐ │
                   │ │ IF/DATA Arbiter           │ │
                   │ │ - Priority: DATA > IF     │ │
                   │ └───────────┬───────────────┘ │
                   │             │ active_req1      │
                   │     Tag/Data Arrays + MESI    │
                   │     LL/SC reservations        │
                   └───────────┬───────────────────┘
                               │
                               ▼
                   ┌───────────────────────────────┐
                   │            Core 1              │
                   │  - IF stage → if_req1          │
                   │  - MEM stage → data_req1       │
                   └───────────────────────────────┘
