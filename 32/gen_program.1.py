# Generate program.hex for two cores with separate instruction regions
# Core 0: increments, Core 1: decrements
# Randomized interleaving applies to shared data accesses

import random

NUM_ITER = 500      # number of atomic operations per core
OFFSET_CORE1 = 256  # starting address for Core 1 instructions

# Build instruction sequences for each core
core0_instr = []
core1_instr = []

for _ in range(NUM_ITER):
    # Core 0: increment shared counter
    core0_instr.extend([
        "10000001",  # LL R1, [0x1000]
        "30010001",  # ADD R1, R1, #1
        "20010001",  # SC R1, [0x1000]
        "50000000",  # BRZ retry
        "00000000"   # NOP padding
    ])
    # Core 1: decrement shared counter
    core1_instr.extend([
        "10000001",  # LL R1, [0x1000]
        "3001FFFE",  # ADD R1, R1, #-1
        "20010001",  # SC R1, [0x1000]
        "50000080",  # BRZ retry
        "00000000"   # NOP padding
    ])

# Shuffle the shared memory operations for more realistic atomic stress
combined_shared = core0_instr + core1_instr
random.shuffle(combined_shared)

# Now, separate instruction memory regions per core
# Core 0 region: first OFFSET_CORE1 entries
instr_mem0 = combined_shared[:OFFSET_CORE1] + core0_instr
instr_mem1 = combined_shared[:OFFSET_CORE1] + core1_instr

# Pad both memories to the same size for $readmemh convenience
max_len = max(len(instr_mem0), len(instr_mem1))
instr_mem0 += ["00000000"] * (max_len - len(instr_mem0))
instr_mem1 += ["00000000"] * (max_len - len(instr_mem1))

# Write to program.hex with Core 0 region first, then Core 1 region
with open("program.hex", "w") as f:
    f.write("; ------------------ Core 0 Instructions (instr_mem0) ------------------\n")
    for instr in instr_mem0:
        f.write(instr + "\n")
    f.write("; ------------------ Core 1 Instructions (instr_mem1) ------------------\n")
    for instr in instr_mem1:
        f.write(instr + "\n")

print("program.hex generated successfully:")
print(f" - Core 0 instructions: {len(instr_mem0)} lines")
print(f" - Core 1 instructions: {len(instr_mem1)} lines")
