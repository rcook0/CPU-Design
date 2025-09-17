# generate_program_hex.py
# Generates program.hex for two cores with randomized LL/SC atomic operations

import random

NUM_OPERATIONS = 100      # number of LL/SC operations per core
OFFSET_CORE1 = 256         # instruction offset for core 1

# Helper function to create LL/SC increment/decrement sequence
def gen_core_instructions(op_type="inc"):
    instrs = []
    for _ in range(NUM_OPERATIONS):
        if op_type == "inc":
            # LL, ADD #1, SC, BRZ retry, NOP padding
            instrs.extend([
                "10001000",  # LL R1, [0x1000]
                "30010001",  # ADD R1, R1, #1
                "20010001",  # SC R1, [0x1000]
                "50000000",  # BRZ retry
                "00000000",  # NOP
            ])
        else:
            # LL, ADD #-1, SC, BRZ retry, NOP padding
            instrs.extend([
                "10001000",  # LL R1, [0x1000]
                "3001FFFE",  # ADD R1, R1, #-1
                "20010001",  # SC R1, [0x1000]
                "50000080",  # BRZ retry
                "00000000",  # NOP
            ])
    return instrs

# Generate instruction streams for both cores
core0_instrs = gen_core_instructions("inc")
core1_instrs = gen_core_instructions("dec")

# Randomize interleaving of shared operations
combined = core0_instrs[:NUM_OPERATIONS*5//2] + core1_instrs[:NUM_OPERATIONS*5//2]
random.shuffle(combined)

# Prepend random interleaved instructions to core-specific sequences
instr_mem0 = combined + core0_instrs
instr_mem1 = combined + core1_instrs

# Pad both memories to the same length
max_len = max(len(instr_mem0), len(instr_mem1))
instr_mem0 += ["00000000"] * (max_len - len(instr_mem0))
instr_mem1 += ["00000000"] * (max_len - len(instr_mem1))

# Write to program.hex with clear sections
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
