# Generate program.hex with randomized interleaved LL/SC atomic ops

import random

NUM_ITER = 500       # Number of atomic ops per core
OFFSET_CORE1 = 128   # Start address for Core 1 (optional for alignment)

# Create lists of instructions for each core
core0_instr = []
core1_instr = []

for _ in range(NUM_ITER):
    # Core 0: increment
    core0_instr.extend([
        "10000001",  # LL R1, [0x1000]
        "30010001",  # ADD R1, R1, #1
        "20010001",  # SC R1, [0x1000]
        "50000000",  # BRZ retry
        "00000000"   # NOP
    ])

    # Core 1: decrement
    core1_instr.extend([
        "10000001",  # LL R1, [0x1000]
        "3001FFFE",  # ADD R1, R1, #-1
        "20010001",  # SC R1, [0x1000]
        "50000080",  # BRZ retry
        "00000000"   # NOP
    ])

# Interleave the instructions randomly
all_instr = core0_instr + core1_instr
random.shuffle(all_instr)

# Optional: pad to offset core1 start if using fixed PC addresses
while len(all_instr) < OFFSET_CORE1:
    all_instr.insert(0, "00000000")  # prepend NOPs

# Write to program.hex
with open("program.hex", "w") as f:
    for instr in all_instr:
        f.write(instr + "\n")

print("Randomized program.hex generated with {} instructions.".format(len(all_instr)))
