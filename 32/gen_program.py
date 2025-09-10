# Generate program.hex for 2-core self-checking LL/SC test

NUM_ITER = 500  # number of atomic iterations per core
OFFSET_CORE1 = 128  # starting address for core 1 program

with open("program.hex", "w") as f:
    # ------------------ Core 0 (increments) ------------------
    for _ in range(NUM_ITER):
        f.write("10000001\n")  # LL R1, [0x1000]
        f.write("30010001\n")  # ADD R1, R1, #1
        f.write("20010001\n")  # SC R1, [0x1000]
        f.write("50000000\n")  # BRZ retry
        f.write("00000000\n")  # NOP padding

    # Fill NOPs to offset Core 1 start
    while f.tell() // 9 < OFFSET_CORE1:  # 9 bytes per line approx (safe)
        f.write("00000000\n")

    # ------------------ Core 1 (decrements) ------------------
    for _ in range(NUM_ITER):
        f.write("10000001\n")  # LL R1, [0x1000]
        f.write("3001FFFE\n")  # ADD R1, R1, #-1
        f.write("20010001\n")  # SC R1, [0x1000]
        f.write("50000080\n")  # BRZ retry
        f.write("00000000\n")  # NOP padding

print("program.hex generated successfully.")
