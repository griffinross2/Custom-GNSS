# Test the Command & Status Registers
.text
_start:
    # Do whatever
    li x2, 1234
    li x3, 5678
    li x5, 9876
    li x6, 0
    add x4, x2, x3
    sw x4, 0x100(x0)

    # Swap x4 with the mscratch CSR
    csrrw x4, mscratch, x4    # mscratch now 1234+5678
    sw x4, 0x104(x0)        # Store the original mscratch (0)

    # Swap mscratch to the x5 register
    csrrw x5, mscratch, x5    # x5 now 1234+5678, mscratch now 9876
    sw x5, 0x108(x0)        # Store the original mscratch (1234+5678)

    # Perform an operation using the mscratch CSR that will rely on forwarding
    csrrw x6, mscratch, x0    # x6 now 9876, mscratch now 0
    addi x6, x6, 1          # x6 now 9877, 1 if forwarding failed
    sw x6, 0x10C(x0)        # Store the result

    # Perform an operation setting the mscratch CSR that will rely on forwarding
    li x7, 2
    li x8, 3
    add x9, x7, x8
    csrrw x0, mscratch, x9    # mscratch now 5 (if forwarding worked)
    csrrw x10, mscratch, x0   # x10 now 5 (if forwarding worked)
    sw x10, 0x110(x0)       # Store the result

    # Test the set and clear CSR instructions
    # Currently mscratch is 0
    csrrsi x0, mscratch, 7    # mscratch now 7
    csrrw x11, mscratch, x0   # x11 now 7
    sw x11, 0x114(x0)       # Store the result

    # Set mscratch
    li x12, 9
    csrrw x0, mscratch, x12   # mscratch now 9
    csrrci x0, mscratch, 8    # mscratch now 1
    csrrw x13, mscratch, x0   # x13 now 1
    sw x13, 0x118(x0)       # Store the result

    ebreak
