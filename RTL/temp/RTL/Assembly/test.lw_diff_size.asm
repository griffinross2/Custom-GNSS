.text
_start:
    # Store some different values
    li x1, 0x12345678
    li x2, 0xFFFFFFFF
    sw x1, 0x100(x0)
    sw x2, 0x104(x0)

    # Test simple loads
    lw x3, 0x100(x0)
    lw x4, 0x104(x0)
    lh x5, 0x100(x0)
    lh x6, 0x102(x0)
    lb x7, 0x104(x0)
    lb x8, 0x105(x0)
    lhu x9, 0x104(x0)
    lbu x10, 0x104(x0)

    # Store the results
    sw x3, 0x200(x0)
    sw x4, 0x204(x0)
    sw x5, 0x208(x0)
    sw x6, 0x20c(x0)
    sw x7, 0x210(x0)
    sw x8, 0x214(x0)
    sw x9, 0x218(x0)
    sw x10, 0x21c(x0)

    # Test load use
    lw x11, 0x100(x0)
    lw x12, 0x104(x0)
    add x13, x11, x12

    lhu x14, 0x100(x0)
    lhu x15, 0x104(x0)
    add x16, x14, x15

    # Store the results
    sw x11, 0x300(x0)
    sw x12, 0x304(x0)
    sw x13, 0x308(x0)
    sw x14, 0x30c(x0)
    sw x15, 0x310(x0)
    sw x16, 0x314(x0)
    
    ebreak
