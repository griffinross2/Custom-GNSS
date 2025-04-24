.text
_start:
    li x1, 0x12345678
    li x2, 0x87654321
    sw x1, 0x100(x0)
    sh x1, 0x104(x0)
    sh x2, 0x106(x0)
    sb x1, 0x108(x0)
    sb x2, 0x109(x0)

    ebreak
