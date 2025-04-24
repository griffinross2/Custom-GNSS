.text

_start:
    li x1, 125345
    li x2, -32453
    mul x3, x1, x2
    mulhu x4, x1, x2
    mulhsu x5, x1, x2
    mulh x6, x1, x2
    sw x1, 0x100(x0)
    sw x2, 0x104(x0)
    sw x3, 0x108(x0)
    sw x4, 0x10c(x0)
    sw x5, 0x110(x0)
    sw x6, 0x114(x0)
    ebreak
