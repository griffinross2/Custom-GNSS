.text
_start:
    addi x10, x0, 1
    addi x11, x0, 1
    add x12, x11, x10
    add x13, x10, x11
    addi x14, x13, 1
    addi x15, x14, 1
    sw x15, 0x10C(x0)
    addi x1, x0, 10
    addi x2, x0, 10
    beq x1, x2, branch
    ebreak
branch:
    addi ra, x0, 52
    jalr ra, 0(ra)
jump:
    addi x3, x0, 1
    sw x3, 0x100(x0)
    sw x3, 0x104(x0)
    sw ra, 0x108(x0)
    addi x2, x0, 7
    lw x1, 0x100(x0)
    addi x2, x1, 5
    sw x2, 0x100(x0)
    ebreak
