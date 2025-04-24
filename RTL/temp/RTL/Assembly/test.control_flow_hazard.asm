.text

_start:
addi x2, x0, 4
addi x3, x0, 4
beq x2, x3, true
# should start to execute but not finish
addi x3, x0, 1
sw x3, 0x100(x0)
ebreak

true:
  addi x3, x0, 1
  sw x3, 0x200(x0)
ebreak
