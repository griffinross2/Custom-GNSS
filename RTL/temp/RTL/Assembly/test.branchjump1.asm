#----------------------------------------------------------
# RISC-V Assembly
#----------------------------------------------------------
#--------------------------------------
# Test branch and jumps
#--------------------------------------
  .text
_start:
  lui x3, 0x0000B
  ori   x4, x0, 0x75C
  add x4, x4, x3
  addi x4, x4, 0x300
   ori   x10, x0, 0x080
   ori   x16, x0, %lo(jmpR)
   beq   x0, x0, braZ
   sw    x4, 0(x10)
 braZ:
  jal   braR
  sw    x4, 4(x10)
 end:
  sw    x1, 16(x10)
   ebreak
braR:
  or    x11, x0, x1
   sw    x1, 8(x10)
   jal   jmpR
  sw    x4, 12(x10)
 jmpR:
  bne   x1, x11, end
   ebreak
