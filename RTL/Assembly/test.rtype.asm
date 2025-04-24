#----------------------------------------------------------
# RISC-V Assembly
#----------------------------------------------------------

  #------------------------------------------------------------------
  # R-type Instruction (ALU) Test Program
  #------------------------------------------------------------------

  .text

_start:
  ori   x4,x0,0x269
  lui   x3, 0x0000D
  add x4, x4, x3
  ori   x10,x0,0x7F1
  lui   x3, 0x00003
  add x10, x10, x3
  
  ori   x21,x0,0x700
  ori   x22,x0,0xF0
 
# Now running all R type instructions
  #or    x11,x4,x10
   and   x12,x4,x10
   andi  x13,x4,0xF
   add  x14,x4,x10
   addi x15,x11,0x740
   sub  x5,x12,x10
   xor   x6,x13,x10
   xori  x7,x4,0x33f
   ori   x31,x0,4 
   sll  x28,x4,x31
   ori   x31,x0,5
   srl  x29,x4,x31
# nor   x30,x4,x10 # No Nor in RV32
#  or x30, x4, x10
#  not x30, x30

# Store them to verify the results
  # sw    x30,0(x22)
   sw    x11,0(x21)
   sw    x12,4(x21)
   sw    x13,8(x21)
   sw    x14,12(x21)
   sw    x15,16(x21)
   sw    x5,20(x21)
   sw    x6,24(x21)
   sw    x7,28(x21)
   sw    x28,32(x21)
   #sw    x29,36(x21)
   ebreak
# that's all
