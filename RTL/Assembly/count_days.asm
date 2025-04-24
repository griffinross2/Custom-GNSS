# Lab 2 Calculate Days Program in RISC-V Assembly
# Estimates the number of days since 2000 using the following formula
# Days = CurrentDay + (30*(CurrentMonth-1)) + (365*(CurrentYear-2000))
.text

_start:
    # Setup data
    li      sp,    0x10000         # Initial Stack pointer
    li      x18,    21              # 21th
    li      x19,    1               # January
    li      x20,    2025            # 2025
    li      x21,    30              # 30
    li      x22,    365             # 365

    # Calculate subtractions
    addi    x19,    x19,     -1     # CurrentMonth - 1
    addi    x20,    x20,     -2000  # CurrentYear - 2000

    # First multiplication
    addi    sp,    sp,    -8        # Reserve space for two operands
    sw      x21, 4(sp)              # First operand
    sw      x19, 0(sp)              # Second operand
    jal     ra,    multiply
    lw      x23, 0(sp)              # Result -> 23
    addi    sp,    sp,    4         # Free space

    # Second multiplication
    addi    sp,    sp,    -8        # Reserve space for two operands
    sw      x22, 4(sp)              # First operand
    sw      x20, 0(sp)              # Second operand
    jal     ra,    multiply
    lw      x24, 0(sp)              # Result -> 23
    addi    sp,    sp,    4         # Free space
    
    # Add
    add     x24,    x24,    x23     # (30*(CurrentMonth-1)) + (365*(CurrentYear-2000))
    add     x10,    x24,    x18     # CurrentDay + (30*(CurrentMonth-1)) + (365*(CurrentYear-2000))

    ebreak

multiply:
    # Setup
    lw      x5,     0(sp)           # 1st op -> 5
    lw      x6,     4(sp)           # 1st op -> 5
    addi    sp,     sp,    8        # Free space
    addi    x7,     zero,  0       # Zero product register
    # Zero multiplier
    beq     x6,     zero,  exit    # Branch to the end
    # Loop
loop:
    add     x7,     x7,     x5      # Add multiplicand to product
    addi    x6,     x6,     -1      # Decrement multiplier
    bne     x6,     zero,  loop    # Loop
exit:
    addi    sp,    sp,    -4        # Reserve space for result
    sw      x7,     0(sp)           # First result
    
    ret
