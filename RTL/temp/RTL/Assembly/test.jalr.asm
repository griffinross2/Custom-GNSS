.text

_start:
# Unit tests for branches
# Let x10 be used as a trace register
    li      x10,    0

#========================#
# Jump and Link Register #
#========================#

    la      ra,    jalr_location
    jalr    ra,    16(ra)
    li      x10,    1
    sw      x10,    0x100(zero)   # Trace 1 (fail)
    sw      ra,    0x104(zero)   # Save the return address register
    ebreak
jalr_location:
    li      x10,    3
    sw      x10,    0x100(zero)   # Trace 3 (fail)
    sw      ra,    0x104(zero)   # Save the return address register
    ebreak
    li      x10,    2
    sw      x10,    0x100(zero)   # Trace 2 (pass)
    sw      ra,    0x104(zero)   # Save the return address register
    ebreak
