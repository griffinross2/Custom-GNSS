.text

_start:
# Unit tests for branches
# Let x10 be used as a trace register
    li      x10,    0

#=================#
# Branch If Equal #
#=================#

# Branch if equal (false)
    li      x11,    1
    li      x12,    2
    beq     x11,    x12,    beq_false_fail
    li      x10,    2
    sw      x10,    0x100(zero)       # Trace 2
    ebreak
beq_false_fail:
    li      x10,    1
    sw      x10,    0x100(zero)       # Trace 1
    ebreak                    # BEQ false failed
