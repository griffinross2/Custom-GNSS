.text

_start:
# Unit tests for branches
# Let x10 be used as a trace register
    li      x10,    0

#=================================#
# Branch If Greater Than Unsigned #
#=================================#

# Branch greater than unsigned (false)
    li      x11,    10
    li      x12,    -10
    bgeu    x11,    x12,    bgeu_false_fail
    li      x10,    2
    sw      x10,    0x100(zero)       # Trace 2
    ebreak
bgeu_false_fail:
    li      x10,    1
    sw      x10,    0x100(zero)       # Trace 1
    ebreak                    # BGEU false failed
