.text

_start:
# Unit tests for branches
# Let x10 be used as a trace register
    li      x10,    0

#=================================#
# Branch If Greater Than Unsigned #
#=================================#

# Branch greater than unsigned (true)
    li      x11,    -10
    li      x12,    181
    bgeu    x11,    x12,    bgeu_true_ok
    li      x10,    1
    sw      x10,    0x100(zero)       # Trace 1
    ebreak                    # BGEU true failed
bgeu_true_ok:
    li      x10,    2
    sw      x10,    0x100(zero)       # Trace 2
    ebreak
