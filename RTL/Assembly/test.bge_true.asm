.text

_start:
# Unit tests for branches
# Let x10 be used as a trace register
    li      x10,    0

#========================#
# Branch If Greater Than #
#========================#

# Branch greater than (true)
    li      x11,    231
    li      x12,    139
    bge     x11,    x12,    bge_true_ok
    li      x10,    1
    sw      x10,    0x100(zero)       # Trace 1
    ebreak                    # BGE true failed
bge_true_ok:
    li      x10,    2
    sw      x10,    0x100(zero)       # Trace 2
    ebreak
