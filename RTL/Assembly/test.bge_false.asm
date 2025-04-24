.text

_start:
# Unit tests for branches
# Let x10 be used as a trace register
    li      x10,    0

#========================#
# Branch If Greater Than #
#========================#

# Branch greater than (false)
    li      x11,    231
    li      x12,    523
    bge     x11,    x12,    bge_false_fail
    li      x10,    2
    sw      x10,    0x100(zero)       # Trace 2
    ebreak
bge_false_fail:
    li      x10,    1
    sw      x10,    0x100(zero)       # Trace 1
    ebreak                    # BGE false failed
