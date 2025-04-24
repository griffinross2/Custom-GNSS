.text

_start:
# Unit tests for branches
# Let x10 be used as a trace register
    li      x10,    0

#=====================#
# Branch If Less Than #
#=====================#

# Branch less than (false)
    li      x11,    4
    li      x12,    -23
    blt     x11,    x12,    blt_false_fail
    li      x10,    2
    sw      x10,    0x100(zero)       # Trace 2
    ebreak
blt_false_fail:
    li      x10,    1
    sw      x10,    0x100(zero)       # Trace 1
    ebreak                    # BLT false failed
