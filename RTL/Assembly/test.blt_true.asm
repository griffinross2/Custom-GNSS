.text

_start:
# Unit tests for branches
# Let x10 be used as a trace register
    li      x10,    0

#=====================#
# Branch If Less Than #
#=====================#

# Branch less than (true)
    li      x11,    4
    li      x12,    132
    blt     x11,    x12,    blt_true_ok
    li      x10,    1
    sw      x10,    0x100(zero)       # Trace 1
    ebreak                    # BLT true failed
blt_true_ok:
    li      x10,    2
    sw      x10,    0x100(zero)       # Trace 2
    ebreak
