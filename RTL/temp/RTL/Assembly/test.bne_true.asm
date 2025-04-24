.text

_start:
# Unit tests for branches
# Let x10 be used as a trace register
    li      x10,    0

#=====================#
# Branch If Not Equal #
#=====================#

# Branch not equal (true)
    li      x11,    -10
    li      x12,    53
    bne     x11,    x12,    bne_true_ok
    li      x10,    1
    sw      x10,    0x100(zero)       # Trace 1
    ebreak                    # BNE true failed
bne_true_ok:
    li      x10,    2
    sw      x10,    0x100(zero)       # Trace 2
    ebreak
