.text

_start:
# Unit tests for branches
# Let x10 be used as a trace register
    li      x10,    0

#==============================#
# Branch If Less Than Unsigned #
#==============================#

# Branch less than unsigned (true)
    li      x11,    231
    li      x12,    -123
    bltu    x11,    x12,    bltu_true_ok
    li      x10,    1
    sw      x10,    0x100(zero)       # Trace 1
    ebreak                    # BLTU true failed
bltu_true_ok:
    li      x10,    2
    sw      x10,    0x100(zero)       # Trace 2
    ebreak
