.text

_start:
# Unit tests for ebreak
# Let x10 be used as a trace register
    li      x10,    0

# Nothing too crazy here
    li      x10,    2
    sw      x10,    0x100(zero)   # Trace 2 (pass)
    ebreak
    li      x10,    1
    sw      x10,    0x100(zero)   # Trace 1 (fail)
