.text

_start:
sw x0, 0(x4) # will put dmemWEN in 3rd clock cycle with x4 address
ori x2, x0, 5
ori x3, x0, 10
lw x1, 0(x4) # will try to access x4 address in this clock cycle 
# CONFLICT
ebreak
