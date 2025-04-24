.text

_start:
ori x1, x0, 8
ori x2, x0, 8
ori x3, x0, 8
ori x4, x0, 8
ori x5, x0, 8
ori x6, x0, 8
ori x7, x0, 8
ori x8, x0, 8
ori x9, x0, 8
ori x10, x0, 8
ori x11, x0, 8


# RAW on x1
add x1,x2,x3
sub x4,x1,x3
and x6,x1,x7
or x8,x1,x9

# WAR on x9
add x9, x1, x2

# WAW on x9 - add may write to x9 before load puts the data on x9
lw x9, 0(x1) 
add x9, x1, x2
sw x9, 100(x1)

# Load use
lw x10, 0(x1)
lw x11, 4(x1)
mul x10, x10, x11
sw x10, 100(x1)

ebreak
