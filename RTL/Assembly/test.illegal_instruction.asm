# Test the illegal instruction exception handler

.text
_start:
    # Do whatever
    li x2, 1234
    li x3, 5678
    add x4, x2, x3
    sw x4, 0x100(x0)

    # Illegal instruction
    .word 0xFFFFFFFF

    # Do whatever
    li x5, 4321
    li x6, 8765
    add x7, x5, x6
    sw x7, 0x10C(x0)

    ebreak

# Handler
.org 0x8000
_exception_handler:
    li x10, 0xABC
    sw x10, 0x104(x0)
    
    # Get the cause of the exception
    csrr x1, mcause
    sw x1, 0x108(x0)

    # Return
    mret
