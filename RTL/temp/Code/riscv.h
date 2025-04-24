#ifndef RISCV_H
#define RISCV_H

// Utility macros for RISCV
#define SET_CSR(csr, val) \
    asm volatile("csrs " #csr ", %0" ::"r"(val))

#define READ_CSR(csr)                 \
    ({                                \
        unsigned long __tmp;          \
        asm volatile("csrr %0, " #csr \
                     : "=r"(__tmp));  \
        __tmp;                        \
    })

#define CLEAR_CSR(csr, val) \
    asm volatile("csrc " #csr ", %0" ::"r"(val))

// IRQ Entry and Exit Macros

// Save interrupt context and re-enable interrupts
#define IRQ_ENTRY()                  \
    int mcause = READ_CSR(mcause);   \
    int mstatus = READ_CSR(mstatus); \
    int mepc = READ_CSR(mepc);       \
    SET_CSR(mstatus, 0x8);

// Disable interrupts and restore interrupt context
#define IRQ_EXIT()           \
    CLEAR_CSR(mstatus, 0x8); \
    SET_CSR(mepc, mepc);     \
    SET_CSR(mstatus, mstatus);

#define IRQ_ENABLE(irq) \
    SET_CSR(mie, (1 << irq)); // Enable the specified interrupt

#define IRQ_DISABLE(irq) \
    CLEAR_CSR(mie, (1 << irq)); // Disable the specified interrupt

#define ENTER_CRITICAL() \
    asm volatile("csrc mstatus, 0x8") // Disable interrupts

#define EXIT_CRITICAL() \
    asm volatile("csrs mstatus, 0x8") // Enable interrupts

// Interrupt assignments
#define UART_RXI 16 // UART RX interrupt

#endif // RISCV_H