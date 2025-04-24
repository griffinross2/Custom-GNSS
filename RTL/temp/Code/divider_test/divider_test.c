#include <stdint.h>

#include "../riscv.h"

#define UART_BASE 0x00020000

#define UART_CFGR (UART_BASE + 0x00)
#define UART_TXDR (UART_BASE + 0x04)
#define UART_RXDR (UART_BASE + 0x08)
#define UART_SR (UART_BASE + 0x0C)

#define SET_CSR(csr, val) \
    asm volatile("csrs " #csr ", %0" ::"r"(val))

typedef struct uart
{
    volatile uint32_t CFGR; // Configuration register
    volatile uint32_t TXDR; // Transmit data register
    volatile uint32_t RXDR; // Receive data register
    volatile uint32_t SR;   // Status register
} UART_TypeDef;

#define UART ((UART_TypeDef *)UART_BASE)

void Unknown_Interrupt_Handler(void) __attribute__((interrupt));
void Exception_Handler(void) __attribute__((interrupt));
void UART_RXI_Handler(void) __attribute__((interrupt));

void uart_init(void);
void uart_send(char chr);
void uart_send_str(char *str);
void uart_receive(char *chr);
uint32_t lfsr32_next(int *lfsr);

void main(void)
{
    // Initialize UART
    uart_init();

    uart_send_str("Starting divider test...\n");

    int lfsr = 0xACE1u;
    int a;
    int b;
    int result;
    int rem;

    // Infinite loop
    while (1)
    {
        a = lfsr32_next(&lfsr);
        b = lfsr32_next(&lfsr);
        result = a / b;
        rem = a % b;

        // Send the test case to UART
        for (int i = 0; i < 32; i++)
        {
            uart_send(((a << i) & 0x80000000) ? '1' : '0');
        }
        uart_send_str(" / ");
        for (int i = 0; i < 32; i++)
        {
            uart_send(((b << i) & 0x80000000) ? '1' : '0');
        }
        uart_send_str(" = ");
        for (int i = 0; i < 32; i++)
        {
            uart_send(((result << i) & 0x80000000) ? '1' : '0');
        }
        uart_send_str(" R ");
        for (int i = 0; i < 32; i++)
        {
            uart_send(((rem << i) & 0x80000000) ? '1' : '0');
        }
        uart_send('\n');
    }
}

// From GPT
uint32_t lfsr32_next(int *lfsr)
{
    uint32_t lsb = *lfsr & 1; // Get LSB (output bit)
    *lfsr >>= 1;              // Shift register
    if (lsb)
    {
        *lfsr ^= 0x80000057u; // Apply feedback polynomial
    }
    return *lfsr;
}

void uart_init(void)
{
    // Initialize UART configuration register
    UART->CFGR = 434; // Set baud rate: 50M / 115200 = 434

    // Enable UART interrupt
    IRQ_ENABLE(UART_RXI); // Set interupt enable bit for UART RXI
}

void uart_send(char chr)
{
    // Wait for the UART to be ready
    while (UART->SR & 0x1) // Check if tx_busy (bit 0) is set
        ;

    // Send the character
    UART->TXDR = chr;

    // Wait for the UART to be busy
    while (!(UART->SR & 0x1)) // Check if tx_busy (bit 0) is set
        ;
}

void uart_send_str(char *str)
{
    while (*str != '\0')
    {
        uart_send(*str); // Send each character in the string
        str++;           // Move to the next character
    }
}

void uart_receive(char *chr)
{
    // Wait for the UART to receive data
    while (!(UART->SR & 0x8)) // Check if rx_done (bit 3) is set
        ;

    // Receive the character
    *chr = UART->RXDR;

    // Clear the rx_done flag
    UART->SR = 0x8; // Clear rx_done (bit 3)
}

void Unknown_Interrupt_Handler()
{
    uart_send_str("Unknown Interrupt!\n");
    while (1)
    {
    }
}

void Exception_Handler()
{
    uart_send_str("Exception!\n");
    while (1)
    {
    }
}

void UART_RXI_Handler()
{
    // Acknowledge the interrupt
    // Clear the rx_done flag
    UART->SR = 0x8; // Clear rx_done (bit 3)

    IRQ_ENTRY(); // Save interrupt context

    // Receive the character and echo it back
    char chr = UART->RXDR; // Read the received character
    uart_send(chr);

    IRQ_EXIT(); // Restore interrupt context
}