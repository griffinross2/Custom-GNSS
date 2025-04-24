/*******************************************/
/* CSR (Command & Status Registers) module */
/*******************************************/
`timescale 1ns/1ns

`include "csr_if.vh"

`include "common_types.vh"
import common_types_pkg::*;

module csr (
    input logic clk, nrst,
    csr_if.csr csr_if
);

// CSR registers
csr_mstatus_t mstatus;
word_t mstatush;
csr_mtvec_t mtvec;
word_t mip;
word_t mie;
word_t mepc;
word_t mcause;
word_t mscratch;

csr_mstatus_t mstatus_n;
word_t mstatush_n;
csr_mtvec_t mtvec_n;
word_t mip_n;
word_t mie_n;
word_t mepc_n;
word_t mcause_n;
word_t mscratch_n;

// FF
always_ff @(posedge clk) begin
    if (~nrst) begin
        mstatus <= 0;
        mstatush <= 0;
        mtvec <= {30'h2000, 2'h0};  // Start with base = 0x8000, mode = Direct
        mip <= 0;
        mie <= 0;
        mepc <= 0;
        mcause <= 0;
        mscratch <= 0;
    end else begin
        mstatus <= mstatus_n;
        mstatush <= mstatush_n;
        mtvec <= mtvec_n;
        mip <= mip_n;
        mie <= mie_n;
        mepc <= mepc_n;
        mcause <= mcause_n;
        mscratch <= mscratch_n;
    end
end

always_comb begin
    mstatus_n = mstatus;
    mstatush_n = mstatush;
    mtvec_n = mtvec;
    mip_n = mip;
    mie_n = mie;
    mepc_n = mepc;
    mcause_n = mcause;
    mscratch_n = mscratch;

    // CSR read
    case (csr_if.csr_raddr)
        MSTATUS: csr_if.csr_rdata = mstatus;
        MSTATUSH: csr_if.csr_rdata = mstatush;
        MTVEC: csr_if.csr_rdata = mtvec;
        MIP: csr_if.csr_rdata = mip;
        MIE: csr_if.csr_rdata = mie;
        MEPC: csr_if.csr_rdata = mepc;
        MCAUSE: csr_if.csr_rdata = mcause;
        MSCRATCH: csr_if.csr_rdata = mscratch;
        default: csr_if.csr_rdata = 0;
    endcase

    // CSR write
    if (csr_if.csr_write) begin
        case (csr_if.csr_waddr)
            MSTATUS: mstatus_n = csr_if.csr_wdata;
            MSTATUSH: mstatush_n = csr_if.csr_wdata;
            MTVEC: mtvec_n = csr_if.csr_wdata;
            MIP: mip_n = csr_if.csr_wdata;
            MIE: mie_n = csr_if.csr_wdata;
            MEPC: mepc_n = csr_if.csr_wdata;
            MCAUSE: mcause_n = csr_if.csr_wdata;
            MSCRATCH: mscratch_n = csr_if.csr_wdata;
            default: mcause_n = mcause;
        endcase
    end

    // Hardware overrides anything here
    if (csr_if.csr_exception) begin
        // Disable interrupts
        mstatus_n.mpie = mstatus.mie;   // Save current MIE in MPIE
        mstatus_n.mie = 0;              // Disable interrupts

        // Set exception cause and PC
        mcause_n = csr_if.csr_exception_cause;
        mepc_n = csr_if.csr_exception_pc;
    end

    if (csr_if.csr_mret) begin
        // Restore MIE from MPIE
        mstatus_n.mie = mstatus.mpie;   // Restore MIE from MPIE
        mstatus_n.mpie = 1;             // Set MPIE
    end
end

// Output
always_comb begin
    csr_if.csr_interrupt_en = mstatus.mie;
    csr_if.csr_mie = mie;
    csr_if.csr_mtvec_mode = mtvec.mode;
    csr_if.csr_mtvec_base = mtvec.base;
    csr_if.csr_mepc = mepc;
end

endmodule