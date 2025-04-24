/*
  Command and Status Registers interface
*/
`ifndef CSR_IF_VH
`define CSR_IF_VH

`include "common_types.vh"
import common_types_pkg::*;

interface csr_if;

  // CSR r/w signals
  logic csr_write;
  logic [11:0] csr_waddr;
  logic [11:0] csr_raddr;
  logic [WORD_W-1:0] csr_wdata;
  logic [WORD_W-1:0] csr_rdata;

  // CSR hw override signals
  logic csr_exception;
  logic csr_mret;
  word_t csr_exception_cause;
  word_t csr_exception_pc;

  // CSR output control signals
  logic csr_interrupt_en;
  word_t csr_mie; // Interrupt enable register
  logic [1:0] csr_mtvec_mode;
  logic [29:0] csr_mtvec_base;
  word_t csr_mepc; // Exception program counter

  // CSR port
  modport csr (
    input   csr_write, csr_waddr, csr_wdata,
            csr_raddr,
            csr_exception, csr_mret, csr_exception_cause, csr_exception_pc,
    output  csr_rdata,
            csr_interrupt_en,
            csr_mie,
            csr_mtvec_mode, csr_mtvec_base,
            csr_mepc

  );
  // tb port
  modport tb (
    input   csr_rdata,
            csr_interrupt_en,
            csr_mie,
            csr_mtvec_mode, csr_mtvec_base,
            csr_mepc,
    output  csr_write, csr_waddr, csr_wdata,
            csr_raddr,
            csr_exception, csr_mret, csr_exception_cause, csr_exception_pc
  );
endinterface

`endif //CSR_IF_VH
