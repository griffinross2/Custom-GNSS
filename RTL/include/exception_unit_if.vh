/*
  Exception Unit interface
*/
`ifndef EXCEPTION_UNIT_IF_VH
`define EXCEPTION_UNIT_IF_VH

`include "common_types.vh"
import common_types_pkg::*;

interface exception_unit_if;

  // From decode
  word_t e2mif_pc;
  logic illegal_inst;

  // To CSR/datapath
  logic exception;
  logic is_interrupt; // 0: exception, 1: interrupt
  word_t exception_pc;
  word_t exception_cause;
  word_t exception_target;

  // Flush signals
  logic f2dif_flush;
  logic d2eif_flush;
  logic e2mif_flush;
  logic m2wif_flush;

  // From CSR
  logic interrupt_en;
  logic [1:0] mtvec_mode;
  word_t mtvec_base;
  word_t mie; // Interrupt enable register

  logic [31:0] interrupt_in_sync;

  // exception ports
  modport exception_unit (
    input   illegal_inst, e2mif_pc,
            interrupt_en, mtvec_mode, mtvec_base, mie,
            interrupt_in_sync,
    output  exception, exception_pc, exception_cause, exception_target, is_interrupt,
            f2dif_flush, d2eif_flush, e2mif_flush, m2wif_flush
  );
  // exception tb
  modport tb (
    input   exception, exception_pc, exception_cause, exception_target, is_interrupt,
            f2dif_flush, d2eif_flush, e2mif_flush, m2wif_flush,
    output  illegal_inst, e2mif_pc,
            interrupt_en, mtvec_mode, mtvec_base, mie,
            interrupt_in_sync
  );
endinterface

`endif //EXCEPTION_IF_VH
