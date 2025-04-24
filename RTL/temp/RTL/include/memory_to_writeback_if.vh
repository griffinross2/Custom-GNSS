/*
  Control Unit interface
*/
`ifndef MEMORY_TO_WRITEBACK_IF_VH
`define MEMORY_TO_WRITEBACK_IF_VH

`include "common_types.vh"
import common_types_pkg::*;

interface memory_to_writeback_if;

  // Enable/Flush
  logic en;
  logic flush;

  // Latched stuff

  /*******************/
  /* Program Counter */
  /*******************/
  word_t pc;

  /****************/
  /* Control Unit */
  /****************/
  logic halt;
  // Register file
  reg_t rd;
  reg_t rs1;
  word_t rdat1;
  // Register write source (ALU output or memory)
  logic [1:0] reg_wr_src;   // 0 - alu, 1 - memory, 2 - pc + 4
  logic [1:0] reg_wr_mem;   // 0 - byte, 1 - halfword, 2 - word
  logic reg_wr_mem_signed;  // 0 - unsigned, 1 - signed
  // CSR
  logic csr_write;
  logic [11:0] csr_waddr;
  logic [1:0] csr_wr_op;    // 0 - move, 1 - set, 2 - clear
  logic csr_wr_imm;         // 0 - rs1 value, 1 - immediate in rs1

  /*******/
  /* ALU */
  /*******/
  word_t alu_out;

  /**********/
  /* MEMORY */
  /**********/
  word_t dload;
endinterface

`endif // MEMORY_TO_WRITEBACK_IF_VH
