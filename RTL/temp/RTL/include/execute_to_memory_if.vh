/*
  Control Unit interface
*/
`ifndef EXECUTE_TO_MEMORY_IF_VH
`define EXECUTE_TO_MEMORY_IF_VH

`include "common_types.vh"
import common_types_pkg::*;

interface execute_to_memory_if;

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
  // Memory control
  logic     dread;
  logic [1:0] dwrite;
  logic     dread_short;
  logic [1:0] dwrite_short;
  // Register write source (ALU output or memory)
  logic [1:0] reg_wr_src;   // 0 - alu, 1 - memory, 2 - pc + 4
  logic [1:0] reg_wr_mem;   // 0 - byte, 1 - halfword, 2 - word
  logic reg_wr_mem_signed;  // 0 - unsigned, 1 - signed
  // Branch/jump control
  logic     branch_pol;     // 0 - branch if alu.zero = 1, branch if alu.zero - 0
  logic [2:0] pc_ctrl;      // 0 - PC increment, 1 - PC branch, 2 - JAL, 3 - JALR, 4 - exception return
  // CSR
  logic csr_write;
  logic [11:0] csr_waddr;
  logic [1:0] csr_wr_op;    // 0 - move, 1 - set, 2 - clear
  logic csr_wr_imm;         // 0 - rs1 value, 1 - immediate in rs1
  // Exception
  logic illegal_inst;

  /*****************/
  /* Register File */
  /*****************/
  word_t rdat1;
  word_t rdat2;

  /*******/
  /* ALU */
  /*******/
  word_t alu_out;
  logic alu_zero;

  /*****************/
  /* PC Arithmetic */
  /*****************/
  word_t pc_plus_imm;

  /***************/
  /* Branch Unit */
  /***************/
  logic branch_predict;
  word_t branch_target;
endinterface

`endif // EXECUTE_TO_MEMORY_IF_VH
