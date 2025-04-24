/*
  Control Unit interface
*/
`ifndef CONTROL_UNIT_IF_VH
`define CONTROL_UNIT_IF_VH

`include "common_types.vh"
import common_types_pkg::*;

interface control_unit_if;

  // Control Unit input
  word_t    inst;
  
  // ALU
  reg_t rs1, rs2, rd;
  alu_op_t   alu_op;
  // Multiplier
  logic     mult;
  logic     mult_half;      // 0 - low half, 1 - high half
  logic     mult_signed_a, mult_signed_b;
  // Divider
  logic     div;
  logic     div_rem;        // 0 - quotient, 1 - remainder
  logic     div_signed;
  // Memory control
  logic     dread;
  logic [1:0] dwrite;
  // Immediate generator out
  word_t    immediate;
  // ALU source 1 (RS2 or PC)
  logic     alu_src1;       // 0 - rs1, 1 - PC
  // ALU source 2 (RS1 or immediate)
  logic     alu_src2;       // 0 - rs2, 1 - immediate
  // Register write source (ALU output or memory)
  logic [1:0] reg_wr_src;   // 0 - alu, 1 - memory, 2 - pc + 4
  logic [1:0] reg_wr_mem;   // 0 - byte, 1 - halfword, 2 - word
  logic reg_wr_mem_signed;  // 0 - unsigned, 1 - signed
  // Branch/jump control
  logic     branch_pol;     // 0 - branch if alu.zero = 1, branch if alu.zero - 0
  logic [2:0] pc_ctrl;      // 0 - PC increment, 1 - PC branch, 2 - JAL, 3 - JALR, 4 - exception return
  // Exception control
  logic halt;
  logic illegal_inst;
  // CSR
  logic csr_write;
  logic [11:0] csr_waddr;
  logic [1:0] csr_wr_op;    // 0 - move, 1 - set, 2 - clear
  logic csr_wr_imm;         // 0 - rs1 value, 1 - immediate in rs1

  // control ports
  modport control_unit (
    input   inst,
    output  rs1, rs2, rd,                                   // Register File
            alu_op,                                         // ALU
            mult, mult_half, mult_signed_a, mult_signed_b,  // Multiplier
            div, div_rem, div_signed,                       // Divider
            dread, dwrite,                                  // Request Unit
            immediate,                                      // Immediate Generator
            alu_src1, alu_src2,                             // ALU source mux
            reg_wr_src, reg_wr_mem, reg_wr_mem_signed,      // Reg File writeback source
            branch_pol, pc_ctrl,                            // Program Counter
            halt, illegal_inst,                             // Exception Control
            csr_write, csr_waddr, csr_wr_op, csr_wr_imm     // CSR
  );
  // control tb
  modport tb (
    input   rs1, rs2, rd,                                   // Register File
            alu_op,                                         // ALU
            mult, mult_half, mult_signed_a, mult_signed_b,  // Multiplier
            div, div_rem, div_signed,                       // Divider
            dread, dwrite,                                  // Request Unit
            immediate,                                      // Immediate Generator
            alu_src1, alu_src2,                             // ALU source mux
            reg_wr_src, reg_wr_mem, reg_wr_mem_signed,      // Reg File writeback source
            branch_pol, pc_ctrl,                            // Program Counter
            halt, illegal_inst,                             // Exception Control
            csr_write, csr_waddr, csr_wr_op, csr_wr_imm,    // CSR
    output  inst
  );
endinterface

`endif //CONTROL_IF_VH
