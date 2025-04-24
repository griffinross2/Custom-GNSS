/*
  Forward Unit
*/
`ifndef FORWARD_UNIT_VH
`define FORWARD_UNIT_VH

`include "common_types.vh"
import common_types_pkg::*;

interface forward_unit_if;

  logic [1:0] forward_a_to_ex;        // 0 - rdat1, 1 - mem_alu_out, 2 - wb_alu_out  
  logic [1:0] forward_b_to_ex;        // 0 - rdat2, 2 - mem_alu_out, 2 - wb_alu_out

  reg_t ex_rsel1;
  reg_t ex_rsel2;

  reg_t mem_rd;
  reg_t wb_rd;

  modport forward_unit (
    input ex_rsel1, ex_rsel2, mem_rd, wb_rd,
    output forward_a_to_ex, forward_b_to_ex
  );

  modport tb (
    output ex_rsel1, ex_rsel2, mem_rd, wb_rd,
    input forward_a_to_ex, forward_b_to_ex
  );


endinterface

`endif
