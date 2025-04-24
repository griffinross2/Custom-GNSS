/*
  Control Unit interface
*/
`ifndef FETCH_TO_DECODE_IF_VH
`define FETCH_TO_DECODE_IF_VH

`include "common_types.vh"
import common_types_pkg::*;

interface fetch_to_decode_if;

  // Enable/Flush
  logic en;
  logic flush;

  // Latched stuff

  /*******************/
  /* Program Counter */
  /*******************/
  word_t pc;

  /***************/
  /* Instruction */
  /***************/
  logic inst_latch; // set to 1 initially, 0 after hready

  /***************/
  /* Branch Unit */
  /***************/
  logic branch_predict;
  word_t branch_target;
endinterface

`endif // FETCH_TO_DECODE_IF_VH
