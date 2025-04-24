/*
  Hazard Unit interface
*/
`ifndef HAZARD_UNIT_IF_VH
`define HAZARD_UNIT_IF_VH

`include "common_types.vh"
import common_types_pkg::*;

interface hazard_unit_if;

  logic halt;           // Halt signal from Control Unit
  logic dread, dwrite;  // Memory control signals from Control Unit
  logic dhit, ihit;     // Cache hit signals
  logic branch_flush;   // Branch flush requested by Branch Unit
  logic d2eif_dread;    // EXECUTE STAGE dread signal
  logic [REG_W-1:0] d2eif_rd;     // EXECUTE STAGE destination register
  logic [REG_W-1:0] f2dif_rs1;    // DECODE STAGE source register 1
  logic [REG_W-1:0] f2dif_rs2;    // DECODE STAGE source register 2
  logic d2eif_mult, mult_ready;   // EXECUTE STAGE multiplier signals
  logic d2eif_div, div_ready;     // EXECUTE STAGE divider signals

  logic f2dif_en;       // Enable fetch to decode
  logic d2eif_en;       // Enable decode to execute
  logic e2mif_en;       // Enable execute to memory
  logic m2wif_en;       // Enable memory to writeback

  logic f2dif_flush;    // Flush fetch to decode
  logic d2eif_flush;    // Flush decode to execute
  logic e2mif_flush;    // Flush execute to memory
  logic m2wif_flush;    // Flush memory to writeback

  logic ex_csr;         // CSR instruction in execute stage
  logic mem_csr;        // CSR instruction in memory stage
  logic wb_csr;         // CSR instruction in writeback stage

  // hazard ports
  modport hazard_unit (
    input   halt,             // Input from control unit
            dread, dwrite,    // Input from control unit
            branch_flush,     // Input from branch unit
            dhit, ihit,       // Input from caches
            d2eif_dread,      // EXECUTE STAGE dread signal
            d2eif_rd,         // EXECUTE STAGE destination register
            f2dif_rs1,        // DECODE STAGE source register 1
            f2dif_rs2,        // DECODE STAGE source register 2
            d2eif_mult,       // EXECUTE STAGE multiplier signals
            mult_ready,       // EXECUTE STAGE multiplier signals
            d2eif_div,        // EXECUTE STAGE divider signals
            div_ready,        // EXECUTE STAGE divider signals
            ex_csr,           // CSR instruction in execute stage
            mem_csr,          // CSR instruction in memory stage
            wb_csr,           // CSR instruction in writeback stage
    output  f2dif_en,         // Output to pipeline
            d2eif_en,         // Output to pipeline
            e2mif_en,         // Output to pipeline
            m2wif_en,         // Output to pipeline
            f2dif_flush,      // Output to pipeline
            d2eif_flush,      // Output to pipeline
            e2mif_flush,      // Output to pipeline
            m2wif_flush       // Output to pipeline
  );
  // control tb
  modport tb (
    input   f2dif_en,
            d2eif_en,
            e2mif_en,
            m2wif_en,
            f2dif_flush,
            d2eif_flush,
            e2mif_flush,
            m2wif_flush,
    output  halt,
            dread, dwrite,
            branch_flush,
            dhit, ihit,
            d2eif_dread,
            d2eif_rd,
            f2dif_rs1,
            f2dif_rs2,
            d2eif_mult,
            mult_ready,
            d2eif_div,
            div_ready,
            ex_csr,
            mem_csr,
            wb_csr
  );
endinterface

`endif // HAZARD_UNIT_IF_VH
