`ifndef BRANCH_UNIT_IF_VH
`define BRANCH_UNIT_IF_VH

`include "common_types.vh"
import common_types_pkg::*;

interface branch_unit_if;
    logic mem_branch;       // Branch signal from MEM stage (if inst is a branch)
    logic mem_taken;        // Taken signal from MEM stage (what branch resolved to)
    word_t mem_pc;          // PC from MEM stage
    word_t mem_target_res;  // Target resolved from MEM stage
    word_t mem_target;      // Target from MEM stage (what the BU predicted)
    logic mem_predict;      // Prediction from MEM stage (what the BU predicted)
    logic mem_flush;        // Flush and jump if incorrect, else continue
    logic mem_branch_miss;  // Branch miss
    word_t fetch_pc;        // PC from fetch stage
    word_t fetch_target;    // Address to fetch
    logic fetch_predict;    // Prediction for fetch stage

    modport branch_unit (
        input mem_branch, mem_taken, mem_pc, mem_target, mem_target_res, mem_predict, fetch_pc,
        output mem_flush, mem_branch_miss, fetch_target, fetch_predict
    );
endinterface

`endif // BRANCH_UNIT_IF_VH
