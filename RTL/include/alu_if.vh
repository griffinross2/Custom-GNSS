`ifndef ALU_IF_VH
`define ALU_IF_VH

`include "common_types.vh"
import common_types_pkg::*;

interface alu_if;
    word_t a, b;
    logic [3:0] op;
    word_t out;
    logic zero;

    modport alu (
        input a, b, op,
        output out, zero
    );
endinterface

`endif // ALU_IF_VH