`ifndef DIVIDER_IF
`define DIVIDER_IF

`include "common_types.vh"
import common_types_pkg::*;

interface divider_if;
    
    logic en, ready;
    logic is_signed;
    logic [31:0] a, b;
    logic [31:0] q, r;
    logic div_by_zero;
    logic overflow;

    modport div (
        input en, is_signed, a, b,
        output q, r, ready, div_by_zero, overflow
    );
endinterface

`endif // DIVIDER_IF