`ifndef MULTIPLIER_IF
`define MULTIPLIER_IF

`include "common_types.vh"
import common_types_pkg::*;

interface multiplier_if;
    
    logic en, ready;
    logic is_signed_a, is_signed_b;
    logic [31:0] a, b;
    logic [63:0] out;

    modport mult (
        input en, is_signed_a, is_signed_b, a, b,
        output out, ready
    );
endinterface

`endif // MULTIPLIER_IF