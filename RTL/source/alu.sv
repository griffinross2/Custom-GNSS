/**************************************/
/* ALU (Arithmetic Logic Unit) module */
/**************************************/
`timescale 1ns/1ns

`include "alu_if.vh"

`include "common_types.vh"
import common_types_pkg::*;

module alu (
    alu_if.alu alu_if
);
    // ALU result
    word_t alu_out;

    word_t add_out;
    cla_32_bit add_inst (
        .a(alu_if.a),
        .b(alu_if.b),
        .cin(1'b0),
        .sum(add_out)
    );

    word_t sub_out;
    cla_32_bit sub_inst (
        .a(alu_if.a),
        .b(~alu_if.b),
        .cin(1'b1),
        .sum(sub_out)
    );

    // ALU operation
    always_comb begin
        case (alu_if.op)
            ALU_ADD: alu_out = add_out;
            ALU_SUB: alu_out = sub_out;
            ALU_AND: alu_out = alu_if.a & alu_if.b;
            ALU_OR:  alu_out = alu_if.a | alu_if.b;
            ALU_XOR: alu_out = alu_if.a ^ alu_if.b;
            ALU_SLT: alu_out = $signed(alu_if.a) < $signed(alu_if.b) ? 1 : 0;
            ALU_SLTU: alu_out = $unsigned(alu_if.a) < $unsigned(alu_if.b) ? 1 : 0;
            ALU_SLL: alu_out = alu_if.a << alu_if.b[4:0];
            ALU_SRL: alu_out = alu_if.a >> alu_if.b[4:0];
            ALU_SRA: alu_out = $signed(alu_if.a) >>> alu_if.b[4:0];
            default: begin
                alu_out = add_out;
            end
        endcase
        // case (alu_if.op)
        //     ALU_ADD: alu_out = $signed(alu_if.a) + $signed(alu_if.b);
        //     ALU_SUB: alu_out = alu_if.a;
        //     ALU_AND: alu_out = alu_if.a;
        //     ALU_OR:  alu_out = alu_if.a;
        //     ALU_XOR: alu_out = alu_if.a;
        //     ALU_SLT: alu_out = alu_if.a;
        //     ALU_SLTU: alu_out = alu_if.a;
        //     ALU_SLL: alu_out = alu_if.a;
        //     ALU_SRL: alu_out = alu_if.a;
        //     ALU_SRA: alu_out = alu_if.a;
        //     default: alu_out = alu_if.a;
        // endcase
    end

    // Assign result to output
    assign alu_if.out = alu_out;
    assign alu_if.zero = ~|alu_out;

endmodule