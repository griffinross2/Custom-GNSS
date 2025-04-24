`timescale 1ns/1ns

`include "alu_if.vh"

`include "common_types.vh"
import common_types_pkg::*;

module alu_tb ();

    // ALU interface
    alu_if alu_if();

    // ALU instance
    alu alu_inst (
        .alu_if(alu_if)
    );

    // Testbench signals
    word_t expected_result;

    // Testbench process
    initial begin
        // Test case 1: Addition
        alu_if.a = 32'h00000005;
        alu_if.b = 32'h00000003;
        alu_if.op = ALU_ADD;
        expected_result = alu_if.a + alu_if.b;
        #10;
        assert(alu_if.out == expected_result) else $fatal("Test case 1 failed");

        // Test case 2: Subtraction
        alu_if.a = 32'h00000005;
        alu_if.b = 32'h00000003;
        alu_if.op = ALU_SUB;
        expected_result = alu_if.a - alu_if.b;
        #10;
        assert(alu_if.out == expected_result) else $fatal("Test case 2 failed");
    end

endmodule