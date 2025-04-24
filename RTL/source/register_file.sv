/************************/
/* Register File module */
/************************/
`timescale 1ns/1ns

`include "register_file_if.vh"

`include "common_types.vh"
import common_types_pkg::*;

module register_file (
    input logic clk, nrst,
    register_file_if.register_file rf_if
);
    // Registers definition
    word_t registers [1:31];
    word_t registers_n [1:31];

    // Registers
    always_ff @(posedge clk) begin
        if(~nrst) begin
            registers <= '{default: 0};
        end else begin
            registers <= registers_n;
        end
    end

    // Next state logic
    always_comb begin
        registers_n = registers;
        if(rf_if.wen && rf_if.wsel != 0) begin
            registers_n[rf_if.wsel] = rf_if.wdat;
        end
    end

    // Read logic
    assign rf_if.rdat1 = (rf_if.rsel1 == 0) ? 0 : registers[rf_if.rsel1];
    assign rf_if.rdat2 = (rf_if.rsel2 == 0) ? 0 : registers[rf_if.rsel2];
    
endmodule