`timescale 1ns/1ns

`include "common_gnss_types.vh"
import common_gnss_types_pkg::*;

module l1ca_code_tb;

    logic clk, nrst;
    logic en, clear;
    integer sv;
    logic code, epoch;
    chip_t chip;

    l1ca_code dut (
        .clk(clk),
        .nrst(nrst),
        .en(en),
        .clear(clear),
        .sv(sv[4:0]),
        .code(code),
        .epoch(epoch),
        .chip(chip)
    );

    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    initial begin
        nrst = 0;
        en = 0;
        clear = 0;
        sv = 5'd0;

        #10 nrst = 1;

        for (sv = 0; sv < 32; sv++) begin
            en = 1;
            wait(epoch == 1'b0);
            wait(epoch == 1'b1);
        end

        // test clear
        sv = 0;
        wait(chip == 10'd500);
        @(negedge clk);
        clear = 1;
        @(negedge clk);
    
        if (epoch == 1'b0) begin
            $display("Clear code generator failed.");
        end

        #10 $finish; // End simulation
    end

endmodule