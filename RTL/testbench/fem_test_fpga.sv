`timescale 1ns/1ns

`include "common_types.vh"
import common_types_pkg::*;

module fem_test_fpga (
    input logic CLK,
    input logic ck_rst,
    input logic jb_0, jb_1, jb_2, jb_3, jb_6,
    output logic jb_4, jb_5, jb_7,
    output logic LED [0:3]
);

    assign {jb_5, jb_4, jb_7} = 3'b100;
    assign LED[0:1] = {jb_2, jb_3};

    
    integer div;
    logic clk_1hz;
    
    always_ff @(posedge jb_6) begin
        if (~ck_rst) begin
            div = 0;
            clk_1hz = 0;
        end else begin
            if (div < 960000) begin
                div = div + 1;
            end else begin
                div = 0;
                clk_1hz = ~clk_1hz;
            end
        end
    end
    
    assign LED[3] = clk_1hz;

endmodule