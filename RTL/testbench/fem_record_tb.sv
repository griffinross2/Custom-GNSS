`timescale 1ns/1ns

`include "common_types.vh"
import common_types_pkg::*;

module fem_record_tb (
);

    localparam CLK_PERIOD = 52.08ns; // 19.2 MHz clock period

    logic clk, nrst;

    initial begin
        clk = 0;
        forever #(CLK_PERIOD/2) clk = ~clk;
    end

    initial begin
        nrst = 0;
        #100 nrst = 1;
    end

    logic uart_tx;
    logic inp_i;

    fem_record_fpga dut (
        .CLK(clk),
        .ck_rst(nrst),
        .jb_0(1'b0),
        .jb_1(1'b0),
        .jb_2(1'b0),
        .jb_3(inp_i),
        .jb_6(clk),
        .UART_TXD(uart_tx)
    );

    integer i;
    initial begin
        inp_i = 1'b0;
        #100;

        forever begin
            i = $random();
            inp_i = i[0];
            @(posedge clk);
        end
    end

endmodule