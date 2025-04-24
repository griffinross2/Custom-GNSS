`timescale 1ns/1ns

`include "common_types.vh"
import common_types_pkg::*;
`include "ram_dump_if.vh"

module system_fpga (
  input logic CLK,
  input logic ck_rst,
  input logic uart_txd_in,
  output logic UART_TXD,
  output logic LED [0:3]
);

  // Clock and reset
  (* keep = "true", dont_touch = "true", mark_debug = "true" *) reg cpuclk;
  logic nrst;

  // Halt signal
  logic halt;

  // RAM dump IF
  ram_dump_if cpu_ram_if();

  // UART
  logic rxd;
  logic txd;

  initial begin
    cpuclk = 1'b0;
  end

  always_ff @(posedge CLK) begin
    cpuclk <= ~cpuclk;
  end

  assign nrst = ck_rst;
  assign UART_TXD = txd;
  assign rxd = uart_txd_in;
  assign LED[0] = ~txd;
  assign LED[1] = ~rxd;
  assign LED[2] = halt;
  assign LED[3] = cpuclk & nrst;

  assign cpu_ram_if.override_ctrl = 0;

  // System
  system system_inst (
    .clk(cpuclk),
    .nrst(nrst),
    .rxd(rxd),
    .txd(txd),
    .halt(halt),
    .cpu_ram_debug_if(cpu_ram_if)
  );

endmodule