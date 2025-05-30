`timescale 1ns/1ns

`include "common_types.vh"
import common_types_pkg::*;
`include "axi_controller_if.vh"

module system_fpga (
    input CLK, ck_rst,
    output qspi_cs,
    inout [3:0] qspi_dq,
    output LED [0:3],
    output UART_TXD,
    input uart_txd_in,
    input jb_0, jb_1, jb_2, jb_3, jb_6,
    output jb_4, jb_5, jb_7
);

  // Clock and reset
  (* keep = "true", dont_touch = "true", mark_debug = "true" *) reg cpuclk;

  // Clock division
  initial begin
    cpuclk = 1'b0;
  end

  always_ff @(posedge CLK) begin
    cpuclk <= ~cpuclk;
  end
  // assign cpuclk = CLK;

  logic [1:0] inp_i;
  logic [1:0] inp_q;

  assign {jb_5, jb_4, jb_7} = 3'b100;

  assign inp_i = {jb_3, jb_2};
  assign inp_q = {jb_1, jb_0};

  logic nrst;
  assign nrst = ck_rst;

  // Halt signal
  logic halt;

  // UART
  logic rxd;
  logic txd;

  axi_controller_if debug_amif();

  assign UART_TXD = txd;
  assign rxd = uart_txd_in;
  assign LED[0] = ~txd;
  assign LED[1] = ~rxd;
  assign LED[2] = halt;
  assign LED[3] = cpuclk & nrst;

  // Zero out dump interface
  always_comb begin
    debug_amif.read = '0;
    debug_amif.write = '0;
    debug_amif.addr = '0;
    debug_amif.store = '0;
    debug_amif.done = '0;
  end

  // System
  system system_inst (
    .clk(cpuclk),
    .nrst(nrst),
    .gnssclk(jb_6),
    .signal_in(inp_i[1]),
    .rxd(rxd),
    .txd(txd),
    .halt(halt),
    .debug_amif(debug_amif),
    .flash_cs(qspi_cs),
    .flash_dq(qspi_dq)
  );

endmodule