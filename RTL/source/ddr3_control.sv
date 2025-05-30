`timescale 1ns/1ns

`include "ram_if.vh"
`include "common_types.vh"
import common_types_pkg::*;

module ddr3_control (
  input logic sys_clk_i, sys_rst,
  output logic clk, nrst,
  axi_bus_if.satellite_to_mux abif
);

  axi_controller_if.axi_controller amif;

endmodule