`timescale 1ns/1ns

`include "ahb_bus_if.vh"

`include "common_types.vh"
import common_types_pkg::*;

module cpu (
  input logic clk, nrst,
  input logic [31:0] interrupt_in_sync,
  output logic halt,
  ahb_bus_if.controller_to_mux abif
);

// Datapath
datapath datapath_inst (
  .clk(clk),
  .nrst(nrst),
  .interrupt_in_sync(interrupt_in_sync),
  .halt(halt),
  .abif(abif)
);
  
endmodule
