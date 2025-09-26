/*
  GNSS Channel to Satellite Interface
*/
`ifndef GNSS_CHANNEL_SATELLITE_IF_VH
`define GNSS_CHANNEL_SATELLITE_IF_VH

`include "common_types.vh"
import common_types_pkg::*;

interface gnss_channel_satellite_if;

  logic clear;
  logic epoch;
  logic [31:0] code_rate;
  logic [31:0] lo_rate;
  logic [15:0] ie, qe, ip, qp, il, ql;

  modport satellite (
    input epoch, ie, qe, ip, qp, il, ql,
    output code_rate, lo_rate, clear
  );

  modport channel (
    output epoch, ie, qe, ip, qp, il, ql,
    input code_rate, lo_rate, clear
  );

endinterface

`endif // GNSS_CHANNEL_SATELLITE_IF_VH
