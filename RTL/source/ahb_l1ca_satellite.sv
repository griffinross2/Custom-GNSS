`timescale 1ns/1ns

`include "common_types.vh"
import common_types_pkg::*;
`include "common_gnss_types.vh"
import common_gnss_types_pkg::*;

`include "ahb_bus_if.vh"

module ahb_l1ca_satellite #(
    parameter NUM_CHANNELS = 1
) (
    input logic clk, hclk, nrst,
    ahb_bus_if.satellite_to_mux abif,
    input epoch [NUM_CHANNELS-1:0],                 // Epoch signal from each channel
    input word_t code_phase [NUM_CHANNELS-1:0],     // Code phase from each channel
    input word_t lo_phase [NUM_CHANNELS-1:0],       // Local oscillator phase from each channel
    input gps_chip_t chip [NUM_CHANNELS-1:0],       // Chip index from each channel
    input acc_t ie [NUM_CHANNELS-1:0],              // Early in-phase accumulator from each channel
    input acc_t qe [NUM_CHANNELS-1:0],              // Early quadrature phase accumulator from each channel
    input acc_t ip [NUM_CHANNELS-1:0],              // Prompt in-phase accumulator from each channel
    input acc_t qp [NUM_CHANNELS-1:0],              // Prompt quadrature phase accumulator from each channel
    input acc_t il [NUM_CHANNELS-1:0],              // Late in-phase accumulator from each channel
    input acc_t ql [NUM_CHANNELS-1:0],              // Late quadrature phase accumulator from each channel
    output logic code_rate [NUM_CHANNELS-1:0],      // Code rate for each channel
    output logic lo_rate [NUM_CHANNELS-1:0]         // Local oscillator rate for each channel
);



endmodule