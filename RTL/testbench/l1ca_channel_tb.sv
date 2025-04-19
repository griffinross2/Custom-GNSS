`timescale 1ns/1ns

`include "common_gnss_types.vh"
import common_gnss_types_pkg::*;

module l1ca_channel_tb;

    logic clk, nrst;                    // Clock and reset
    logic en, clear;                    // Module enable and clear
    sv_t sv;                            // SV number for code generation
    logic delay_code, delay_cycles;     // Code delay control
    logic [31:0] code_rate, lo_rate;    // Code and local oscillator rates
    logic signal_in;                    // Signal input
    logic epoch;                        // Epoch signal
    acc_t ie, qe;                       // Early in-phase and quadrature phase output accumulator
    acc_t ip, qp;                       // Prompt in-phase and quadrature phase output accumulator
    acc_t il, ql;                       // Late in-phase and quadrature phase output accumulator
    logic [31:0] code_phase, lo_phase;  // Code and local oscillator phases

    l1ca_channel dut (
        .clk(clk),
        .nrst(nrst),
        .en(en),
        .clear(clear),
        .sv(sv),
        .delay_code(delay_code),
        .delay_cycles(delay_cycles),
        .code_rate(code_rate),
        .lo_rate(lo_rate),
        .signal_in(signal_in),
        .epoch(epoch),
        .ie(ie),
        .qe(qe),
        .ip(ip),
        .qp(qp),
        .il(il),
        .ql(ql),
        .code_phase(code_phase),
        .lo_phase(lo_phase)
    );

    localparam FS = 69.984e6;
    localparam FC = 9.334875e6;

    initial begin
        clk = 0;
        forever #(1s/real'(FS)) clk = ~clk; // Clock generation
    end

    initial begin
        nrst = 0;
        en = 0;
        clear = 0;
        sv = 5'd0;
        delay_code = 1'b0;
        delay_cycles = 1'b0;
        code_rate = int'((4294967295.0 * 1023000.0) / real'(FS)); // Code rate calculation
        lo_rate = int'((4294967295.0 * real'(FC)) / real'(FS)); // LO rate calculation
        signal_in = 1'b0;

        @(posedge clk);
        @(posedge clk);
        nrst = 1; // Release reset
        @(posedge clk);
        en = 1; // Enable the module

        #10ms;
        $finish;
    end

    initial begin
        forever begin
            @(posedge clk);
            signal_in = $urandom_range(0, 1); // Random signal input
        end
    end

endmodule