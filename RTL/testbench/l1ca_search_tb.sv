`timescale 1ns/1ns

`include "common_gnss_types.vh"
import common_gnss_types_pkg::*;

module l1ca_search_tb;
    logic clk, nrst;                    // Clock and reset
    logic signal_in;                    // Input signal
    logic start;                        // Start acquisition
    sv_t sv;                            // SV number to search
    word_t acc_out;                     // Maximum correlation
    logic [11:0] code_index;            // Chip index of maximum correlation
    logic [4:0] dop_index;              // Doppler index of maximum correlation
    logic busy;

    l1ca_search dut (
        .clk(clk),
        .nrst(nrst),
        .start(start),
        .signal_in(signal_in),
        .sv(sv[4:0]),
        .acc_out(acc_out),
        .code_index(code_index),
        .dop_index(dop_index),
        .busy(busy)
    );
    initial begin
        clk = 0;
        forever #26.0417 clk = ~clk;
    end

    integer fd;
    logic [2:0] bit_count;
    logic [7:0] signal_byte;
    initial begin
        nrst = 0;
        start = 0;
        signal_in = 0;
        sv = 5'd12;
        bit_count = 0;

        #200 nrst = 1;
        
        start = 1;

        fd = $fopen("../../../../signal.bin", "r");
        if (fd == 0) begin
            $display("Error opening signal.bin");
            $finish;
        end

        // Read the input signal from the binary file
        while (!$feof(fd)) begin
            if (bit_count == 0) begin
                // Read byte every 8 bits
                signal_byte = $fgetc(fd);
            end

            signal_in = signal_byte[bit_count];
            bit_count = bit_count + 1;
            @(posedge clk);
        end

        start = 0;

        wait (busy == 1'b0);

        #10 $finish; // End simulation
    end

endmodule