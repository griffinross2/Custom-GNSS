`timescale 1ns/1ns

`include "common_gnss_types.vh"
import common_gnss_types_pkg::*;

module l1ca_code (
    input logic clk, nrst,
    input logic en, clear,
    input sv_t sv,
    output logic code, epoch,
    output gps_chip_t chip
);

    gps_chip_t next_chip;

    l1ca_lfsr_t g1, next_g1;
    l1ca_lfsr_t g2, next_g2;

    always_ff @(posedge clk) begin
        if (~nrst) begin
            g1 <= '1;
            g2 <= '1;
            chip <= '0;
        end else begin
            g1 <= next_g1;
            g2 <= next_g2;
            chip <= next_chip;
        end
    end

    always_comb begin
        next_g1 = g1;
        next_g2 = g2;
        next_chip = chip;

        if (clear) begin
            next_g1 = '1;
            next_g2 = '1;
            next_chip = '0;
        end else if (en) begin
            // G1 polynomial: x^10 x^3 + 1
            next_g1[10:2] = g1[9:1];
            next_g1[1] = g1[10] ^ g1[3];

            // G2 polynomial: x^10 + x^9 + x^8 + x^6 + x^3 + x^2 + 1
            next_g2[10:2] = g2[9:1];
            next_g2[1] = g2[10] ^ g2[9] ^ g2[8] ^ g2[6] ^ g2[3] ^ g2[2];

            // Period of 1023 chips
            next_chip = chip + 10'd1;
            if (chip == 10'd1022) begin
                next_chip = '0;
            end
        end

        // Epoch happens when G1 is all ones again
        epoch = (g1 == '1) ? 1'b1 : 1'b0;

        // Code is the modulo 2 sum of G1[10] and 2 positions in G2 determined by the SV
        case (sv)
            6'd0: code = g1[10] ^ g2[2] ^ g2[6];
            6'd1: code = g1[10] ^ g2[3] ^ g2[7];
            6'd2: code = g1[10] ^ g2[4] ^ g2[8];
            6'd3: code = g1[10] ^ g2[5] ^ g2[9];
            6'd4: code = g1[10] ^ g2[1] ^ g2[9];
            6'd5: code = g1[10] ^ g2[2] ^ g2[10];
            6'd6: code = g1[10] ^ g2[1] ^ g2[8];
            6'd7: code = g1[10] ^ g2[2] ^ g2[9];
            6'd8: code = g1[10] ^ g2[3] ^ g2[10];
            6'd9: code = g1[10] ^ g2[2] ^ g2[3];
            6'd10: code = g1[10] ^ g2[3] ^ g2[4];
            6'd11: code = g1[10] ^ g2[5] ^ g2[6];
            6'd12: code = g1[10] ^ g2[6] ^ g2[7];
            6'd13: code = g1[10] ^ g2[7] ^ g2[8];
            6'd14: code = g1[10] ^ g2[8] ^ g2[9];
            6'd15: code = g1[10] ^ g2[9] ^ g2[10];
            6'd16: code = g1[10] ^ g2[1] ^ g2[4];
            6'd17: code = g1[10] ^ g2[2] ^ g2[5];
            6'd18: code = g1[10] ^ g2[3] ^ g2[6];
            6'd19: code = g1[10] ^ g2[4] ^ g2[7];
            6'd20: code = g1[10] ^ g2[5] ^ g2[8];
            6'd21: code = g1[10] ^ g2[6] ^ g2[9];
            6'd22: code = g1[10] ^ g2[1] ^ g2[3];
            6'd23: code = g1[10] ^ g2[4] ^ g2[6];
            6'd24: code = g1[10] ^ g2[5] ^ g2[7];
            6'd25: code = g1[10] ^ g2[6] ^ g2[8];
            6'd26: code = g1[10] ^ g2[7] ^ g2[9];
            6'd27: code = g1[10] ^ g2[8] ^ g2[10];
            6'd28: code = g1[10] ^ g2[1] ^ g2[6];
            6'd29: code = g1[10] ^ g2[2] ^ g2[7];
            6'd30: code = g1[10] ^ g2[3] ^ g2[8];
            6'd31: code = g1[10] ^ g2[4] ^ g2[9];
            default: code = 1'b0;
        endcase
    end

endmodule