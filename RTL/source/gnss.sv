/************************/
/*   Full GNSS module   */
/************************/
`timescale 1ns/1ns

`include "common_types.vh"
import common_types_pkg::*;
`include "common_gnss_types.vh"
import common_gnss_types_pkg::*;
`include "ahb_bus_if.vh"
`include "gnss_channel_satellite_if.vh"

module gnss (
    input logic gnssclk, cpuclk, nrst,
    input logic signal_in,
    ahb_bus_if.satellite_to_mux abif
);
    // Internal signals
    logic search_busy;
    logic search_start;
    sv_t search_sv;
    sv_t channel_sv;
    logic channel_start;
    logic [5:0] search_channel_in;
    logic [5:0] search_channel_out;
    logic [5:0] search_dop;
    word_t search_corr;
    logic [9:0] search_code;
    logic [9:0] search_code_adjusted;       // Fixed for code slip and doppler
    logic [4:0] search_subcode;
    logic [4:0] search_subcode_adjusted;    // Fixed for code slip and doppler

    gnss_channel_satellite_if channel_0 ();

    gps_chip_t chip_0;
    word_t code_phase_0;
    word_t lo_phase_0;

    always_comb begin
        search_code_adjusted = search_code;
        case (signed'(search_dop))
            // Adjust code index for doppler and code slip
            -6'd20: search_code_adjusted = (10'd1023 - search_code + 10'd69) % 10'd1023;
            -6'd19: search_code_adjusted = (10'd1023 - search_code + 10'd69) % 10'd1023;
            -6'd18: search_code_adjusted = (10'd1023 - search_code + 10'd69) % 10'd1023;
            -6'd17: search_code_adjusted = (10'd1023 - search_code + 10'd69) % 10'd1023;
            -6'd16: search_code_adjusted = (10'd1023 - search_code + 10'd69) % 10'd1023;
            -6'd15: search_code_adjusted = (10'd1023 - search_code + 10'd69) % 10'd1023;
            -6'd14: search_code_adjusted = (10'd1023 - search_code + 10'd69) % 10'd1023;
            -6'd13: search_code_adjusted = (10'd1023 - search_code + 10'd69) % 10'd1023;
            -6'd12: search_code_adjusted = (10'd1023 - search_code + 10'd70) % 10'd1023;
            -6'd11: search_code_adjusted = (10'd1023 - search_code + 10'd70) % 10'd1023;
            -6'd10: search_code_adjusted = (10'd1023 - search_code + 10'd70) % 10'd1023;
            -6'd09: search_code_adjusted = (10'd1023 - search_code + 10'd70) % 10'd1023;
            -6'd08: search_code_adjusted = (10'd1023 - search_code + 10'd70) % 10'd1023;
            -6'd07: search_code_adjusted = (10'd1023 - search_code + 10'd70) % 10'd1023;
            -6'd06: search_code_adjusted = (10'd1023 - search_code + 10'd70) % 10'd1023;
            -6'd05: search_code_adjusted = (10'd1023 - search_code + 10'd70) % 10'd1023;
            -6'd04: search_code_adjusted = (10'd1023 - search_code + 10'd70) % 10'd1023;
            -6'd03: search_code_adjusted = (10'd1023 - search_code + 10'd70) % 10'd1023;
            -6'd02: search_code_adjusted = (10'd1023 - search_code + 10'd70) % 10'd1023;
            -6'd01: search_code_adjusted = (10'd1023 - search_code + 10'd70) % 10'd1023;
            6'd00: search_code_adjusted = (10'd1023 - search_code + 10'd70) % 10'd1023;
            6'd01: search_code_adjusted = (10'd1023 - search_code + 10'd70) % 10'd1023;
            6'd02: search_code_adjusted = (10'd1023 - search_code + 10'd70) % 10'd1023;
            6'd03: search_code_adjusted = (10'd1023 - search_code + 10'd70) % 10'd1023;
            6'd04: search_code_adjusted = (10'd1023 - search_code + 10'd70) % 10'd1023;
            6'd05: search_code_adjusted = (10'd1023 - search_code + 10'd70) % 10'd1023;
            6'd06: search_code_adjusted = (10'd1023 - search_code + 10'd70) % 10'd1023;
            6'd07: search_code_adjusted = (10'd1023 - search_code + 10'd70) % 10'd1023;
            6'd08: search_code_adjusted = (10'd1023 - search_code + 10'd70) % 10'd1023;
            6'd09: search_code_adjusted = (10'd1023 - search_code + 10'd70) % 10'd1023;
            6'd10: search_code_adjusted = (10'd1023 - search_code + 10'd70) % 10'd1023;
            6'd11: search_code_adjusted = (10'd1023 - search_code + 10'd70) % 10'd1023;
            6'd12: search_code_adjusted = (10'd1023 - search_code + 10'd70) % 10'd1023;
            6'd13: search_code_adjusted = (10'd1023 - search_code + 10'd70) % 10'd1023;
            6'd14: search_code_adjusted = (10'd1023 - search_code + 10'd70) % 10'd1023;
            6'd15: search_code_adjusted = (10'd1023 - search_code + 10'd71) % 10'd1023;
            6'd16: search_code_adjusted = (10'd1023 - search_code + 10'd71) % 10'd1023;
            6'd17: search_code_adjusted = (10'd1023 - search_code + 10'd71) % 10'd1023;
            6'd18: search_code_adjusted = (10'd1023 - search_code + 10'd71) % 10'd1023;
            6'd19: search_code_adjusted = (10'd1023 - search_code + 10'd71) % 10'd1023;
            6'd20: search_code_adjusted = (10'd1023 - search_code + 10'd71) % 10'd1023;
            default: search_code_adjusted = (10'd1023 - search_code + 10'd70) % 10'd1023;
        endcase

        search_subcode_adjusted = search_subcode;
        case (signed'(search_dop))
            // Adjust sub code chip index for doppler and code slip
            -6'd20: search_subcode_adjusted = (search_subcode + 5'd13) % 5'd19;
            -6'd19: search_subcode_adjusted = (search_subcode + 5'd13) % 5'd19;
            -6'd18: search_subcode_adjusted = (search_subcode + 5'd15) % 5'd19;
            -6'd17: search_subcode_adjusted = (search_subcode + 5'd15) % 5'd19;
            -6'd16: search_subcode_adjusted = (search_subcode + 5'd15) % 5'd19;
            -6'd15: search_subcode_adjusted = (search_subcode + 5'd17) % 5'd19;
            -6'd14: search_subcode_adjusted = (search_subcode + 5'd17) % 5'd19;
            -6'd13: search_subcode_adjusted = (search_subcode + 5'd17) % 5'd19;
            -6'd12: search_subcode_adjusted = (search_subcode + 5'd0) % 5'd19;
            -6'd11: search_subcode_adjusted = (search_subcode + 5'd0) % 5'd19;
            -6'd10: search_subcode_adjusted = (search_subcode + 5'd2) % 5'd19;
            -6'd09: search_subcode_adjusted = (search_subcode + 5'd2) % 5'd19;
            -6'd08: search_subcode_adjusted = (search_subcode + 5'd2) % 5'd19;
            -6'd07: search_subcode_adjusted = (search_subcode + 5'd4) % 5'd19;
            -6'd06: search_subcode_adjusted = (search_subcode + 5'd4) % 5'd19;
            -6'd05: search_subcode_adjusted = (search_subcode + 5'd4) % 5'd19;
            -6'd04: search_subcode_adjusted = (search_subcode + 5'd6) % 5'd19;
            -6'd03: search_subcode_adjusted = (search_subcode + 5'd6) % 5'd19;
            -6'd02: search_subcode_adjusted = (search_subcode + 5'd6) % 5'd19;
            -6'd01: search_subcode_adjusted = (search_subcode + 5'd8) % 5'd19;
            6'd00: search_subcode_adjusted = (search_subcode + 5'd8) % 5'd19;
            6'd01: search_subcode_adjusted = (search_subcode + 5'd9) % 5'd19;
            6'd02: search_subcode_adjusted = (search_subcode + 5'd9) % 5'd19;
            6'd03: search_subcode_adjusted = (search_subcode + 5'd9) % 5'd19;
            6'd04: search_subcode_adjusted = (search_subcode + 5'd11) % 5'd19;
            6'd05: search_subcode_adjusted = (search_subcode + 5'd11) % 5'd19;
            6'd06: search_subcode_adjusted = (search_subcode + 5'd11) % 5'd19;
            6'd07: search_subcode_adjusted = (search_subcode + 5'd13) % 5'd19;
            6'd08: search_subcode_adjusted = (search_subcode + 5'd13) % 5'd19;
            6'd09: search_subcode_adjusted = (search_subcode + 5'd13) % 5'd19;
            6'd10: search_subcode_adjusted = (search_subcode + 5'd15) % 5'd19;
            6'd11: search_subcode_adjusted = (search_subcode + 5'd15) % 5'd19;
            6'd12: search_subcode_adjusted = (search_subcode + 5'd17) % 5'd19;
            6'd13: search_subcode_adjusted = (search_subcode + 5'd17) % 5'd19;
            6'd14: search_subcode_adjusted = (search_subcode + 5'd17) % 5'd19;
            6'd15: search_subcode_adjusted = (search_subcode + 5'd0) % 5'd19;
            6'd16: search_subcode_adjusted = (search_subcode + 5'd0) % 5'd19;
            6'd17: search_subcode_adjusted = (search_subcode + 5'd0) % 5'd19;
            6'd18: search_subcode_adjusted = (search_subcode + 5'd2) % 5'd19;
            6'd19: search_subcode_adjusted = (search_subcode + 5'd2) % 5'd19;
            6'd20: search_subcode_adjusted = (search_subcode + 5'd2) % 5'd19;
            default: search_subcode_adjusted = (search_subcode + 5'd8) % 5'd19;
        endcase
    end

    ahb_gnss_satellite ahb_sat_inst (
        .clk(gnssclk),
        .hclk(cpuclk),
        .nrst(nrst),
        .abif(abif),
        .search_busy(search_busy),
        .search_start(search_start),
        .search_sv(search_sv),
        .search_channel(search_channel_in),
        .search_dop({{26{search_dop[5]}}, search_dop}),
        .search_code({22'b0, 10'd1023 - search_code}),
        .search_subcode({27'b0, search_subcode}),
        .search_corr(search_corr),
        .search_channel_in(search_channel_out),
        .search_done(channel_start),
        .channel_0(channel_0)
    );

    ac_pca_search l1ca_search_inst (
        .clk(gnssclk),
        .nrst(nrst),
        .signal_in(signal_in),
        .start(search_start),
        .channel_in(search_channel_in),
        .sv(search_sv),
        .acc_out(search_corr),
        .code_index(search_code),
        .start_index(search_subcode),
        .dop_index(search_dop),
        .channel_out(search_channel_out),
        .sv_out(channel_sv),
        .start_out(channel_start),
        .busy(search_busy)
    );

    l1ca_channel l1ca_channel_0 (
        .clk(gnssclk),
        .nrst(nrst),
        .start(channel_start && search_channel_out == 6'd0),
        .clear(channel_0.clear),
        .sv(channel_sv),
        .code_rate(channel_0.code_rate),
        .lo_rate(channel_0.lo_rate),
        .signal_in(signal_in),
        .code_index(search_code_adjusted),
        .start_index(search_subcode_adjusted),
        .epoch(channel_0.epoch),
        .chip(chip_0),
        .ie(channel_0.ie),
        .qe(channel_0.qe),
        .ip(channel_0.ip),
        .qp(channel_0.qp),
        .il(channel_0.il),
        .ql(channel_0.ql),
        .code_phase(code_phase_0),
        .lo_phase(lo_phase_0)
    );

endmodule