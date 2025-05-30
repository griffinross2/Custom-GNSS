/************************/
/*   Full GNSS module   */
/************************/
`timescale 1ns/1ns

`include "common_types.vh"
import common_types_pkg::*;
`include "common_gnss_types.vh"
import common_gnss_types_pkg::*;
`include "ahb_bus_if.vh"

module gnss (
    input logic gnssclk, cpuclk, nrst,
    input logic signal_in,
    ahb_bus_if.satellite_to_mux abif
);
    // Internal signals
    logic search_busy;
    logic search_start;
    sv_t search_sv;
    logic [4:0] search_dop;
    word_t search_corr;
    logic [11:0] search_code;
    logic [7:0] fine_search_dop;
    word_t fine_search_corr;
    logic [14:0] fine_search_code;
    logic fine_search_busy;
    
    logic search_busy_reg;
    always_ff @(posedge gnssclk) begin
        if (~nrst) begin
            search_busy_reg <= 1'b0;
        end else begin
            search_busy_reg <= search_busy;
        end
    end

    ahb_gnss_satellite ahb_sat_inst (
        .clk(gnssclk),
        .hclk(cpuclk),
        .nrst(nrst),
        .abif(abif),
        .search_busy(search_busy),
        .fine_search_busy(fine_search_busy),
        .search_start(search_start),
        .search_sv(search_sv),
        .search_dop({24'b0, fine_search_dop}),
        .search_code({17'b0, fine_search_code}),
        .search_corr(fine_search_corr)
    );

    l1ca_search l1ca_search_inst (
        .clk(gnssclk),
        .nrst(nrst),
        .signal_in(signal_in),
        .start(search_start),
        .sv(search_sv),
        .acc_out(search_corr),
        .code_index(search_code),
        .dop_index(search_dop),
        .busy(search_busy)
    );

    l1ca_fine_search l1ca_fine_search_inst (
        .clk(gnssclk),
        .nrst(nrst),
        .signal_in(signal_in),
        .coarse_dop_idx(search_dop),
        .coarse_code_idx(search_code),
        .start(search_busy_reg && !search_busy),
        .sv(search_sv),
        .acc_out(fine_search_corr),
        .code_index(fine_search_code),
        .dop_index(fine_search_dop),
        .busy(fine_search_busy)
    );

endmodule