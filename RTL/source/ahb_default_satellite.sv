/********************************/
/*  AHB-Lite Bus Default Satellite  */
/********************************/
`timescale 1ns/1ns

`include "ahb_bus_if.vh"

`include "common_types.vh"
import common_types_pkg::*;

module ahb_default_satellite (
    input logic clk, nrst,
    ahb_bus_if.satellite_to_mux abif
);
    
    always_comb begin
        // Default response
        abif.hrdata = '0;
        abif.hreadyout = 1'b1;
        abif.hresp = 1'b0;
    end

endmodule