`ifndef AHB_BUS_IF_VH
`define AHB_BUS_IF_VH

`include "common_types.vh"
import common_types_pkg::*;

interface ahb_bus_if;
    
    word_t haddr;
    logic [2:0] hburst;
    logic [1:0] hsize;
    htrans_t htrans;
    word_t hwdata;
    word_t hrdata;
    logic hready;
    logic hreadyout;
    logic hresp;
    logic hwrite;
    logic hsel;

    modport controller_to_mux (
        input hrdata, hready, hresp,
        output haddr, hburst, hsize, htrans, hwdata, hwrite
    );

    modport mux_to_controller (
        output hrdata, hready, hresp,
        input haddr, hburst, hsize, htrans, hwdata, hwrite
    );

    modport mux_to_satellite (
        input hrdata, hreadyout, hresp,
        output haddr, hburst, hsize, htrans, hwdata, hwrite, hsel, hready
    );

    modport satellite_to_mux (
        input haddr, hburst, hsize, htrans, hwdata, hwrite, hsel, hready,
        output hrdata, hreadyout, hresp
    );
endinterface

`endif // AHB_BUS_IF_VH