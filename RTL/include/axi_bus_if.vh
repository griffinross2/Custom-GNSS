`ifndef AXI_BUS_IF_VH
`define AXI_BUS_IF_VH

`include "common_types.vh"
import common_types_pkg::*;

interface axi_bus_if;
    
    // Satellite Write Address Channel
    logic [3:0] awid;
    word_t awaddr;
    logic [7:0] awlen;
    logic [2:0] awsize;
    logic [1:0] awburst;
    logic awlock;
    logic [3:0] awcache;
    logic [2:0] awprot;
    logic [3:0] awqos;
    logic awvalid;
    logic awready;

    // Satellite Write Data Channel
    word_t wdata;
    logic [3:0] wstrb;
    logic wlast;
    logic wvalid;
    logic wready;

    // Satellite Write Response Channel
    logic [3:0] bid;
    logic [1:0] bresp;
    logic bvalid;
    logic bready;

    // Satellite Read Address Channel
    logic [3:0] arid;
    word_t araddr;
    logic [7:0] arlen;
    logic [2:0] arsize;
    logic [1:0] arburst;
    logic arlock;
    logic [3:0] arcache;
    logic [2:0] arprot;
    logic [3:0] arqos;
    logic arvalid;
    logic arready;

    // Satellite Read Data Channel
    logic [3:0] rid;
    word_t rdata;
    logic [1:0] rresp;
    logic rlast;
    logic rvalid;
    logic rready;

    modport controller_to_mux (
        // Satellite Write Address Channel
        output awid,
        output awaddr,
        output awlen,
        output awsize,
        output awburst,
        output awlock,
        output awcache,
        output awprot,
        output awqos,
        output awvalid,
        input awready,

        // Satellite Write Data Channel
        output wdata,
        output wstrb,
        output wlast,
        output wvalid,
        input wready,
        
        // Satellite Write Response Channel
        input bid,
        input bresp,
        input bvalid,
        output bready,

        // Satellite Read Address Channel
        output arid,
        output araddr,
        output arlen,
        output arsize,
        output arburst,
        output arlock,
        output arcache,
        output arprot,
        output arqos,
        output arvalid,
        input arready,

        // Satellite Read Data Channel
        input rid,
        input rdata,
        input rresp,
        input rlast,
        input rvalid,
        output rready
    );

    modport mux_to_controller (
        // Satellite Write Address Channel
        input awid,
        input awaddr,
        input awlen,
        input awsize,
        input awburst,
        input awlock,
        input awcache,
        input awprot,
        input awqos,
        input awvalid,
        output awready,

        // Satellite Write Data Channel
        input wdata,
        input wstrb,
        input wlast,
        input wvalid,
        output wready,

        // Satellite Write Response Channel
        output bid,
        output bresp,
        output bvalid,
        input bready,

        // Satellite Read Address Channel
        input arid,
        input araddr,
        input arlen,
        input arsize,
        input arburst,
        input arlock,
        input arcache,
        input arprot,
        input arqos,
        input arvalid,
        output arready,

        // Satellite Read Data Channel
        output rid,
        output rdata,
        output rresp,
        output rlast,
        output rvalid,
        input rready
    );

    modport mux_to_satellite (
        // Satellite Write Address Channel
        output awid,
        output awaddr,
        output awlen,
        output awsize,
        output awburst,
        output awlock,
        output awcache,
        output awprot,
        output awqos,
        output awvalid,
        input awready,

        // Satellite Write Data Channel
        output wdata,
        output wstrb,
        output wlast,
        output wvalid,
        input wready,
        
        // Satellite Write Response Channel
        input bid,
        input bresp,
        input bvalid,
        output bready,

        // Satellite Read Address Channel
        output arid,
        output araddr,
        output arlen,
        output arsize,
        output arburst,
        output arlock,
        output arcache,
        output arprot,
        output arqos,
        output arvalid,
        input arready,

        // Satellite Read Data Channel
        input rid,
        input rdata,
        input rresp,
        input rlast,
        input rvalid,
        output rready
    );

    modport satellite_to_mux (
        // Satellite Write Address Channel
        input awid,
        input awaddr,
        input awlen,
        input awsize,
        input awburst,
        input awlock,
        input awcache,
        input awprot,
        input awqos,
        input awvalid,
        output awready,

        // Satellite Write Data Channel
        input wdata,
        input wstrb,
        input wlast,
        input wvalid,
        output wready,

        // Satellite Write Response Channel
        output bid,
        output bresp,
        output bvalid,
        input bready,

        // Satellite Read Address Channel
        input arid,
        input araddr,
        input arlen,
        input arsize,
        input arburst,
        input arlock,
        input arcache,
        input arprot,
        input arqos,
        input arvalid,
        output arready,

        // Satellite Read Data Channel
        output rid,
        output rdata,
        output rresp,
        output rlast,
        output rvalid,
        input rready
    );
endinterface

`endif // AXI_BUS_IF_VH