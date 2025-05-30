/**************************/
/*  AXI Bus Interconnect  */
/**************************/
`timescale 1ns/1ns

`include "axi_bus_if.vh"

`include "common_types.vh"
import common_types_pkg::*;

module axi_interconnect (
    input logic clk, nrst,
    axi_bus_if.mux_to_controller abif_to_icache,
    axi_bus_if.mux_to_controller abif_to_dcache,
    axi_bus_if.mux_to_controller abif_to_dma,
    axi_bus_if.mux_to_satellite abif_to_sram,
    axi_bus_if.mux_to_satellite abif_to_flash,
    axi_bus_if.mux_to_satellite abif_to_dram,
    axi_bus_if.mux_to_satellite abif_to_ahb
);

// The job of the interconnect will be to route requests from the controllers
// to the appropriate satellites based on the addresses. In this implementation, we
// will only allow for each satellite to have one transaction active at a time.

typedef enum logic {
    IDLE,
    BUSY
} satellite_state_t;

typedef logic [1:0] satellite_controller_t;

satellite_state_t sram_state, next_sram_state;
satellite_controller_t sram_controller, next_sram_controller;
satellite_state_t flash_state, next_flash_state;
satellite_controller_t flash_controller, next_flash_controller;
satellite_state_t dram_state, next_dram_state;
satellite_controller_t dram_controller, next_dram_controller;
satellite_state_t ahb_state, next_ahb_state;
satellite_controller_t ahb_controller, next_ahb_controller;

logic icache_writing_sram;
logic dcache_writing_sram;
logic dma_writing_sram;
logic icache_reading_sram;
logic dcache_reading_sram;
logic dma_reading_sram;

logic icache_writing_flash;
logic dcache_writing_flash;
logic dma_writing_flash;
logic icache_reading_flash;
logic dcache_reading_flash;
logic dma_reading_flash;

logic icache_writing_dram;
logic dcache_writing_dram;
logic dma_writing_dram;
logic icache_reading_dram;
logic dcache_reading_dram;
logic dma_reading_dram;

logic icache_writing_ahb;
logic dcache_writing_ahb;
logic dma_writing_ahb;
logic icache_reading_ahb;
logic dcache_reading_ahb;
logic dma_reading_ahb;


// Arbitration

// It is important that we try to not to starve any of the controllers' requests
// To do this, after each request, we will prioritize the next controller in a fixed sequence
// In a scenario with all controllers requesting, this will result in a round-robin scheme
// Ex: icache -> dcache -> dma -> icache -> dcache -> dma
function satellite_controller_t get_next_controller;
    input satellite_controller_t current_controller;
    input logic icache_writing;
    input logic dcache_writing;
    input logic dma_writing;
    input logic icache_reading;
    input logic dcache_reading;
    input logic dma_reading;
    begin
    // 00: icache
    // 01: dcache
    // 10: dma

    case (current_controller)
        // Previous controller was icache
        2'b00: begin
            if (dcache_writing) begin
                get_next_controller = 2'b01;
            end else if (dma_writing) begin
                get_next_controller = 2'b10;
            end else if (icache_writing) begin
                get_next_controller = 2'b00;
            end else if (dcache_reading) begin
                get_next_controller = 2'b01;
            end else if (dma_reading) begin
                get_next_controller = 2'b10;
            end else if (icache_reading) begin
                get_next_controller = 2'b00;
            end else begin
                get_next_controller = current_controller; // Default to same
            end
        end

        // Previous controller was dcache
        2'b01: begin
            if (dma_writing) begin
                get_next_controller = 2'b10;
            end else if (icache_writing) begin
                get_next_controller = 2'b00;
            end else if (dcache_writing) begin
                get_next_controller = 2'b01;
            end else if (dma_reading) begin
                get_next_controller = 2'b10;
            end else if (icache_reading) begin
                get_next_controller = 2'b00;
            end else if (dcache_reading) begin
                get_next_controller = 2'b01;
            end else begin
                get_next_controller = current_controller; // Default to same
            end
        end

        // Previous controller was dma
        2'b10: begin
            if (icache_writing) begin
                get_next_controller = 2'b00;
            end else if (dcache_writing) begin
                get_next_controller = 2'b01;
            end else if (dma_writing) begin
                get_next_controller = 2'b10;
            end else if (icache_reading) begin
                get_next_controller = 2'b00;
            end else if (dcache_reading) begin
                get_next_controller = 2'b01;
            end else if (dma_reading) begin
                get_next_controller = 2'b10;
            end else begin
                get_next_controller = current_controller; // Default to same
            end
        end

        // Default case
        default: begin
            get_next_controller = 2'b00;
        end
    endcase
    end
endfunction

always_comb begin
    next_sram_state = sram_state;
    next_sram_controller = sram_controller;
    next_flash_state = flash_state;
    next_flash_controller = flash_controller;
    next_dram_state = dram_state;
    next_dram_controller = dram_controller;
    next_ahb_state = ahb_state;
    next_ahb_controller = ahb_controller;

    // First check the requests
    icache_writing_sram = abif_to_icache.awvalid && abif_to_icache.awaddr < 32'h0000_1000;
    dcache_writing_sram = abif_to_dcache.awvalid && abif_to_dcache.awaddr < 32'h0000_1000;
    dma_writing_sram = abif_to_dma.awvalid && abif_to_dma.awaddr < 32'h0000_1000;

    icache_reading_sram = abif_to_icache.arvalid && abif_to_icache.araddr < 32'h0000_1000;
    dcache_reading_sram = abif_to_dcache.arvalid && abif_to_dcache.araddr < 32'h0000_1000;
    dma_reading_sram = abif_to_dma.arvalid && abif_to_dma.araddr < 32'h0000_1000;

    icache_writing_flash = abif_to_icache.awvalid && abif_to_icache.awaddr >= 32'h0080_0000 && abif_to_icache.awaddr < 32'h0100_0000;
    dcache_writing_flash = abif_to_dcache.awvalid && abif_to_dcache.awaddr >= 32'h0080_0000 && abif_to_dcache.awaddr < 32'h0100_0000;
    dma_writing_flash = abif_to_dma.awvalid && abif_to_dma.awaddr >= 32'h0080_0000 && abif_to_dma.awaddr < 32'h0100_0000;
    
    icache_reading_flash = abif_to_icache.arvalid && abif_to_icache.araddr >= 32'h0080_0000 && abif_to_icache.araddr < 32'h0100_0000;
    dcache_reading_flash = abif_to_dcache.arvalid && abif_to_dcache.araddr >= 32'h0080_0000 && abif_to_dcache.araddr < 32'h0100_0000;
    dma_reading_flash = abif_to_dma.arvalid && abif_to_dma.araddr >= 32'h0080_0000 && abif_to_dma.araddr < 32'h0100_0000;

    icache_writing_dram = abif_to_icache.awvalid && abif_to_icache.awaddr >= 32'h1000_0000 && abif_to_icache.awaddr < 32'h2000_0000;
    dcache_writing_dram = abif_to_dcache.awvalid && abif_to_dcache.awaddr >= 32'h1000_0000 && abif_to_dcache.awaddr < 32'h2000_0000;
    dma_writing_dram = abif_to_dma.awvalid && abif_to_dma.awaddr >= 32'h1000_0000 && abif_to_dma.awaddr < 32'h2000_0000;

    icache_reading_dram = abif_to_icache.arvalid && abif_to_icache.araddr >= 32'h1000_0000 && abif_to_icache.araddr < 32'h2000_0000;
    dcache_reading_dram = abif_to_dcache.arvalid && abif_to_dcache.araddr >= 32'h1000_0000 && abif_to_dcache.araddr < 32'h2000_0000;
    dma_reading_dram = abif_to_dma.arvalid && abif_to_dma.araddr >= 32'h1000_0000 && abif_to_dma.araddr < 32'h2000_0000;

    icache_writing_ahb = abif_to_icache.awvalid && abif_to_icache.awaddr >= 32'h2000_0000;
    dcache_writing_ahb = abif_to_dcache.awvalid && abif_to_dcache.awaddr >= 32'h2000_0000;
    dma_writing_ahb = abif_to_dma.awvalid && abif_to_dma.awaddr >= 32'h2000_0000;

    icache_reading_ahb = abif_to_icache.arvalid && abif_to_icache.araddr >= 32'h2000_0000;
    dcache_reading_ahb = abif_to_dcache.arvalid && abif_to_dcache.araddr >= 32'h2000_0000;
    dma_reading_ahb = abif_to_dma.arvalid && abif_to_dma.araddr >= 32'h2000_0000;
    
    // Now arbitrate the SRAM
    if (sram_state == IDLE && (icache_writing_sram || dcache_writing_sram || dma_writing_sram || icache_reading_sram || dcache_reading_sram || dma_reading_sram)) begin
        next_sram_state = BUSY;
        next_sram_controller = get_next_controller(sram_controller, icache_writing_sram, dcache_writing_sram, dma_writing_sram, icache_reading_sram, dcache_reading_sram, dma_reading_sram);
    end

    // Now arbitrate the flash
    if (flash_state == IDLE && (icache_writing_flash || dcache_writing_flash || dma_writing_flash || icache_reading_flash || dcache_reading_flash || dma_reading_flash)) begin
        next_flash_state = BUSY;
        next_flash_controller = get_next_controller(flash_controller, icache_writing_flash, dcache_writing_flash, dma_writing_flash, icache_reading_flash, dcache_reading_flash, dma_reading_flash);
    end

    // Now arbitrate the dram
    if (dram_state == IDLE && (icache_writing_dram || dcache_writing_dram || dma_writing_dram || icache_reading_dram || dcache_reading_dram || dma_reading_dram)) begin
        next_dram_state = BUSY;
        next_dram_controller = get_next_controller(dram_controller, icache_writing_dram, dcache_writing_dram, dma_writing_dram, icache_reading_dram, dcache_reading_dram, dma_reading_dram);
    end

    // Now arbitrate the ahb
    if (ahb_state == IDLE && (icache_writing_ahb || dcache_writing_ahb || dma_writing_ahb || icache_reading_ahb || dcache_reading_ahb || dma_reading_ahb)) begin
        next_ahb_state = BUSY;
        next_ahb_controller = get_next_controller(ahb_controller, icache_writing_ahb, dcache_writing_ahb, dma_writing_ahb, icache_reading_ahb, dcache_reading_ahb, dma_reading_ahb);
    end

    // Check for the completion of the transactions
    if (sram_state == BUSY && ((abif_to_sram.bvalid && abif_to_sram.bready) || (abif_to_sram.rvalid && abif_to_sram.rready))) begin
        next_sram_state = IDLE;
    end

    if (flash_state == BUSY && ((abif_to_flash.bvalid && abif_to_flash.bready) || (abif_to_flash.rvalid && abif_to_flash.rready))) begin
        next_flash_state = IDLE;
    end

    if (dram_state == BUSY && ((abif_to_dram.bvalid && abif_to_dram.bready) || (abif_to_dram.rvalid && abif_to_dram.rready))) begin
        next_dram_state = IDLE;
    end

    if (ahb_state == BUSY && ((abif_to_ahb.bvalid && abif_to_ahb.bready) || (abif_to_ahb.rvalid && abif_to_ahb.rready))) begin
        next_ahb_state = IDLE;
    end
end

// Connect the controllers to the satellites
always_comb begin
    // Default signals
    abif_to_icache.awready = 1'b0;
    abif_to_icache.wready = 1'b0;
    abif_to_icache.bid = 4'b0;
    abif_to_icache.bresp = 2'b00;
    abif_to_icache.bvalid = 1'b0;
    abif_to_icache.arready = 1'b0;
    abif_to_icache.rid = 4'b0;
    abif_to_icache.rdata = 32'b0;
    abif_to_icache.rresp = 2'b00;
    abif_to_icache.rvalid = 1'b0;
    abif_to_icache.rlast = 1'b0;

    abif_to_dcache.awready = 1'b0;
    abif_to_dcache.wready = 1'b0;
    abif_to_dcache.bid = 4'b0;
    abif_to_dcache.bresp = 2'b00;
    abif_to_dcache.bvalid = 1'b0;
    abif_to_dcache.arready = 1'b0;
    abif_to_dcache.rid = 4'b0;
    abif_to_dcache.rdata = 32'b0;
    abif_to_dcache.rresp = 2'b00;
    abif_to_dcache.rvalid = 1'b0;
    abif_to_dcache.rlast = 1'b0;

    abif_to_dma.awready = 1'b0;
    abif_to_dma.wready = 1'b0;
    abif_to_dma.bid = 4'b0;
    abif_to_dma.bresp = 2'b00;
    abif_to_dma.bvalid = 1'b0;
    abif_to_dma.arready = 1'b0;
    abif_to_dma.rid = 4'b0;
    abif_to_dma.rdata = 32'b0;
    abif_to_dma.rresp = 2'b00;
    abif_to_dma.rvalid = 1'b0;
    abif_to_dma.rlast = 1'b0;


    // Connect the controllers to the satellites
    if (sram_state == BUSY) begin
        casez (sram_controller)
            // Connect to icache
            2'b00: begin
                abif_to_icache.awready = abif_to_sram.awready;
                abif_to_icache.wready = abif_to_sram.wready;
                abif_to_icache.bid = abif_to_sram.bid;
                abif_to_icache.bresp = abif_to_sram.bresp;
                abif_to_icache.bvalid = abif_to_sram.bvalid;
                abif_to_icache.arready = abif_to_sram.arready;
                abif_to_icache.rid = abif_to_sram.rid;
                abif_to_icache.rdata = abif_to_sram.rdata;
                abif_to_icache.rresp = abif_to_sram.rresp;
                abif_to_icache.rvalid = abif_to_sram.rvalid;
                abif_to_icache.rlast = abif_to_sram.rlast;
                abif_to_sram.awid =  abif_to_icache.awid;
                abif_to_sram.awaddr = abif_to_icache.awaddr;
                abif_to_sram.awlen = abif_to_icache.awlen;
                abif_to_sram.awsize = abif_to_icache.awsize;
                abif_to_sram.awburst = abif_to_icache.awburst;
                abif_to_sram.awlock = abif_to_icache.awlock;
                abif_to_sram.awcache = abif_to_icache.awcache;
                abif_to_sram.awprot = abif_to_icache.awprot;
                abif_to_sram.awqos = abif_to_icache.awqos;
                abif_to_sram.awvalid = abif_to_icache.awvalid;
                abif_to_sram.wdata = abif_to_icache.wdata;
                abif_to_sram.wstrb = abif_to_icache.wstrb;
                abif_to_sram.wlast = abif_to_icache.wlast;
                abif_to_sram.wvalid = abif_to_icache.wvalid;
                abif_to_sram.bready = abif_to_icache.bready;
                abif_to_sram.arid =  abif_to_icache.arid ;
                abif_to_sram.araddr = abif_to_icache.araddr;
                abif_to_sram.arlen = abif_to_icache.arlen;
                abif_to_sram.arsize = abif_to_icache.arsize;
                abif_to_sram.arburst = abif_to_icache.arburst;
                abif_to_sram.arlock = abif_to_icache.arlock;
                abif_to_sram.arcache = abif_to_icache.arcache;
                abif_to_sram.arprot = abif_to_icache.arprot;
                abif_to_sram.arqos = abif_to_icache.arqos;
                abif_to_sram.arvalid = abif_to_icache.arvalid;
                abif_to_sram.rready = abif_to_icache.rready;
            end
            // Connect to dcache
            2'b01: begin
                abif_to_dcache.awready = abif_to_sram.awready;
                abif_to_dcache.wready = abif_to_sram.wready;
                abif_to_dcache.bid = abif_to_sram.bid;
                abif_to_dcache.bresp = abif_to_sram.bresp;
                abif_to_dcache.bvalid = abif_to_sram.bvalid;
                abif_to_dcache.arready = abif_to_sram.arready;
                abif_to_dcache.rid = abif_to_sram.rid;
                abif_to_dcache.rdata = abif_to_sram.rdata;
                abif_to_dcache.rresp = abif_to_sram.rresp;
                abif_to_dcache.rvalid = abif_to_sram.rvalid;
                abif_to_dcache.rlast = abif_to_sram.rlast;
                abif_to_sram.awid =  abif_to_dcache.awid;
                abif_to_sram.awaddr = abif_to_dcache.awaddr;
                abif_to_sram.awlen = abif_to_dcache.awlen;
                abif_to_sram.awsize = abif_to_dcache.awsize;
                abif_to_sram.awburst = abif_to_dcache.awburst;
                abif_to_sram.awlock = abif_to_dcache.awlock;
                abif_to_sram.awcache = abif_to_dcache.awcache;
                abif_to_sram.awprot = abif_to_dcache.awprot;
                abif_to_sram.awqos = abif_to_dcache.awqos;
                abif_to_sram.awvalid = abif_to_dcache.awvalid;
                abif_to_sram.wdata = abif_to_dcache.wdata;
                abif_to_sram.wstrb = abif_to_dcache.wstrb;
                abif_to_sram.wlast = abif_to_dcache.wlast;
                abif_to_sram.wvalid = abif_to_dcache.wvalid;
                abif_to_sram.bready = abif_to_dcache.bready;
                abif_to_sram.arid =  abif_to_dcache.arid ;
                abif_to_sram.araddr = abif_to_dcache.araddr;
                abif_to_sram.arlen = abif_to_dcache.arlen;
                abif_to_sram.arsize = abif_to_dcache.arsize;
                abif_to_sram.arburst = abif_to_dcache.arburst;
                abif_to_sram.arlock = abif_to_dcache.arlock;
                abif_to_sram.arcache = abif_to_dcache.arcache;
                abif_to_sram.arprot = abif_to_dcache.arprot;
                abif_to_sram.arqos = abif_to_dcache.arqos;
                abif_to_sram.arvalid = abif_to_dcache.arvalid;
                abif_to_sram.rready = abif_to_dcache.rready;
            end
            // Connect to dma
            2'b10: begin
                abif_to_dma.awready = abif_to_sram.awready;
                abif_to_dma.wready = abif_to_sram.wready;
                abif_to_dma.bid = abif_to_sram.bid;
                abif_to_dma.bresp = abif_to_sram.bresp;
                abif_to_dma.bvalid = abif_to_sram.bvalid;
                abif_to_dma.arready = abif_to_sram.arready;
                abif_to_dma.rid = abif_to_sram.rid;
                abif_to_dma.rdata = abif_to_sram.rdata;
                abif_to_dma.rresp = abif_to_sram.rresp;
                abif_to_dma.rvalid = abif_to_sram.rvalid;
                abif_to_dma.rlast = abif_to_sram.rlast;
                abif_to_sram.awid =  abif_to_dma.awid;
                abif_to_sram.awaddr = abif_to_dma.awaddr;
                abif_to_sram.awlen = abif_to_dma.awlen;
                abif_to_sram.awsize = abif_to_dma.awsize;
                abif_to_sram.awburst = abif_to_dma.awburst;
                abif_to_sram.awlock = abif_to_dma.awlock;
                abif_to_sram.awcache = abif_to_dma.awcache;
                abif_to_sram.awprot = abif_to_dma.awprot;
                abif_to_sram.awqos = abif_to_dma.awqos;
                abif_to_sram.awvalid = abif_to_dma.awvalid;
                abif_to_sram.wdata = abif_to_dma.wdata;
                abif_to_sram.wstrb = abif_to_dma.wstrb;
                abif_to_sram.wlast = abif_to_dma.wlast;
                abif_to_sram.wvalid = abif_to_dma.wvalid;
                abif_to_sram.bready = abif_to_dma.bready;
                abif_to_sram.arid =  abif_to_dma.arid ;
                abif_to_sram.araddr = abif_to_dma.araddr;
                abif_to_sram.arlen = abif_to_dma.arlen;
                abif_to_sram.arsize = abif_to_dma.arsize;
                abif_to_sram.arburst = abif_to_dma.arburst;
                abif_to_sram.arlock = abif_to_dma.arlock;
                abif_to_sram.arcache = abif_to_dma.arcache;
                abif_to_sram.arprot = abif_to_dma.arprot;
                abif_to_sram.arqos = abif_to_dma.arqos;
                abif_to_sram.arvalid = abif_to_dma.arvalid;
                abif_to_sram.rready = abif_to_dma.rready;
            end
            default: begin
                abif_to_sram.awid = 4'b0;
                abif_to_sram.awaddr = 32'b0;
                abif_to_sram.awlen = 8'b0;
                abif_to_sram.awsize = 3'b0;
                abif_to_sram.awburst = 2'b0;
                abif_to_sram.awlock = 1'b0;
                abif_to_sram.awcache = 4'b0;
                abif_to_sram.awprot = 3'b0;
                abif_to_sram.awqos = 4'b0;
                abif_to_sram.awvalid = 1'b0;
                abif_to_sram.wdata = 32'b0;
                abif_to_sram.wstrb = 4'b0;
                abif_to_sram.wlast = 1'b0;
                abif_to_sram.wvalid = 1'b0;
                abif_to_sram.bready = 1'b0;
                abif_to_sram.arid = 4'b0;
                abif_to_sram.araddr = 32'b0;
                abif_to_sram.arlen = 8'b0;
                abif_to_sram.arsize = 3'b0;
                abif_to_sram.arburst = 2'b0;
                abif_to_sram.arlock = 1'b0;
                abif_to_sram.arcache = 4'b0;
                abif_to_sram.arprot = 3'b0;
                abif_to_sram.arqos = 4'b0;
                abif_to_sram.arvalid = 1'b0;
                abif_to_sram.rready = 1'b0;
            end
        endcase
    end else begin
        abif_to_sram.awid = 4'b0;
        abif_to_sram.awaddr = 32'b0;
        abif_to_sram.awlen = 8'b0;
        abif_to_sram.awsize = 3'b0;
        abif_to_sram.awburst = 2'b0;
        abif_to_sram.awlock = 1'b0;
        abif_to_sram.awcache = 4'b0;
        abif_to_sram.awprot = 3'b0;
        abif_to_sram.awqos = 4'b0;
        abif_to_sram.awvalid = 1'b0;
        abif_to_sram.wdata = 32'b0;
        abif_to_sram.wstrb = 4'b0;
        abif_to_sram.wlast = 1'b0;
        abif_to_sram.wvalid = 1'b0;
        abif_to_sram.bready = 1'b0;
        abif_to_sram.arid = 4'b0;
        abif_to_sram.araddr = 32'b0;
        abif_to_sram.arlen = 8'b0;
        abif_to_sram.arsize = 3'b0;
        abif_to_sram.arburst = 2'b0;
        abif_to_sram.arlock = 1'b0;
        abif_to_sram.arcache = 4'b0;
        abif_to_sram.arprot = 3'b0;
        abif_to_sram.arqos = 4'b0;
        abif_to_sram.arvalid = 1'b0;
        abif_to_sram.rready = 1'b0;
    end

    if (flash_state == BUSY) begin
        casez (flash_controller)
            // Connect to icache
            2'b00: begin
                abif_to_icache.awready = abif_to_flash.awready;
                abif_to_icache.wready = abif_to_flash.wready;
                abif_to_icache.bid = abif_to_flash.bid;
                abif_to_icache.bresp = abif_to_flash.bresp;
                abif_to_icache.bvalid = abif_to_flash.bvalid;
                abif_to_icache.arready = abif_to_flash.arready;
                abif_to_icache.rid = abif_to_flash.rid;
                abif_to_icache.rdata = abif_to_flash.rdata;
                abif_to_icache.rresp = abif_to_flash.rresp;
                abif_to_icache.rvalid = abif_to_flash.rvalid;
                abif_to_icache.rlast = abif_to_flash.rlast;
                abif_to_flash.awid =  abif_to_icache.awid;
                abif_to_flash.awaddr = abif_to_icache.awaddr;
                abif_to_flash.awlen = abif_to_icache.awlen;
                abif_to_flash.awsize = abif_to_icache.awsize;
                abif_to_flash.awburst = abif_to_icache.awburst;
                abif_to_flash.awlock = abif_to_icache.awlock;
                abif_to_flash.awcache = abif_to_icache.awcache;
                abif_to_flash.awprot = abif_to_icache.awprot;
                abif_to_flash.awqos = abif_to_icache.awqos;
                abif_to_flash.awvalid = abif_to_icache.awvalid;
                abif_to_flash.wdata = abif_to_icache.wdata;
                abif_to_flash.wstrb = abif_to_icache.wstrb;
                abif_to_flash.wlast = abif_to_icache.wlast;
                abif_to_flash.wvalid = abif_to_icache.wvalid;
                abif_to_flash.bready = abif_to_icache.bready;
                abif_to_flash.arid =  abif_to_icache.arid ;
                abif_to_flash.araddr = abif_to_icache.araddr;
                abif_to_flash.arlen = abif_to_icache.arlen;
                abif_to_flash.arsize = abif_to_icache.arsize;
                abif_to_flash.arburst = abif_to_icache.arburst;
                abif_to_flash.arlock = abif_to_icache.arlock;
                abif_to_flash.arcache = abif_to_icache.arcache;
                abif_to_flash.arprot = abif_to_icache.arprot;
                abif_to_flash.arqos = abif_to_icache.arqos;
                abif_to_flash.arvalid = abif_to_icache.arvalid;
                abif_to_flash.rready = abif_to_icache.rready;
            end
            // Connect to dcache
            2'b01: begin
                abif_to_dcache.awready = abif_to_flash.awready;
                abif_to_dcache.wready = abif_to_flash.wready;
                abif_to_dcache.bid = abif_to_flash.bid;
                abif_to_dcache.bresp = abif_to_flash.bresp;
                abif_to_dcache.bvalid = abif_to_flash.bvalid;
                abif_to_dcache.arready = abif_to_flash.arready;
                abif_to_dcache.rid = abif_to_flash.rid;
                abif_to_dcache.rdata = abif_to_flash.rdata;
                abif_to_dcache.rresp = abif_to_flash.rresp;
                abif_to_dcache.rvalid = abif_to_flash.rvalid;
                abif_to_dcache.rlast = abif_to_flash.rlast;
                abif_to_flash.awid =  abif_to_dcache.awid;
                abif_to_flash.awaddr = abif_to_dcache.awaddr;
                abif_to_flash.awlen = abif_to_dcache.awlen;
                abif_to_flash.awsize = abif_to_dcache.awsize;
                abif_to_flash.awburst = abif_to_dcache.awburst;
                abif_to_flash.awlock = abif_to_dcache.awlock;
                abif_to_flash.awcache = abif_to_dcache.awcache;
                abif_to_flash.awprot = abif_to_dcache.awprot;
                abif_to_flash.awqos = abif_to_dcache.awqos;
                abif_to_flash.awvalid = abif_to_dcache.awvalid;
                abif_to_flash.wdata = abif_to_dcache.wdata;
                abif_to_flash.wstrb = abif_to_dcache.wstrb;
                abif_to_flash.wlast = abif_to_dcache.wlast;
                abif_to_flash.wvalid = abif_to_dcache.wvalid;
                abif_to_flash.bready = abif_to_dcache.bready;
                abif_to_flash.arid =  abif_to_dcache.arid ;
                abif_to_flash.araddr = abif_to_dcache.araddr;
                abif_to_flash.arlen = abif_to_dcache.arlen;
                abif_to_flash.arsize = abif_to_dcache.arsize;
                abif_to_flash.arburst = abif_to_dcache.arburst;
                abif_to_flash.arlock = abif_to_dcache.arlock;
                abif_to_flash.arcache = abif_to_dcache.arcache;
                abif_to_flash.arprot = abif_to_dcache.arprot;
                abif_to_flash.arqos = abif_to_dcache.arqos;
                abif_to_flash.arvalid = abif_to_dcache.arvalid;
                abif_to_flash.rready = abif_to_dcache.rready;
            end
            // Connect to dma
            2'b10: begin
                abif_to_dma.awready = abif_to_flash.awready;
                abif_to_dma.wready = abif_to_flash.wready;
                abif_to_dma.bid = abif_to_flash.bid;
                abif_to_dma.bresp = abif_to_flash.bresp;
                abif_to_dma.bvalid = abif_to_flash.bvalid;
                abif_to_dma.arready = abif_to_flash.arready;
                abif_to_dma.rid = abif_to_flash.rid;
                abif_to_dma.rdata = abif_to_flash.rdata;
                abif_to_dma.rresp = abif_to_flash.rresp;
                abif_to_dma.rvalid = abif_to_flash.rvalid;
                abif_to_dma.rlast = abif_to_flash.rlast;
                abif_to_flash.awid =  abif_to_dma.awid;
                abif_to_flash.awaddr = abif_to_dma.awaddr;
                abif_to_flash.awlen = abif_to_dma.awlen;
                abif_to_flash.awsize = abif_to_dma.awsize;
                abif_to_flash.awburst = abif_to_dma.awburst;
                abif_to_flash.awlock = abif_to_dma.awlock;
                abif_to_flash.awcache = abif_to_dma.awcache;
                abif_to_flash.awprot = abif_to_dma.awprot;
                abif_to_flash.awqos = abif_to_dma.awqos;
                abif_to_flash.awvalid = abif_to_dma.awvalid;
                abif_to_flash.wdata = abif_to_dma.wdata;
                abif_to_flash.wstrb = abif_to_dma.wstrb;
                abif_to_flash.wlast = abif_to_dma.wlast;
                abif_to_flash.wvalid = abif_to_dma.wvalid;
                abif_to_flash.bready = abif_to_dma.bready;
                abif_to_flash.arid =  abif_to_dma.arid ;
                abif_to_flash.araddr = abif_to_dma.araddr;
                abif_to_flash.arlen = abif_to_dma.arlen;
                abif_to_flash.arsize = abif_to_dma.arsize;
                abif_to_flash.arburst = abif_to_dma.arburst;
                abif_to_flash.arlock = abif_to_dma.arlock;
                abif_to_flash.arcache = abif_to_dma.arcache;
                abif_to_flash.arprot = abif_to_dma.arprot;
                abif_to_flash.arqos = abif_to_dma.arqos;
                abif_to_flash.arvalid = abif_to_dma.arvalid;
                abif_to_flash.rready = abif_to_dma.rready;
            end
            default: begin
                abif_to_flash.awid = 4'b0;
                abif_to_flash.awaddr = 32'b0;
                abif_to_flash.awlen = 8'b0;
                abif_to_flash.awsize = 3'b0;
                abif_to_flash.awburst = 2'b0;
                abif_to_flash.awlock = 1'b0;
                abif_to_flash.awcache = 4'b0;
                abif_to_flash.awprot = 3'b0;
                abif_to_flash.awqos = 4'b0;
                abif_to_flash.awvalid = 1'b0;
                abif_to_flash.wdata = 32'b0;
                abif_to_flash.wstrb = 4'b0;
                abif_to_flash.wlast = 1'b0;
                abif_to_flash.wvalid = 1'b0;
                abif_to_flash.bready = 1'b0;
                abif_to_flash.arid = 4'b0;
                abif_to_flash.araddr = 32'b0;
                abif_to_flash.arlen = 8'b0;
                abif_to_flash.arsize = 3'b0;
                abif_to_flash.arburst = 2'b0;
                abif_to_flash.arlock = 1'b0;
                abif_to_flash.arcache = 4'b0;
                abif_to_flash.arprot = 3'b0;
                abif_to_flash.arqos = 4'b0;
                abif_to_flash.arvalid = 1'b0;
                abif_to_flash.rready = 1'b0;
            end
        endcase
    end else begin
        abif_to_flash.awid = 4'b0;
        abif_to_flash.awaddr = 32'b0;
        abif_to_flash.awlen = 8'b0;
        abif_to_flash.awsize = 3'b0;
        abif_to_flash.awburst = 2'b0;
        abif_to_flash.awlock = 1'b0;
        abif_to_flash.awcache = 4'b0;
        abif_to_flash.awprot = 3'b0;
        abif_to_flash.awqos = 4'b0;
        abif_to_flash.awvalid = 1'b0;
        abif_to_flash.wdata = 32'b0;
        abif_to_flash.wstrb = 4'b0;
        abif_to_flash.wlast = 1'b0;
        abif_to_flash.wvalid = 1'b0;
        abif_to_flash.bready = 1'b0;
        abif_to_flash.arid = 4'b0;
        abif_to_flash.araddr = 32'b0;
        abif_to_flash.arlen = 8'b0;
        abif_to_flash.arsize = 3'b0;
        abif_to_flash.arburst = 2'b0;
        abif_to_flash.arlock = 1'b0;
        abif_to_flash.arcache = 4'b0;
        abif_to_flash.arprot = 3'b0;
        abif_to_flash.arqos = 4'b0;
        abif_to_flash.arvalid = 1'b0;
        abif_to_flash.rready = 1'b0;
    end

    if (dram_state == BUSY) begin
        casez (dram_controller)
            // Connect to icache
            2'b00: begin
                abif_to_icache.awready = abif_to_dram.awready;
                abif_to_icache.wready = abif_to_dram.wready;
                abif_to_icache.bid = abif_to_dram.bid;
                abif_to_icache.bresp = abif_to_dram.bresp;
                abif_to_icache.bvalid = abif_to_dram.bvalid;
                abif_to_icache.arready = abif_to_dram.arready;
                abif_to_icache.rid = abif_to_dram.rid;
                abif_to_icache.rdata = abif_to_dram.rdata;
                abif_to_icache.rresp = abif_to_dram.rresp;
                abif_to_icache.rvalid = abif_to_dram.rvalid;
                abif_to_icache.rlast = abif_to_dram.rlast;
                abif_to_dram.awid =  abif_to_icache.awid;
                abif_to_dram.awaddr = abif_to_icache.awaddr;
                abif_to_dram.awlen = abif_to_icache.awlen;
                abif_to_dram.awsize = abif_to_icache.awsize;
                abif_to_dram.awburst = abif_to_icache.awburst;
                abif_to_dram.awlock = abif_to_icache.awlock;
                abif_to_dram.awcache = abif_to_icache.awcache;
                abif_to_dram.awprot = abif_to_icache.awprot;
                abif_to_dram.awqos = abif_to_icache.awqos;
                abif_to_dram.awvalid = abif_to_icache.awvalid;
                abif_to_dram.wdata = abif_to_icache.wdata;
                abif_to_dram.wstrb = abif_to_icache.wstrb;
                abif_to_dram.wlast = abif_to_icache.wlast;
                abif_to_dram.wvalid = abif_to_icache.wvalid;
                abif_to_dram.bready = abif_to_icache.bready;
                abif_to_dram.arid =  abif_to_icache.arid ;
                abif_to_dram.araddr = abif_to_icache.araddr;
                abif_to_dram.arlen = abif_to_icache.arlen;
                abif_to_dram.arsize = abif_to_icache.arsize;
                abif_to_dram.arburst = abif_to_icache.arburst;
                abif_to_dram.arlock = abif_to_icache.arlock;
                abif_to_dram.arcache = abif_to_icache.arcache;
                abif_to_dram.arprot = abif_to_icache.arprot;
                abif_to_dram.arqos = abif_to_icache.arqos;
                abif_to_dram.arvalid = abif_to_icache.arvalid;
                abif_to_dram.rready = abif_to_icache.rready;
            end
            // Connect to dcache
            2'b01: begin
                abif_to_dcache.awready = abif_to_dram.awready;
                abif_to_dcache.wready = abif_to_dram.wready;
                abif_to_dcache.bid = abif_to_dram.bid;
                abif_to_dcache.bresp = abif_to_dram.bresp;
                abif_to_dcache.bvalid = abif_to_dram.bvalid;
                abif_to_dcache.arready = abif_to_dram.arready;
                abif_to_dcache.rid = abif_to_dram.rid;
                abif_to_dcache.rdata = abif_to_dram.rdata;
                abif_to_dcache.rresp = abif_to_dram.rresp;
                abif_to_dcache.rvalid = abif_to_dram.rvalid;
                abif_to_dcache.rlast = abif_to_dram.rlast;
                abif_to_dram.awid =  abif_to_dcache.awid;
                abif_to_dram.awaddr = abif_to_dcache.awaddr;
                abif_to_dram.awlen = abif_to_dcache.awlen;
                abif_to_dram.awsize = abif_to_dcache.awsize;
                abif_to_dram.awburst = abif_to_dcache.awburst;
                abif_to_dram.awlock = abif_to_dcache.awlock;
                abif_to_dram.awcache = abif_to_dcache.awcache;
                abif_to_dram.awprot = abif_to_dcache.awprot;
                abif_to_dram.awqos = abif_to_dcache.awqos;
                abif_to_dram.awvalid = abif_to_dcache.awvalid;
                abif_to_dram.wdata = abif_to_dcache.wdata;
                abif_to_dram.wstrb = abif_to_dcache.wstrb;
                abif_to_dram.wlast = abif_to_dcache.wlast;
                abif_to_dram.wvalid = abif_to_dcache.wvalid;
                abif_to_dram.bready = abif_to_dcache.bready;
                abif_to_dram.arid =  abif_to_dcache.arid ;
                abif_to_dram.araddr = abif_to_dcache.araddr;
                abif_to_dram.arlen = abif_to_dcache.arlen;
                abif_to_dram.arsize = abif_to_dcache.arsize;
                abif_to_dram.arburst = abif_to_dcache.arburst;
                abif_to_dram.arlock = abif_to_dcache.arlock;
                abif_to_dram.arcache = abif_to_dcache.arcache;
                abif_to_dram.arprot = abif_to_dcache.arprot;
                abif_to_dram.arqos = abif_to_dcache.arqos;
                abif_to_dram.arvalid = abif_to_dcache.arvalid;
                abif_to_dram.rready = abif_to_dcache.rready;
            end
            // Connect to dma
            2'b10: begin
                abif_to_dma.awready = abif_to_dram.awready;
                abif_to_dma.wready = abif_to_dram.wready;
                abif_to_dma.bid = abif_to_dram.bid;
                abif_to_dma.bresp = abif_to_dram.bresp;
                abif_to_dma.bvalid = abif_to_dram.bvalid;
                abif_to_dma.arready = abif_to_dram.arready;
                abif_to_dma.rid = abif_to_dram.rid;
                abif_to_dma.rdata = abif_to_dram.rdata;
                abif_to_dma.rresp = abif_to_dram.rresp;
                abif_to_dma.rvalid = abif_to_dram.rvalid;
                abif_to_dma.rlast = abif_to_dram.rlast;
                abif_to_dram.awid =  abif_to_dma.awid;
                abif_to_dram.awaddr = abif_to_dma.awaddr;
                abif_to_dram.awlen = abif_to_dma.awlen;
                abif_to_dram.awsize = abif_to_dma.awsize;
                abif_to_dram.awburst = abif_to_dma.awburst;
                abif_to_dram.awlock = abif_to_dma.awlock;
                abif_to_dram.awcache = abif_to_dma.awcache;
                abif_to_dram.awprot = abif_to_dma.awprot;
                abif_to_dram.awqos = abif_to_dma.awqos;
                abif_to_dram.awvalid = abif_to_dma.awvalid;
                abif_to_dram.wdata = abif_to_dma.wdata;
                abif_to_dram.wstrb = abif_to_dma.wstrb;
                abif_to_dram.wlast = abif_to_dma.wlast;
                abif_to_dram.wvalid = abif_to_dma.wvalid;
                abif_to_dram.bready = abif_to_dma.bready;
                abif_to_dram.arid =  abif_to_dma.arid ;
                abif_to_dram.araddr = abif_to_dma.araddr;
                abif_to_dram.arlen = abif_to_dma.arlen;
                abif_to_dram.arsize = abif_to_dma.arsize;
                abif_to_dram.arburst = abif_to_dma.arburst;
                abif_to_dram.arlock = abif_to_dma.arlock;
                abif_to_dram.arcache = abif_to_dma.arcache;
                abif_to_dram.arprot = abif_to_dma.arprot;
                abif_to_dram.arqos = abif_to_dma.arqos;
                abif_to_dram.arvalid = abif_to_dma.arvalid;
                abif_to_dram.rready = abif_to_dma.rready;
            end
            default: begin
                abif_to_dram.awid = 4'b0;
                abif_to_dram.awaddr = 32'b0;
                abif_to_dram.awlen = 8'b0;
                abif_to_dram.awsize = 3'b0;
                abif_to_dram.awburst = 2'b0;
                abif_to_dram.awlock = 1'b0;
                abif_to_dram.awcache = 4'b0;
                abif_to_dram.awprot = 3'b0;
                abif_to_dram.awqos = 4'b0;
                abif_to_dram.awvalid = 1'b0;
                abif_to_dram.wdata = 32'b0;
                abif_to_dram.wstrb = 4'b0;
                abif_to_dram.wlast = 1'b0;
                abif_to_dram.wvalid = 1'b0;
                abif_to_dram.bready = 1'b0;
                abif_to_dram.arid = 4'b0;
                abif_to_dram.araddr = 32'b0;
                abif_to_dram.arlen = 8'b0;
                abif_to_dram.arsize = 3'b0;
                abif_to_dram.arburst = 2'b0;
                abif_to_dram.arlock = 1'b0;
                abif_to_dram.arcache = 4'b0;
                abif_to_dram.arprot = 3'b0;
                abif_to_dram.arqos = 4'b0;
                abif_to_dram.arvalid = 1'b0;
                abif_to_dram.rready = 1'b0;
            end
        endcase
    end else begin
        abif_to_dram.awid = 4'b0;
        abif_to_dram.awaddr = 32'b0;
        abif_to_dram.awlen = 8'b0;
        abif_to_dram.awsize = 3'b0;
        abif_to_dram.awburst = 2'b0;
        abif_to_dram.awlock = 1'b0;
        abif_to_dram.awcache = 4'b0;
        abif_to_dram.awprot = 3'b0;
        abif_to_dram.awqos = 4'b0;
        abif_to_dram.awvalid = 1'b0;
        abif_to_dram.wdata = 32'b0;
        abif_to_dram.wstrb = 4'b0;
        abif_to_dram.wlast = 1'b0;
        abif_to_dram.wvalid = 1'b0;
        abif_to_dram.bready = 1'b0;
        abif_to_dram.arid = 4'b0;
        abif_to_dram.araddr = 32'b0;
        abif_to_dram.arlen = 8'b0;
        abif_to_dram.arsize = 3'b0;
        abif_to_dram.arburst = 2'b0;
        abif_to_dram.arlock = 1'b0;
        abif_to_dram.arcache = 4'b0;
        abif_to_dram.arprot = 3'b0;
        abif_to_dram.arqos = 4'b0;
        abif_to_dram.arvalid = 1'b0;
        abif_to_dram.rready = 1'b0;
    end

    if (ahb_state == BUSY) begin
        casez (ahb_controller)
            // Connect to icache
            2'b00: begin
                abif_to_icache.awready = abif_to_ahb.awready;
                abif_to_icache.wready = abif_to_ahb.wready;
                abif_to_icache.bid = abif_to_ahb.bid;
                abif_to_icache.bresp = abif_to_ahb.bresp;
                abif_to_icache.bvalid = abif_to_ahb.bvalid;
                abif_to_icache.arready = abif_to_ahb.arready;
                abif_to_icache.rid = abif_to_ahb.rid;
                abif_to_icache.rdata = abif_to_ahb.rdata;
                abif_to_icache.rresp = abif_to_ahb.rresp;
                abif_to_icache.rvalid = abif_to_ahb.rvalid;
                abif_to_icache.rlast = abif_to_ahb.rlast;
                abif_to_ahb.awid =  abif_to_icache.awid;
                abif_to_ahb.awaddr = abif_to_icache.awaddr;
                abif_to_ahb.awlen = abif_to_icache.awlen;
                abif_to_ahb.awsize = abif_to_icache.awsize;
                abif_to_ahb.awburst = abif_to_icache.awburst;
                abif_to_ahb.awlock = abif_to_icache.awlock;
                abif_to_ahb.awcache = abif_to_icache.awcache;
                abif_to_ahb.awprot = abif_to_icache.awprot;
                abif_to_ahb.awqos = abif_to_icache.awqos;
                abif_to_ahb.awvalid = abif_to_icache.awvalid;
                abif_to_ahb.wdata = abif_to_icache.wdata;
                abif_to_ahb.wstrb = abif_to_icache.wstrb;
                abif_to_ahb.wlast = abif_to_icache.wlast;
                abif_to_ahb.wvalid = abif_to_icache.wvalid;
                abif_to_ahb.bready = abif_to_icache.bready;
                abif_to_ahb.arid =  abif_to_icache.arid ;
                abif_to_ahb.araddr = abif_to_icache.araddr;
                abif_to_ahb.arlen = abif_to_icache.arlen;
                abif_to_ahb.arsize = abif_to_icache.arsize;
                abif_to_ahb.arburst = abif_to_icache.arburst;
                abif_to_ahb.arlock = abif_to_icache.arlock;
                abif_to_ahb.arcache = abif_to_icache.arcache;
                abif_to_ahb.arprot = abif_to_icache.arprot;
                abif_to_ahb.arqos = abif_to_icache.arqos;
                abif_to_ahb.arvalid = abif_to_icache.arvalid;
                abif_to_ahb.rready = abif_to_icache.rready;
            end
            // Connect to dcache
            2'b01: begin
                abif_to_dcache.awready = abif_to_ahb.awready;
                abif_to_dcache.wready = abif_to_ahb.wready;
                abif_to_dcache.bid = abif_to_ahb.bid;
                abif_to_dcache.bresp = abif_to_ahb.bresp;
                abif_to_dcache.bvalid = abif_to_ahb.bvalid;
                abif_to_dcache.arready = abif_to_ahb.arready;
                abif_to_dcache.rid = abif_to_ahb.rid;
                abif_to_dcache.rdata = abif_to_ahb.rdata;
                abif_to_dcache.rresp = abif_to_ahb.rresp;
                abif_to_dcache.rvalid = abif_to_ahb.rvalid;
                abif_to_dcache.rlast = abif_to_ahb.rlast;
                abif_to_ahb.awid =  abif_to_dcache.awid;
                abif_to_ahb.awaddr = abif_to_dcache.awaddr;
                abif_to_ahb.awlen = abif_to_dcache.awlen;
                abif_to_ahb.awsize = abif_to_dcache.awsize;
                abif_to_ahb.awburst = abif_to_dcache.awburst;
                abif_to_ahb.awlock = abif_to_dcache.awlock;
                abif_to_ahb.awcache = abif_to_dcache.awcache;
                abif_to_ahb.awprot = abif_to_dcache.awprot;
                abif_to_ahb.awqos = abif_to_dcache.awqos;
                abif_to_ahb.awvalid = abif_to_dcache.awvalid;
                abif_to_ahb.wdata = abif_to_dcache.wdata;
                abif_to_ahb.wstrb = abif_to_dcache.wstrb;
                abif_to_ahb.wlast = abif_to_dcache.wlast;
                abif_to_ahb.wvalid = abif_to_dcache.wvalid;
                abif_to_ahb.bready = abif_to_dcache.bready;
                abif_to_ahb.arid =  abif_to_dcache.arid ;
                abif_to_ahb.araddr = abif_to_dcache.araddr;
                abif_to_ahb.arlen = abif_to_dcache.arlen;
                abif_to_ahb.arsize = abif_to_dcache.arsize;
                abif_to_ahb.arburst = abif_to_dcache.arburst;
                abif_to_ahb.arlock = abif_to_dcache.arlock;
                abif_to_ahb.arcache = abif_to_dcache.arcache;
                abif_to_ahb.arprot = abif_to_dcache.arprot;
                abif_to_ahb.arqos = abif_to_dcache.arqos;
                abif_to_ahb.arvalid = abif_to_dcache.arvalid;
                abif_to_ahb.rready = abif_to_dcache.rready;
            end
            // Connect to dma
            2'b10: begin
                abif_to_dma.awready = abif_to_ahb.awready;
                abif_to_dma.wready = abif_to_ahb.wready;
                abif_to_dma.bid = abif_to_ahb.bid;
                abif_to_dma.bresp = abif_to_ahb.bresp;
                abif_to_dma.bvalid = abif_to_ahb.bvalid;
                abif_to_dma.arready = abif_to_ahb.arready;
                abif_to_dma.rid = abif_to_ahb.rid;
                abif_to_dma.rdata = abif_to_ahb.rdata;
                abif_to_dma.rresp = abif_to_ahb.rresp;
                abif_to_dma.rvalid = abif_to_ahb.rvalid;
                abif_to_dma.rlast = abif_to_ahb.rlast;
                abif_to_ahb.awid =  abif_to_dma.awid;
                abif_to_ahb.awaddr = abif_to_dma.awaddr;
                abif_to_ahb.awlen = abif_to_dma.awlen;
                abif_to_ahb.awsize = abif_to_dma.awsize;
                abif_to_ahb.awburst = abif_to_dma.awburst;
                abif_to_ahb.awlock = abif_to_dma.awlock;
                abif_to_ahb.awcache = abif_to_dma.awcache;
                abif_to_ahb.awprot = abif_to_dma.awprot;
                abif_to_ahb.awqos = abif_to_dma.awqos;
                abif_to_ahb.awvalid = abif_to_dma.awvalid;
                abif_to_ahb.wdata = abif_to_dma.wdata;
                abif_to_ahb.wstrb = abif_to_dma.wstrb;
                abif_to_ahb.wlast = abif_to_dma.wlast;
                abif_to_ahb.wvalid = abif_to_dma.wvalid;
                abif_to_ahb.bready = abif_to_dma.bready;
                abif_to_ahb.arid =  abif_to_dma.arid ;
                abif_to_ahb.araddr = abif_to_dma.araddr;
                abif_to_ahb.arlen = abif_to_dma.arlen;
                abif_to_ahb.arsize = abif_to_dma.arsize;
                abif_to_ahb.arburst = abif_to_dma.arburst;
                abif_to_ahb.arlock = abif_to_dma.arlock;
                abif_to_ahb.arcache = abif_to_dma.arcache;
                abif_to_ahb.arprot = abif_to_dma.arprot;
                abif_to_ahb.arqos = abif_to_dma.arqos;
                abif_to_ahb.arvalid = abif_to_dma.arvalid;
                abif_to_ahb.rready = abif_to_dma.rready;
            end
            default: begin
                abif_to_ahb.awid = 4'b0;
                abif_to_ahb.awaddr = 32'b0;
                abif_to_ahb.awlen = 8'b0;
                abif_to_ahb.awsize = 3'b0;
                abif_to_ahb.awburst = 2'b0;
                abif_to_ahb.awlock = 1'b0;
                abif_to_ahb.awcache = 4'b0;
                abif_to_ahb.awprot = 3'b0;
                abif_to_ahb.awqos = 4'b0;
                abif_to_ahb.awvalid = 1'b0;
                abif_to_ahb.wdata = 32'b0;
                abif_to_ahb.wstrb = 4'b0;
                abif_to_ahb.wlast = 1'b0;
                abif_to_ahb.wvalid = 1'b0;
                abif_to_ahb.bready = 1'b0;
                abif_to_ahb.arid = 4'b0;
                abif_to_ahb.araddr = 32'b0;
                abif_to_ahb.arlen = 8'b0;
                abif_to_ahb.arsize = 3'b0;
                abif_to_ahb.arburst = 2'b0;
                abif_to_ahb.arlock = 1'b0;
                abif_to_ahb.arcache = 4'b0;
                abif_to_ahb.arprot = 3'b0;
                abif_to_ahb.arqos = 4'b0;
                abif_to_ahb.arvalid = 1'b0;
                abif_to_ahb.rready = 1'b0;
            end
        endcase
    end else begin
        abif_to_ahb.awid = 4'b0;
        abif_to_ahb.awaddr = 32'b0;
        abif_to_ahb.awlen = 8'b0;
        abif_to_ahb.awsize = 3'b0;
        abif_to_ahb.awburst = 2'b0;
        abif_to_ahb.awlock = 1'b0;
        abif_to_ahb.awcache = 4'b0;
        abif_to_ahb.awprot = 3'b0;
        abif_to_ahb.awqos = 4'b0;
        abif_to_ahb.awvalid = 1'b0;
        abif_to_ahb.wdata = 32'b0;
        abif_to_ahb.wstrb = 4'b0;
        abif_to_ahb.wlast = 1'b0;
        abif_to_ahb.wvalid = 1'b0;
        abif_to_ahb.bready = 1'b0;
        abif_to_ahb.arid = 4'b0;
        abif_to_ahb.araddr = 32'b0;
        abif_to_ahb.arlen = 8'b0;
        abif_to_ahb.arsize = 3'b0;
        abif_to_ahb.arburst = 2'b0;
        abif_to_ahb.arlock = 1'b0;
        abif_to_ahb.arcache = 4'b0;
        abif_to_ahb.arprot = 3'b0;
        abif_to_ahb.arqos = 4'b0;
        abif_to_ahb.arvalid = 1'b0;
        abif_to_ahb.rready = 1'b0;
    end
end

always_ff @(posedge clk) begin
    if (~nrst) begin
        sram_state <= IDLE;
        flash_state <= IDLE;
        dram_state <= IDLE;
        ahb_state <= IDLE;
        sram_controller <= 2'b00;
        flash_controller <= 2'b00;
        dram_controller <= 2'b00;
        ahb_controller <= 2'b00;
    end else begin
        sram_state <= next_sram_state;
        flash_state <= next_flash_state;
        dram_state <= next_dram_state;
        ahb_state <= next_ahb_state;
        sram_controller <= next_sram_controller;
        flash_controller <= next_flash_controller;
        dram_controller <= next_dram_controller;
        ahb_controller <= next_ahb_controller;
    end
end

endmodule