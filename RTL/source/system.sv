/*******************************/
/*   Top-level System module   */
/* Contains the CPU and memory */
/*******************************/
`timescale 1ns/1ns

`include "common_types.vh"
import common_types_pkg::*;
`include "axi_controller_if.vh"
`include "cache_if.vh"
`include "axi_bus_if.vh"
`include "ahb_bus_if.vh"
`include "ram_if.vh"

module system (
    input logic clk, nrst,
    input logic gnssclk, signal_in,
    input logic rxd,
    output logic txd,
    output logic halt,
    axi_controller_if.axi_controller debug_amif,
    output logic flash_cs,
    inout wire [3:0] flash_dq
);

// Interrupt lines
logic [31:0] interrupt_in_sync;
logic uart_rx_int;

always_comb begin
    interrupt_in_sync = 32'b0;
    interrupt_in_sync[16] = uart_rx_int;
end

// Interfaces
cache_if fetch_amif();
cache_if mem_amif();
axi_controller_if icache_amif();
axi_controller_if dcache_amif();

axi_bus_if multiplexor_abif();
axi_bus_if debug_abif();
axi_bus_if icache_abif();
axi_bus_if dcache_abif();
axi_bus_if sram_abif();
axi_bus_if flash_abif();
axi_bus_if dram_abif();
axi_bus_if ahb_abif();
ahb_bus_if controller_ahb();
ahb_bus_if uart_ahb();
ahb_bus_if gnss_ahb();
ahb_bus_if def_ahb();

ram_if rom_if();
ram_if sram_if();

// Datapath
datapath datapath_inst (
    .clk(clk),
    .nrst(nrst),
    .interrupt_in_sync(interrupt_in_sync),
    .halt(mem_amif.halt),
    .amif_fetch(fetch_amif),
    .amif_mem(mem_amif)
);
assign halt = mem_amif.flushed;

// GNSS
gnss gnss_inst (
    .gnssclk(gnssclk),
    .cpuclk(clk),
    .nrst(nrst),
    .signal_in(signal_in),
    .abif(gnss_ahb)
);

// AXI interconnect
axi_interconnect axi_interconnect_inst (
    .clk(clk),
    .nrst(nrst),
    .abif_to_icache(icache_abif),
    .abif_to_dcache(dcache_abif),
    .abif_to_dma(debug_abif),   // Plug debug into DMA for now
    .abif_to_sram(sram_abif),
    .abif_to_flash(flash_abif),
    .abif_to_dram(dram_abif),
    .abif_to_ahb(ahb_abif)
);

// Debug AXI controller
axi_controller debug_controller (
    .clk(clk),
    .nrst(nrst),
    .amif(debug_amif),
    .abif(debug_abif)
);

// Instruction Cache
icache icache_inst (
    .clk(clk),
    .nrst(nrst),
    .amif(icache_amif),
    .cif(fetch_amif)
);

// always_comb begin
//     fetch_amif.load = icache_amif.load;
//     fetch_amif.ready = icache_amif.ready;
//     icache_amif.done = fetch_amif.done;
//     icache_amif.store = fetch_amif.store;
//     icache_amif.read = fetch_amif.read;
//     icache_amif.write = fetch_amif.write;
//     icache_amif.addr = fetch_amif.addr;
// end

// Data Cache
// dcache dcache_inst (
//     .clk(clk),
//     .nrst(nrst),
//     .amif(dcache_amif),
//     .cif(mem_amif)
// );

always_comb begin
    mem_amif.load = dcache_amif.load;
    mem_amif.ready = dcache_amif.ready;
    dcache_amif.done = mem_amif.done;
    dcache_amif.store = mem_amif.store;
    dcache_amif.read = mem_amif.read;
    dcache_amif.write = mem_amif.write;
    dcache_amif.addr = mem_amif.addr;
    mem_amif.flushed = mem_amif.halt;
end

// AXI ICache controller
axi_controller icache_controller (
    .clk(clk),
    .nrst(nrst),
    .amif(icache_amif),
    .abif(icache_abif)
);

// AXI DCache controller
axi_controller dcache_controller (
    .clk(clk),
    .nrst(nrst),
    .amif(dcache_amif),
    .abif(dcache_abif)
);

// AXI Flash controller
axi_flash_controller flash_controller (
    .clk(clk),
    .nrst(nrst),
    .abif(flash_abif),
    .clk_div(4'd1),
    .flash_cs(flash_cs),
    .flash_dq(flash_dq)
);

// AXI ROM controller
axi_rom_controller rom_controller (
    .clk(clk),
    .nrst(nrst),
    .abif(sram_abif),
    .ram_if(rom_if)
);

// ROM
rom rom_inst (
    .clk(clk),
    .nrst(nrst),
    .ram_if(rom_if)
);

// AXI SRAM controller
axi_sram_controller sram_controller (
    .clk(clk),
    .nrst(nrst),
    .abif(dram_abif),
    .ram_if(sram_if)
);

// SRAM
sram sram_inst (
    .clk(clk),
    .nrst(nrst),
    .ram_if(sram_if)
);

// AXI to AHB-Lite bridge
axi_to_ahb_bridge axi_to_ahb_inst (
    .clk(clk),
    .nrst(nrst),
    .axi(ahb_abif),
    .ahb(controller_ahb)
);

// AHB-Lite Multiplexor
ahb_multiplexor ahb_multiplexor_inst (
    .clk(clk),
    .nrst(nrst),
    .abif_to_controller(controller_ahb),
    .abif_to_def(def_ahb),
    .abif_to_uart(uart_ahb),
    .abif_to_gnss(gnss_ahb)
);

// AHB-Lite UART satellite
ahb_uart_satellite ahb_uart_inst (
    .clk(clk),
    .nrst(nrst),
    .abif(uart_ahb),
    .rxd(rxd),
    .txd(txd),
    .rxi(uart_rx_int)
);

// AHB-Lite default satellite
ahb_default_satellite ahb_default_inst (
    .clk(clk),
    .nrst(nrst),
    .abif(def_ahb)
);

endmodule