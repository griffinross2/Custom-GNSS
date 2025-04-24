/*******************************/
/*   Top-level System module   */
/* Contains the CPU and memory */
/*******************************/
`timescale 1ns/1ns

`include "common_types.vh"
import common_types_pkg::*;
`include "ram_dump_if.vh"
`include "ahb_controller_if.vh"
`include "ahb_bus_if.vh"
`include "ram_if.vh"

module system (
    input logic clk, nrst,
    input logic rxd,
    output logic txd,
    output logic halt,
    ram_dump_if.tb cpu_ram_debug_if
);

// Interrupt lines
logic [31:0] interrupt_in_sync;
logic uart_rx_int;

always_comb begin
    interrupt_in_sync = 32'b0;
    interrupt_in_sync[16] = uart_rx_int;
end

// Interfaces
ahb_controller_if debug_controller_if();

ahb_bus_if multiplexor_abif();
ahb_bus_if debug_abif();
ahb_bus_if cpu_abif();
ahb_bus_if def_abif();
ahb_bus_if ram_abif();
ahb_bus_if uart_abif();

ram_if ram_if();

// Debug AHB interface
assign debug_controller_if.iread = cpu_ram_debug_if.iren;
assign debug_controller_if.dread = cpu_ram_debug_if.dren;
always_comb begin
    casez (cpu_ram_debug_if.dwen)
        4'b0001, 4'b0010, 4'b0100, 4'b1000: debug_controller_if.dwrite = 2'b01;
        4'b1100, 4'b0011: debug_controller_if.dwrite = 2'b10;
        4'b1111: debug_controller_if.dwrite = 2'b11;
        default: debug_controller_if.dwrite = 2'b00; // No write
    endcase
end
assign debug_controller_if.iaddr = cpu_ram_debug_if.iaddr;
assign debug_controller_if.daddr = cpu_ram_debug_if.daddr;
assign debug_controller_if.dstore = cpu_ram_debug_if.dstore;

assign cpu_ram_debug_if.iwait = ~debug_controller_if.ihit;
assign cpu_ram_debug_if.dwait = ~debug_controller_if.dhit;
assign cpu_ram_debug_if.iload = debug_controller_if.iload;
assign cpu_ram_debug_if.dload = debug_controller_if.dload;

// Connect control from TB or CPU to AHB bus
assign multiplexor_abif.haddr = cpu_ram_debug_if.override_ctrl ? debug_abif.haddr : cpu_abif.haddr;
assign multiplexor_abif.hburst = cpu_ram_debug_if.override_ctrl ? debug_abif.hburst : cpu_abif.hburst;
assign multiplexor_abif.hsize = cpu_ram_debug_if.override_ctrl ? debug_abif.hsize : cpu_abif.hsize;
assign multiplexor_abif.htrans = cpu_ram_debug_if.override_ctrl ? debug_abif.htrans : cpu_abif.htrans;
assign multiplexor_abif.hwdata = cpu_ram_debug_if.override_ctrl ? debug_abif.hwdata : cpu_abif.hwdata;
assign multiplexor_abif.hwrite = cpu_ram_debug_if.override_ctrl ? debug_abif.hwrite : cpu_abif.hwrite;

// Connect AHB bus to TB and CPU
assign cpu_abif.hrdata = multiplexor_abif.hrdata;
assign cpu_abif.hready = multiplexor_abif.hready;
assign cpu_abif.hresp = multiplexor_abif.hresp;
assign debug_abif.hrdata = multiplexor_abif.hrdata;
assign debug_abif.hready = multiplexor_abif.hready;
assign debug_abif.hresp = multiplexor_abif.hresp;

// Debug AHB controller
ahb_controller debug_controller (
    .clk(clk),
    .nrst(nrst),
    .amif(debug_controller_if),
    .abif(debug_abif)
);

// CPU
cpu cpu_inst(
    .clk(clk),
    .nrst(nrst),
    .interrupt_in_sync(interrupt_in_sync),
    .halt(halt),
    .abif(cpu_abif)
);

// AHB multiplexor
ahb_multiplexor ahb_mux_inst (
    .clk(clk),
    .nrst(nrst),
    .abif_to_controller(multiplexor_abif),
    .abif_to_def(def_abif),
    .abif_to_ram(ram_abif),
    .abif_to_uart(uart_abif)
);

// AHB default satellite
ahb_default_satellite ahb_satellite_def (
    .clk(clk),
    .nrst(nrst),
    .abif(def_abif)
);

// Memory controller
memory_control memory_control_inst (
    .clk(clk),
    .nrst(nrst),
    .ahb_bus_if(ram_abif),
    .ram_if(ram_if)
);

// UART
ahb_uart_satellite uart_inst (
    .clk(clk),
    .nrst(nrst),
    .rxd(rxd),
    .txd(txd),
    .rxi(uart_rx_int),
    .abif(uart_abif)
);

// Shared Instruction-Data RAM
ram ram_inst (
    .clk(clk),
    .nrst(nrst),
    .ram_if(ram_if)
);

endmodule