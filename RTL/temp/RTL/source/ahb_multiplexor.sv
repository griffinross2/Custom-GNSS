/******************************/
/*  AHB-Lite Bus Multiplexor  */
/******************************/
`timescale 1ns/1ns

`include "ahb_bus_if.vh"

`include "common_types.vh"
import common_types_pkg::*;

module ahb_multiplexor (
    input logic clk, nrst,
    ahb_bus_if.mux_to_controller abif_to_controller,
    ahb_bus_if.mux_to_satellite abif_to_def,
    ahb_bus_if.mux_to_satellite abif_to_ram,
    ahb_bus_if.mux_to_satellite abif_to_uart
);

    integer sel_i;
    integer sel_i_reg;

    word_t haddr_reg;
    htrans_t htrans_reg;
    logic readyout;

    always_ff @(posedge clk) begin
        if (~nrst) begin
            haddr_reg <= '0;
            htrans_reg <= HTRANS_IDLE;
            sel_i_reg <= 0;
        end else begin
            // Latch the address from the controller interface if the current satellite is ready
            haddr_reg <= readyout ? abif_to_controller.haddr : haddr_reg;
            htrans_reg <= readyout ? abif_to_controller.htrans : htrans_reg;
            sel_i_reg <= readyout ? sel_i : sel_i_reg;
        end
    end

    
    always_comb begin
        // Default satellite
        sel_i = 0;

        // Select the appropriate satellite based on the address
        abif_to_def.hsel = 1'b0;
        abif_to_ram.hsel = 1'b0;
        abif_to_uart.hsel = 1'b0;

        // RAM address range = 0x0000_0000 to 0x0000_FFFF
        if (abif_to_controller.htrans != HTRANS_IDLE && abif_to_controller.haddr < 32'h0001_0000) begin
            abif_to_ram.hsel = 1'b1;
            sel_i = 1;
        // UART address range = 0x0002_0000 to 0x0002_000C
        end else if (abif_to_controller.htrans != HTRANS_IDLE && abif_to_controller.haddr >= 32'h0002_0000 && abif_to_controller.haddr < 32'h0002_0010) begin
            abif_to_uart.hsel = 1'b1;
            sel_i = 2;
        end else begin
            abif_to_def.hsel = 1'b1;
            sel_i = 0;
        end
    end

    always_comb begin
        casez(sel_i_reg)
            0: begin // Default satellite
                abif_to_controller.hrdata = abif_to_def.hrdata;
                readyout = abif_to_def.hreadyout;
                abif_to_controller.hresp = abif_to_def.hresp;
            end

            1: begin // RAM satellite
                abif_to_controller.hrdata = abif_to_ram.hrdata;
                readyout = abif_to_ram.hreadyout;
                abif_to_controller.hresp = abif_to_ram.hresp;
            end

            2: begin // UART satellite
                abif_to_controller.hrdata = abif_to_uart.hrdata;
                readyout = abif_to_uart.hreadyout;
                abif_to_controller.hresp = abif_to_uart.hresp;
            end

            default: begin // Invalid address, default to default satellite
                abif_to_controller.hrdata = abif_to_def.hrdata;
                readyout = abif_to_def.hreadyout;
                abif_to_controller.hresp = abif_to_def.hresp;
            end
        endcase
    
        // Send ready output
        abif_to_controller.hready = readyout;
    end

    always_comb begin

        // Send signals to all satellites
        abif_to_def.hwdata = abif_to_controller.hwdata;
        abif_to_def.haddr = abif_to_controller.haddr;
        abif_to_def.hburst = abif_to_controller.hburst;
        abif_to_def.hsize = abif_to_controller.hsize;
        abif_to_def.htrans = abif_to_controller.htrans;
        abif_to_def.hwrite = abif_to_controller.hwrite;
        abif_to_def.hready = readyout;

        abif_to_ram.hwdata = abif_to_controller.hwdata;
        abif_to_ram.haddr = abif_to_controller.haddr;
        abif_to_ram.hburst = abif_to_controller.hburst;
        abif_to_ram.hsize = abif_to_controller.hsize;
        abif_to_ram.htrans = abif_to_controller.htrans;
        abif_to_ram.hwrite = abif_to_controller.hwrite;
        abif_to_ram.hready = readyout;

        abif_to_uart.hwdata = abif_to_controller.hwdata;
        abif_to_uart.haddr = abif_to_controller.haddr;
        abif_to_uart.hburst = abif_to_controller.hburst;
        abif_to_uart.hsize = abif_to_controller.hsize;
        abif_to_uart.htrans = abif_to_controller.htrans;
        abif_to_uart.hwrite = abif_to_controller.hwrite;
        abif_to_uart.hready = readyout;
    end
endmodule