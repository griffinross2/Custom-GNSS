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
    ahb_bus_if.mux_to_satellite abif_to_uart,
    ahb_bus_if.mux_to_satellite abif_to_gnss
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
        abif_to_uart.hsel = 1'b0;
        abif_to_gnss.hsel = 1'b0;

        // UART address range = 0x2002_0000 to 0x2002_000C
        if (abif_to_controller.htrans != HTRANS_IDLE && abif_to_controller.haddr >= 32'h2002_0000 && abif_to_controller.haddr < 32'h2002_0010) begin
            abif_to_uart.hsel = 1'b1;
            sel_i = 1;
        end else if (abif_to_controller.htrans != HTRANS_IDLE && abif_to_controller.haddr >= 32'h2004_0000 && abif_to_controller.haddr < 32'h2004_0800) begin
            abif_to_gnss.hsel = 1'b1;
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

            1: begin // UART satellite
                abif_to_controller.hrdata = abif_to_uart.hrdata;
                readyout = abif_to_uart.hreadyout;
                abif_to_controller.hresp = abif_to_uart.hresp;
            end

            2: begin // GNSS satellite
                abif_to_controller.hrdata = abif_to_gnss.hrdata;
                readyout = abif_to_gnss.hreadyout;
                abif_to_controller.hresp = abif_to_gnss.hresp;
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

        abif_to_uart.hwdata = abif_to_controller.hwdata;
        abif_to_uart.haddr = abif_to_controller.haddr;
        abif_to_uart.hburst = abif_to_controller.hburst;
        abif_to_uart.hsize = abif_to_controller.hsize;
        abif_to_uart.htrans = abif_to_controller.htrans;
        abif_to_uart.hwrite = abif_to_controller.hwrite;
        abif_to_uart.hready = readyout;

        abif_to_gnss.hwdata = abif_to_controller.hwdata;
        abif_to_gnss.haddr = abif_to_controller.haddr;
        abif_to_gnss.hburst = abif_to_controller.hburst;
        abif_to_gnss.hsize = abif_to_controller.hsize;
        abif_to_gnss.htrans = abif_to_controller.htrans;
        abif_to_gnss.hwrite = abif_to_controller.hwrite;
        abif_to_gnss.hready = readyout;
    end
endmodule