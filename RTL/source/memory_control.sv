`timescale 1ns/1ns

`include "ram_if.vh"
`include "ahb_bus_if.vh"
`include "common_types.vh"
import common_types_pkg::*;

module memory_control (
  input logic clk, nrst,
  ahb_bus_if.satellite_to_mux ahb_bus_if,
  ram_if.ramctrl ram_if
);

  word_t haddr_reg;
  logic hsel_reg;
  htrans_t htrans_reg;
  logic [1:0] hsize_reg;
  logic hwrite_reg;

  always_ff @(posedge clk) begin
    if (~nrst) begin
      haddr_reg <= '0;
      hsel_reg <= 1'b0;
      htrans_reg <= HTRANS_IDLE;
      hsize_reg <= 2'b00;
      hwrite_reg <= 1'b0;
    end else if (ahb_bus_if.hready) begin
      haddr_reg <= ahb_bus_if.haddr;
      hsel_reg <= ahb_bus_if.hsel;
      htrans_reg <= ahb_bus_if.htrans;
      hsize_reg <= ahb_bus_if.hsize;
      hwrite_reg <= ahb_bus_if.hwrite;
    end
  end

  always_comb begin
    // Default to nothing
    ram_if.ren = 1'b0;
    ram_if.wen = 4'b0;

    // Default to ready/ok response
    ahb_bus_if.hreadyout = 1'b1;
    ahb_bus_if.hresp = 1'b0;

    // Data/address lines
    ram_if.store = ahb_bus_if.hwdata;
    ahb_bus_if.hrdata = ram_if.load;
    ram_if.addr = haddr_reg;

    if (hsel_reg) begin
      // Set the ready signal to indicate the operation is complete
      if (ram_if.state == RAM_DONE) begin
        ahb_bus_if.hreadyout = 1'b1;
      end else begin
        ahb_bus_if.hreadyout = 1'b0;
      end
      
      case (htrans_reg)
        HTRANS_IDLE: begin
          // No action needed
        end

        HTRANS_NONSEQ, HTRANS_SEQ: begin
          // Read or write operation
          if (hwrite_reg) begin
            // Write operation
            case (hsize_reg)
              2'b00: begin
                // Byte strobe
                case (haddr_reg[1:0])
                  2'b00: ram_if.wen = 4'b0001; // Byte 0
                  2'b01: ram_if.wen = 4'b0010; // Byte 1
                  2'b10: ram_if.wen = 4'b0100; // Byte 2
                  2'b11: ram_if.wen = 4'b1000; // Byte 3
                endcase
              end
              2'b01: begin
                // Halfword strobe
                case (haddr_reg[1])
                  1'b0: ram_if.wen = 4'b0011; // Halfword 0
                  1'b1: ram_if.wen = 4'b1100; // Halfword 1
                endcase
              end
              2'b10: ram_if.wen = 4'b1111; // Word
              default: begin
                // Invalid size, return error response
                ahb_bus_if.hresp = 1'b1;
                ahb_bus_if.hreadyout = 1'b1;
              end
            endcase
          end else begin
            // Read operation
            ram_if.ren = 1'b1;
          end
        end

        default: begin
          // Invalid transaction, return error response
          ahb_bus_if.hresp = 1'b1;
          ahb_bus_if.hreadyout = 1'b1;
        end
      endcase
    end
  end

endmodule
