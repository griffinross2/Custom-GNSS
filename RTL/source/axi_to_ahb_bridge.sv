/**************************/
/* AXI to AHB-Lite Bridge */
/**************************/

`timescale 1ns/1ns

`include "axi_bus_if.vh"
`include "ahb_bus_if.vh"
`include "common_types.vh"
import common_types_pkg::*;

module axi_to_ahb_bridge (
  input logic clk, nrst,
  axi_bus_if.satellite_to_mux axi,
  ahb_bus_if.controller_to_mux ahb
);

typedef enum logic [1:0] {
  TRANSFER_IDLE,
  TRANSFER_WRITE_DATA,
  TRANSFER_WRITE_RESP,
  TRANSFER_READ_DATA
} state_t;

state_t state, next_state;
word_t axi_addr, next_axi_addr;
word_t axi_data, next_axi_data;
logic [3:0] axi_id, next_axi_id;

// Next state logic
always_comb begin
  next_state = state;
  
  case (state)
    TRANSFER_IDLE: begin
      if (axi.arvalid && ahb.hready) begin
        next_state = TRANSFER_READ_DATA;
      end else if (axi.awvalid) begin
        next_state = TRANSFER_WRITE_DATA;
      end
    end

    TRANSFER_WRITE_DATA: begin
      if (axi.wvalid && ahb.hready) begin
        next_state = TRANSFER_WRITE_RESP;
      end
    end

    TRANSFER_WRITE_RESP: begin
      if (axi.bready && axi.bvalid) begin
        next_state = TRANSFER_IDLE;
      end
    end

    TRANSFER_READ_DATA: begin
      if (axi.rready && axi.rvalid) begin
        next_state = TRANSFER_IDLE;
      end
    end

    default: begin
      next_state = TRANSFER_IDLE;
    end
  endcase
end

// State outputs
always_comb begin
  axi.arready = 1'b1;

  axi.rlast = 1'b1;
  axi.rid = '0;
  axi.rdata = '0;
  axi.rresp = '0;
  axi.rvalid = '0;
  
  axi.awready = 1'b1;
  
  axi.wready = 1'b1;

  axi.bid = '0;
  axi.bresp = '0;
  axi.bvalid = '0;

  ahb.haddr = '0;
  ahb.hburst = '0;
  ahb.hsize = 2'b10;
  ahb.htrans = HTRANS_IDLE;
  ahb.hwdata = '0;
  ahb.hwrite = 1'b0;

  next_axi_addr = axi_addr;
  next_axi_data = axi_data;
  next_axi_id = axi_id;

  case (state)
    TRANSFER_IDLE: begin
      if (axi.arvalid) begin
        if (~ahb.hready) begin
          // HREADY must be high to proceed
          axi.arready = 1'b0;
        end
        ahb.haddr = axi.araddr;
        ahb.htrans = HTRANS_NONSEQ;
        next_axi_id = axi.arid;
      end else if (axi.awvalid) begin
        // Wait until data phase to start write, as we need the strobe
        next_axi_addr = axi.awaddr;
      end
    end

    TRANSFER_WRITE_DATA: begin
      if (~ahb.hready) begin
        // HREADY must be high to proceed
        axi.wready = 1'b0;
      end else begin
        axi.wready = 1'b1;
      end

      if (axi.wvalid) begin
        // Take the data and send the address phase
        ahb.haddr = axi_addr;
        ahb.htrans = HTRANS_NONSEQ;
        ahb.hwrite = 1'b1;
        
        // Strobe to size
        case (axi.wstrb)
          4'b0001: ahb.hsize = 2'b00;
          4'b0010: ahb.hsize = 2'b00;
          4'b0100: ahb.hsize = 2'b00;
          4'b1000: ahb.hsize = 2'b00;
          4'b0011: ahb.hsize = 2'b01;
          4'b1100: ahb.hsize = 2'b01;
          4'b1111: ahb.hsize = 2'b10;
          default: ahb.hsize = 2'b10;
        endcase

        next_axi_data = axi.wdata;
        next_axi_id = axi.awid;
      end
    end

    TRANSFER_WRITE_RESP: begin
      // Send data
      ahb.hwdata = axi_data;

      if (~ahb.hready) begin
        // HREADY must be high to proceed
        axi.bvalid = 1'b0;
      end else begin
        axi.bvalid = 1'b1;
        axi.bresp = ahb.hresp ? 2'b01 : 2'b00;
        axi.bid = axi_id;
      end
    end

    TRANSFER_READ_DATA: begin
      if (~ahb.hready) begin
        // HREADY must be high to proceed
        axi.rvalid = 1'b0;
      end else begin
        axi.rvalid = 1'b1;
        axi.rdata = ahb.hrdata;
        axi.rresp = ahb.hresp ? 2'b01 : 2'b00;
        axi.rid = axi_id;
      end
    end

    default: begin
    end
  endcase
end

always_ff @(posedge clk) begin
  if (!nrst) begin
    state <= TRANSFER_IDLE;
    axi_addr <= '0;
    axi_data <= '0;
    axi_id <= '0;
  end else begin
    state <= next_state;
    axi_addr <= next_axi_addr;
    axi_data <= next_axi_data;
    axi_id <= next_axi_id;
  end
end

endmodule