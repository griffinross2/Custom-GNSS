`timescale 1ns/1ns

`include "ram_if.vh"
`include "ahb_bus_if.vh"
`include "common_types.vh"
import common_types_pkg::*;

module axi_sram_controller (
  input logic clk, nrst,
  axi_bus_if.satellite_to_mux abif,
  ram_if.ramctrl ram_if
);

typedef enum logic [1:0] {
  RAM_CTRL_IDLE,
  RAM_CTRL_WRITE_DATA,
  RAM_CTRL_WRITE_RESP,
  RAM_CTRL_READ_DATA
} state_t;

state_t state, next_state;

word_t next_ram_addr;
word_t next_ram_store;
logic [3:0] next_ram_wen;
logic next_ram_ren;
logic [3:0] id, next_id;
word_t next_rdata;
logic next_rvalid;

// Next state logic
always_comb begin
  next_state = state;
  case (state)
    RAM_CTRL_IDLE: begin
      if (abif.awvalid) begin
        next_state = RAM_CTRL_WRITE_DATA;
      end else if (abif.arvalid) begin
        next_state = RAM_CTRL_READ_DATA;
      end
    end

    RAM_CTRL_WRITE_DATA: begin
      if (ram_if.state == RAM_DONE) begin
        next_state = RAM_CTRL_WRITE_RESP;
      end
    end

    RAM_CTRL_WRITE_RESP: begin
      if (abif.bready) begin
        next_state = RAM_CTRL_IDLE;
      end
    end

    RAM_CTRL_READ_DATA: begin
      if (abif.rvalid && abif.rready) begin
        next_state = RAM_CTRL_IDLE;
      end
    end

    default: begin
      next_state = RAM_CTRL_IDLE;
    end
  endcase
end

// State output logic
always_comb begin
  next_ram_addr = ram_if.addr;
  next_ram_store = ram_if.store;
  next_ram_wen = ram_if.wen;
  next_ram_ren = ram_if.ren;
  next_id = id;
  next_rdata = abif.rdata;
  next_rvalid = abif.rvalid;

  abif.awready = 1'b0;
  abif.wready = 1'b0;
  abif.bid = id;
  abif.bresp = 2'b00; // OKAY
  abif.bvalid = 1'b0;
  abif.arready = 1'b0;
  abif.rid = id;
  abif.rresp = 2'b00; // OKAY
  abif.rlast = 1'b1;

  case (state)
    RAM_CTRL_IDLE: begin
      abif.awready = 1'b1;
      abif.arready = 1'b1;
      if (abif.awvalid) begin
        next_ram_addr = {abif.awaddr[31:2], 2'b00} - 32'h1000_0000;
      end else if (abif.arvalid) begin
        next_ram_addr = {abif.araddr[31:2], 2'b00} - 32'h1000_0000;
        next_ram_ren = 1'b1;
      end
    end

    RAM_CTRL_WRITE_DATA: begin
      abif.wready = 1'b1;
      if (abif.wvalid) begin
        // Data has come in, we can actually do the write
        next_ram_store = abif.wdata;
        next_ram_wen = abif.wstrb;
      end

      if (ram_if.state == RAM_DONE) begin
        next_ram_store = '0;
        next_ram_wen = '0;
        next_ram_addr = '0;
      end
    end

    RAM_CTRL_WRITE_RESP: begin
      abif.bvalid = 1'b1;
      abif.bresp = 2'b00; // OKAY
      abif.bid = id;
    end

    RAM_CTRL_READ_DATA: begin
      if (ram_if.state == RAM_DONE) begin
        next_rdata = ram_if.load;
        next_rvalid = 1'b1;
        next_ram_ren = 1'b0;
      end

      if (abif.rvalid && abif.rready) begin
        next_rdata = '0;
        next_rvalid = '0;
      end
    end

    default: begin
    end
  endcase
end

always_ff @(posedge clk) begin
  if (~nrst) begin
    state <= RAM_CTRL_IDLE;
    id <= 4'b0;
    ram_if.addr <= '0;
    ram_if.store <= '0;
    ram_if.wen <= '0;
    ram_if.ren <= '0;
    abif.rdata <= '0;
    abif.rvalid <= '0;
    id <= 4'b0;
  end else begin
    state <= next_state;
    id <= next_id;
    ram_if.addr <= next_ram_addr;
    ram_if.store <= next_ram_store;
    ram_if.wen <= next_ram_wen;
    ram_if.ren <= next_ram_ren;
    abif.rdata <= next_rdata;
    abif.rvalid <= next_rvalid;
    id <= next_id;
  end
end

endmodule
