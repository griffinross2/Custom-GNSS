/*************************/
/*  AXI4 Bus Controller  */
/*************************/
`timescale 1ns/1ns

`include "axi_controller_if.vh"
`include "axi_bus_if.vh"

`include "common_types.vh"
import common_types_pkg::*;

module axi_controller (
    input logic clk, nrst,
    axi_controller_if.axi_controller amif,
    axi_bus_if.controller_to_mux abif
);

typedef enum logic [2:0] {
    TRANSFER_IDLE,
    TRANSFER_READ_ADDR,
    TRANSFER_READ_DATA,
    TRANSFER_WRITE_ADDR,
    TRANSFER_WRITE_DATA,
    TRANSFER_WRITE_RESP
} transfer_state_t;

transfer_state_t state, next_state;
logic [7:0] read_counter, next_read_counter;

// Controller side next signals
logic next_ready;
word_t next_load;

// Bus side next signals
word_t next_awaddr;
logic next_awvalid;

word_t next_wdata;
logic [3:0] next_wstrb;
logic next_wvalid;

logic next_bready;

word_t next_araddr;
logic next_arvalid;

logic next_rready;

always_ff @(posedge clk) begin
    if (~nrst) begin
        state <= TRANSFER_IDLE;
        read_counter <= '0;
        amif.ready <= 1'b0;
        amif.load <= '0;
        abif.awaddr <= '0;
        abif.awvalid <= 1'b0;
        abif.wdata <= '0;
        abif.wstrb <= '0;
        abif.wvalid <= 1'b0;
        abif.bready <= 1'b0;
        abif.araddr <= '0;
        abif.arvalid <= 1'b0;
        abif.rready <= 1'b0;
    end else begin
        state <= next_state;
        read_counter <= next_read_counter;
        amif.ready <= next_ready;
        amif.load <= next_load;
        abif.awaddr <= next_awaddr;
        abif.awvalid <= next_awvalid;
        abif.wdata <= next_wdata;
        abif.wstrb <= next_wstrb;
        abif.wvalid <= next_wvalid;
        abif.bready <= next_bready;
        abif.araddr <= next_araddr;
        abif.arvalid <= next_arvalid;
        abif.rready <= next_rready;
    end
end

// Next state logic
always_comb begin
    // Default to no change
    next_state = state;

    casez(state)
        TRANSFER_IDLE: begin
            if (amif.read) begin
                // Read with priority
                next_state = TRANSFER_READ_ADDR;
            end else if (|amif.write) begin
                // Write
                next_state = TRANSFER_WRITE_ADDR;
            end
        end

        TRANSFER_READ_ADDR: begin
            if (abif.arready) begin
                // Satellite is ready to accept transaction
                next_state = TRANSFER_READ_DATA;
            end
        end

        TRANSFER_READ_DATA: begin
            if (amif.done) begin
                // Controller is proceeding
                next_state = TRANSFER_IDLE;
            end
        end

        TRANSFER_WRITE_ADDR: begin
            if (abif.awready) begin
                // Satellite is ready to accept transaction
                next_state = TRANSFER_WRITE_DATA;
            end
        end

        TRANSFER_WRITE_DATA: begin
            if (abif.wready) begin
                // Satellite is ready to accept data
                next_state = TRANSFER_WRITE_RESP;
            end
        end

        TRANSFER_WRITE_RESP: begin
            if (amif.done) begin
                // Controller is proceeding
                next_state = TRANSFER_IDLE;
            end
        end

        default: begin
        end
    endcase
end

// State output logic
always_comb begin
    // Default to no change
    next_read_counter = read_counter;

    next_ready = amif.ready;
    next_load = amif.load;

    next_awaddr = abif.awaddr;
    next_awvalid = abif.awvalid;
    next_wdata = abif.wdata;
    next_wstrb = abif.wstrb;
    next_wvalid = abif.wvalid;
    next_bready = abif.bready;
    next_araddr = abif.araddr;
    next_arvalid = abif.arvalid;
    next_rready = abif.rready;

    abif.arid = '0;         // ID
    abif.arlen = 8'd0;      // 1 data transfer
    abif.arsize = 3'd2;     // 4-byte word
    abif.arburst = 2'b00;   // Fixed (no burst)
    abif.arlock = 1'b0;     // Normal access
    abif.arcache = 4'b0000; // Normal access
    abif.arprot = 3'b000;   // Normal access
    abif.arqos = 4'b0000;   // No QoS

    abif.awid = 4'b0000;    // ID
    abif.awlen = 8'd0;      // 1 data transfer
    abif.awsize = 3'd2;     // 4-byte word
    abif.awburst = 2'b00;   // Fixed (no burst)
    abif.awlock = 1'b0;     // Normal access
    abif.awcache = 4'b0000; // Normal access
    abif.awprot = 3'b000;   // Normal access
    abif.awqos = 4'b0000;   // No QoS

    abif.wlast = 1'b1;      // Always last data transfer

    casez(state)
        TRANSFER_IDLE: begin
            next_ready = 1'b0;
            next_arvalid = 1'b0;
            next_awvalid = 1'b0;
            next_wvalid = 1'b0;
            next_bready = 1'b0;
            next_rready = 1'b0;

            if (amif.read) begin
                // Read address
                next_araddr = amif.addr;
                next_arvalid = 1'b1;
            end else if (|amif.write) begin
                // Write address
                next_awaddr = amif.addr;
                next_awvalid = 1'b1;
            end
        end

        TRANSFER_READ_ADDR: begin
            if (abif.arready) begin
                // Satellite is ready to accept transaction
                next_arvalid = 1'b0;
                next_rready = 1'b1;
                next_read_counter = '0;
            end
        end

        TRANSFER_READ_DATA: begin
            if (abif.rvalid && (abif.rlast | read_counter == abif.arlen)) begin
                // Read data is valid
                next_load = abif.rdata;
                next_rready = 1'b0;
                next_ready = 1'b1;
            end else if (abif.rvalid) begin
                // For now we hardcode only 1 read length so ignore
                // any extra data

                // Increment read counter
                next_read_counter = read_counter + 8'd1;
            end

            if (amif.done) begin
                // Transaction is done
                next_ready = 1'b0;
            end
        end

        TRANSFER_WRITE_ADDR: begin
            if (abif.awready) begin
                // Satellite is ready to accept transaction
                next_awvalid = 1'b0;
                next_wvalid = 1'b1;
                next_wdata = amif.store;

                case (amif.write)
                    2'b01: begin
                        case (amif.addr[1:0])
                            2'b00: next_wstrb = 4'b0001; // Byte
                            2'b01: next_wstrb = 4'b0010; // Byte
                            2'b10: next_wstrb = 4'b0100; // Byte
                            2'b11: next_wstrb = 4'b1000; // Byte
                            default: next_wstrb = 4'b0000; // No write
                        endcase
                    end
                    2'b10: begin
                        case (amif.addr[1:0])
                            2'b00: next_wstrb = 4'b0011; // Halfword
                            2'b01: next_wstrb = 4'b0011; // Halfword
                            2'b10: next_wstrb = 4'b1100; // Halfword
                            2'b11: next_wstrb = 4'b1100; // Halfword
                            default: next_wstrb = 4'b0000; // No write
                        endcase
                    end
                    2'b11: next_wstrb = 4'b1111; // Word
                    default: next_wstrb = 4'b0000; // No write
                endcase
            end
        end

        TRANSFER_WRITE_DATA: begin
            if (abif.wready) begin
                // Write data is accepted
                next_wvalid = 1'b0;
                next_bready = 1'b1;
            end
        end

        TRANSFER_WRITE_RESP: begin
            if (abif.bvalid) begin
                // Write response is valid
                next_bready = 1'b0;
                next_ready = 1'b1;
            end

            if (amif.done) begin
                // Transaction is done
                next_ready = 1'b0;
            end
        end

        default: begin
        end
    endcase
end

endmodule