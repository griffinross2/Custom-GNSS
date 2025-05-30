/*****************************/
/*  AHB-Lite Bus Controller  */
/*****************************/
`timescale 1ns/1ns

`include "ahb_controller_if.vh"
`include "ahb_bus_if.vh"

`include "common_types.vh"
import common_types_pkg::*;

module ahb_controller (
    input logic clk, nrst,
    ahb_controller_if.ahb_controller amif,
    ahb_bus_if.controller_to_mux abif
);

word_t hwdata_n;

typedef enum logic [1:0] {
    TRANSFER_IDLE,
    TRANSFER_IREAD,
    TRANSFER_DREAD,
    TRANSFER_DWRITE
} transfer_t;

// Transfer in data phase
transfer_t data_transfer;

// Transfer in addr phase (next data phase)
transfer_t addr_transfer;

logic ihit_reg, dhit_reg;
logic ihit_n, dhit_n;
word_t iload_reg, dload_reg;
word_t iload_n, dload_n;

always_ff @(posedge clk) begin
    if (~nrst) begin
        abif.hwdata <= '0;
        data_transfer <= TRANSFER_IDLE;
        iload_reg <= '0;
        dload_reg <= '0;
        ihit_reg <= 1'b0;
        dhit_reg <= 1'b0;
    end else begin
        abif.hwdata <= hwdata_n;
        data_transfer <= abif.hready ? addr_transfer : data_transfer;
        iload_reg <= iload_n;
        dload_reg <= dload_n;
        ihit_reg <= ihit_n;
        dhit_reg <= dhit_n;
    end
end

always_comb begin
    // Default to no change
    hwdata_n = abif.hwdata;
    addr_transfer = data_transfer;
    
    // Output signals
    ihit_n = ihit_reg;
    dhit_n = dhit_reg;
    amif.iload = abif.hrdata;
    amif.dload = abif.hrdata;
    iload_n = iload_reg;
    dload_n = dload_reg;

    abif.haddr = '0;
    abif.hburst = '0;
    abif.hsize = '0;
    abif.htrans = HTRANS_IDLE;
    hwdata_n = amif.dstore;
    abif.hwrite = '0;
    addr_transfer = data_transfer;

    if (abif.hready && nrst) begin
        // Finish the current transaction
        casez (data_transfer)
            TRANSFER_IREAD: begin
                ihit_n = 1'b1;
                dhit_n = 1'b0;
            end
            TRANSFER_DREAD: begin
                dhit_n = 1'b1;
            end
            TRANSFER_DWRITE: begin
                dhit_n = 1'b1;
            end
            default: begin
                // Idle, do nothing
            end
        endcase
    end

    if (nrst) begin
        // Start a new transaction
        if (amif.iread) begin
            abif.haddr = amif.iaddr;
            abif.hburst = 3'b000;
            abif.hsize = 2'b10;
            abif.htrans = HTRANS_NONSEQ;
            abif.hwrite = 1'b0;
            addr_transfer = TRANSFER_IREAD;

            ihit_n = 1'b0;
            dhit_n = 1'b0;
        end else if (amif.dread) begin
            abif.haddr = amif.daddr;
            abif.hburst = 3'b000;
            abif.hsize = 2'b10;
            abif.htrans = HTRANS_NONSEQ;
            abif.hwrite = 1'b0;
            addr_transfer = TRANSFER_DREAD;

            dhit_n = 1'b0;
        end else if (|amif.dwrite) begin
            abif.haddr = amif.daddr;
            abif.hburst = 3'b000;
            casez (amif.dwrite)
                2'b01: abif.hsize = 2'b00; // byte
                2'b10: abif.hsize = 2'b01; // halfword
                2'b11: abif.hsize = 2'b10; // word
                default: abif.hsize = 2'b10;
            endcase
            abif.htrans = HTRANS_NONSEQ;
            abif.hwrite = 1'b1;
            addr_transfer = TRANSFER_DWRITE;

            dhit_n = 1'b0;
        end else begin
            abif.haddr = '0;
            abif.hburst = '0;
            abif.hsize = '0;
            abif.htrans = HTRANS_IDLE;
            abif.hwrite = '0;
            addr_transfer = TRANSFER_IDLE;
        end
    end
end

always_comb begin
    // Hit output
    amif.ihit = ihit_reg;
    amif.dhit = dhit_reg;

    if (abif.hready && nrst) begin
        // Finish the current transaction
        casez (data_transfer)
            TRANSFER_IREAD: begin
                amif.ihit = 1'b1;
            end
            TRANSFER_DREAD: begin
                amif.dhit = 1'b1;
            end
            TRANSFER_DWRITE: begin
                amif.dhit = 1'b1;
            end
            default: begin
                // Idle, do nothing
            end
        endcase
    end
end

endmodule