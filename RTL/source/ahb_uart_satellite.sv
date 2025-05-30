/********************************/
/*  AHB-Lite Bus Default Satellite  */
/********************************/
`timescale 1ns/1ns

`include "ahb_bus_if.vh"

`include "common_types.vh"
import common_types_pkg::*;

module ahb_uart_satellite #(
    parameter BASE_ADDR = 32'h2002_0000
)
(
    input logic clk, nrst,
    input logic rxd,    // Receive data
    output logic txd,   // Transmit data
    output logic rxi,   // Receive interrupt
    ahb_bus_if.satellite_to_mux abif
);

    logic [15:0] bit_period;
    logic [7:0] tx_data;
    logic [7:0] rx_data;
    logic tx_busy, tx_done;
    logic rx_busy, rx_done;
    logic tx_start;

    uart_tx uart_tx_inst (
        .clk(clk), 
        .nrst(nrst),
        .bit_period(bit_period),
        .data(tx_data),
        .start(tx_start),
        .serial_out(txd),
        .tx_busy(tx_busy),
        .tx_done(tx_done)
    );
    
    uart_rx uart_rx_inst (
        .clk(clk),
        .nrst(nrst),
        .bit_period(bit_period),
        .serial_in(rxd),
        .data(rx_data),
        .rx_busy(rx_busy),
        .rx_done(rx_done)
    );

    /********************************************/
    /*              Register File               */
    /* 0x00: CFGR - Configuration Register - RW */
    /*       - Bits [16:0]: Baud Rate Div       */
    /* 0x04: TXDR - Transmit Data Register - RW */
    /*       - Bits [7:0]: Data to send         */
    /* 0x08: RXDR - Receive Data Register  - RO */
    /*       - Bits [7:0]: Data received        */
    /* 0x0C: TXSR - TX Status Register     - RW */
    /*       - Bits [0]: TX Busy                */
    /*       - Bits [1]: TX Done                */
    /*       - Bits [2]: RX Busy                */
    /*       - Bits [3]: RX Done                */
    /********************************************/
    // Registers
    word_t cfgr, txdr, rxdr, sr;
    
    // Writeable registers next states
    logic [15:0] cfgr_n;
    logic [7:0] txdr_n;
    logic sr_tx_done;
    logic sr_rx_done;

    // Register AHB signals
    logic [1:0] hsize_reg;
    always_ff @(posedge clk) begin
        if (~nrst) begin
            hsize_reg <= 2'b00;
        end else begin
            hsize_reg <= abif.hsize;
        end
    end

    // Address decoder
    logic [2:0] wen;
    logic [2:0] wen_n;
    always_ff @(posedge clk) begin
        if (~nrst) begin
            wen <= 3'b00;
        end else begin
            wen <= wen_n;
        end
    end

    logic [3:0] ren;

    always_comb begin
        // Default response
        abif.hreadyout = 1'b1;
        abif.hresp = 1'b0;

        // Decoder outputs
        wen_n = 3'b00;
        ren = 4'b0000;

        if (abif.hsel && abif.hready) begin
            if (abif.htrans == HTRANS_NONSEQ) begin
                if (abif.hwrite) begin
                    casez (abif.haddr)
                        BASE_ADDR + 32'h00: begin
                            // Configuration Register
                            wen_n = 3'b001;
                        end
                        BASE_ADDR + 32'h04: begin
                            // Transmit Data Register
                            wen_n = 3'b010;
                        end
                        BASE_ADDR + 32'h0C: begin
                            // Status Register
                            wen_n = 3'b100;
                        end
                        default: begin
                            // Invalid address, set error response
                            abif.hresp = 1'b1;
                        end
                    endcase
                end else begin
                    casez (abif.haddr)
                        BASE_ADDR + 32'h00: begin
                            // Configuration Register
                            ren = 4'b0001;
                        end
                        BASE_ADDR + 32'h04: begin
                            // Transmit Data Register
                            ren = 4'b0010;
                        end
                        BASE_ADDR + 32'h08: begin
                            // Receive Data Register
                            ren = 4'b0100;
                        end
                        BASE_ADDR + 32'h0C: begin
                            // Status Register
                            ren = 4'b1000;
                        end
                        default: begin
                            // Invalid address, set error response
                            abif.hresp = 1'b1;
                        end
                    endcase
                end
            end else if (abif.htrans != HTRANS_IDLE) begin
                // Other modes not supported
                abif.hresp = 1'b1;
            end
        end
    end

    // Writeable registers
    always_ff @(posedge clk) begin
        if (~nrst) begin
            cfgr <= '0;
            txdr <= '0;
            sr_tx_done <= 1'b0;
            sr_rx_done <= 1'b0;
        end else begin
            cfgr <= {16'd0, cfgr_n};
            txdr <= {24'd0, txdr_n}; 
            sr_tx_done <= tx_done ? 1'b1 : ((wen[2] & abif.hwdata[1]) ? 1'b0 : sr_tx_done); // Set from hardware, clear on write of a 1
            sr_rx_done <= rx_done ? 1'b1 : ((wen[2] & abif.hwdata[3]) ? 1'b0 : sr_rx_done); // Set from hardware, clear on write of a 1
        end
    end

    assign bit_period = cfgr[15:0];
    assign tx_data = txdr[7:0];

    // Readable registers
    assign rxdr = {24'd0, rx_data};
    assign sr = {28'd0, sr_rx_done, rx_busy, sr_tx_done, tx_busy};

    // Write data to registers
    always_comb begin
        // Default to no change
        cfgr_n = cfgr[15:0];
        txdr_n = txdr[7:0];

        // Write data to registers based on write enable signals
        if (wen[0]) begin
            if (hsize_reg == 2'b0) begin
                // Byte write
                cfgr_n = {cfgr[15:8], abif.hwdata[7:0]};
            end else begin
                // Other writes
                cfgr_n = abif.hwdata[15:0];
            end
        end else if (wen[1]) begin
            txdr_n = abif.hwdata[7:0];
        end
    end

    // On a write to the TXDR register, additionally start tx
    always_ff @(posedge clk) begin
        if (~nrst) begin
            tx_start <= 1'b0;
        end else if (wen[1]) begin
            tx_start <= 1'b1;
        end else if (tx_busy) begin
            tx_start <= 1'b0;
        end
    end

    // Read data from registers
    logic [31:0] hrdata_n;
    always_ff @(posedge clk) begin
        if (~nrst) begin
            abif.hrdata <= '0;
        end else begin
            abif.hrdata <= hrdata_n;
        end
    end

    always_comb begin
        // Default to 0
        hrdata_n = '0;

        // Read data from registers based on read enable signals
        case (ren)
            4'b0001: begin
                hrdata_n = cfgr;
            end
            4'b0010: begin
                hrdata_n = txdr;
            end
            4'b0100: begin
                hrdata_n = rxdr;
            end
            4'b1000: begin
                hrdata_n = sr;
            end
            default: begin
                hrdata_n = '0;
            end
        endcase
    end

    // Interrupt generation
    assign rxi = sr_rx_done;

endmodule