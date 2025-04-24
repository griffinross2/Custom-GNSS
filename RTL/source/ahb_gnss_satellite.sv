`timescale 1ns/1ns

`include "common_types.vh"
import common_types_pkg::*;
`include "common_gnss_types.vh"
import common_gnss_types_pkg::*;

`include "ahb_bus_if.vh"

module ahb_gnss_satellite (
    input logic clk, hclk, nrst,
    ahb_bus_if.satellite_to_mux abif,
    input logic [31:0] epoch,           // Epoch signal from each channel
    output logic snapshot               // Take snapshot signal to every channel
);

logic cdc_req_hclk, cdc_req_hclk_n; // Cross-domain clock request signal (in hclk domain)
logic cdc_req_clk, cdc_req_clk_n; // Cross-domain clock request signal (in clk domain)
logic cdc_ack_hclk, cdc_ack_hclk_n; // Cross-domain clock acknowledge signal (in hclk domain)
logic cdc_ack_clk, cdc_ack_clk_n; // Cross-domain clock acknowledge signal (in clk domain)

logic cdc_req_sync [2:0]; // Synchronization registers for request signal
logic cdc_ack_sync [2:0]; // Synchronization registers for acknowledge signal

logic [1:0] rsel_hclk, rsel_hclk_n; // Read select signal (in hclk domain)
logic [1:0] rsel_clk, rsel_clk_n; // Read select signal (in clk domain)

logic [1:0] wsel_hclk, wsel_hclk_n; // Read select signal (in hclk domain)
logic [1:0] wsel_clk, wsel_clk_n; // Read select signal (in clk domain)

word_t wdata_clk, wdata_clk_n; // Read data (in clk domain)
word_t wdata_hclk, wdata_hclk_n; // Read data (in hclk domain)

word_t rdata_clk, rdata_clk_n; // Read data (in clk domain)
word_t rdata_hclk, rdata_hclk_n; // Read data (in hclk domain)

logic hreadyout_n;

logic [31:0] epoch_edge; // Register the epoch signal to detect edges
logic snapshot_n; // Snapshot next state
word_t csr, csr_n; // Channel status register for the satellite

// Epoch, snapshot, CSR FF
always_ff @(posedge clk) begin
    if (~nrst) begin
        epoch_edge <= '0;
        snapshot <= '0;
        csr <= '0;
    end else begin
        epoch_edge <= epoch;
        snapshot <= snapshot_n;
        csr <= csr_n;
    end
end

// Bus FF
always_ff @(posedge hclk) begin
    if (~nrst) begin
        abif.hreadyout <= 1'b1;
    end else begin
        abif.hreadyout <= hreadyout_n;
    end
end

// Bus logic
always_comb begin
    rsel_hclk_n = rsel_hclk;
    wsel_hclk_n = wsel_hclk;
    hreadyout_n = abif.hreadyout;

    abif.hrdata = '0;
    
    abif.hresp = '0;

    // Service bus request
    if (abif.hsel && abif.htrans == HTRANS_NONSEQ && abif.hready) begin
        rsel_hclk_n = 2'd0;
        wsel_hclk_n = 2'b0;
        hreadyout_n = 1'b0; // Set hreadyout to 0 when a request is being processed

        if (abif.hwrite) begin
            // Decode the address to determine the read select signal
            case (abif.haddr[9:0])
                10'h000: wsel_hclk_n = 2'd1;
                10'h004: wsel_hclk_n = 2'd2;
                default: wsel_hclk_n = 2'd0; // Default case
            endcase
        end else begin
            // Decode the address to determine the read select signal
            case (abif.haddr[9:0])
                10'h000: rsel_hclk_n = 2'd1;
                10'h004: rsel_hclk_n = 2'd2;
                default: rsel_hclk_n = 2'd0; // Default case
            endcase
        end
    end

    // Get wdata from the bus
    wdata_hclk_n = abif.hwdata;

    // Bus response (if ack is going high, set hreadyout to 1)
    // Also clear rsel/wsel since we are done with the request
    if (~cdc_ack_hclk & cdc_ack_hclk_n) begin
        hreadyout_n = 1'b1;
        rsel_hclk_n = 2'd0;
        wsel_hclk_n = 2'b0;
    end

    // Give bus the data
    if (abif.hreadyout) begin
        abif.hrdata = rdata_hclk;
    end
end

// Request signal generation
always_comb begin
    cdc_req_hclk_n = cdc_req_hclk;

    // If we decided to read or write, send a request to the clk domain
    if (rsel_hclk_n != 2'd0 || wsel_hclk_n != 2'd0) begin
        cdc_req_hclk_n = 1'b1;
    end

    // If an acknowledge signal is received, clear the request signal
    if (cdc_ack_hclk) begin
        cdc_req_hclk_n = 1'b0;
    end
end

// hclk domain
always_ff @(posedge hclk) begin
    if (~nrst) begin
        cdc_req_hclk <= 1'b0;
        cdc_ack_hclk <= 1'b0;
        rdata_hclk <= 1'b0;
        rsel_hclk <= '0;
        wsel_hclk <= '0;
        wdata_hclk <= '0;
    end else begin
        cdc_req_hclk <= cdc_req_hclk_n;
        cdc_ack_hclk <= cdc_ack_hclk_n;
        rdata_hclk <= rdata_hclk_n;
        rsel_hclk <= rsel_hclk_n;
        wsel_hclk <= wsel_hclk_n;
        wdata_hclk <= wdata_hclk_n;
    end
end

// clk domain
always_ff @(posedge clk) begin
    if (~nrst) begin
        cdc_req_clk <= 1'b0;
        cdc_ack_clk <= 1'b0;
        rdata_clk <= 1'b0;
        rsel_clk <= '0;
        wsel_clk <= '0;
        wdata_clk <= '0;
    end else begin
        cdc_req_clk <= cdc_req_clk_n;
        cdc_ack_clk <= cdc_ack_clk_n;
        rdata_clk <= rdata_clk_n;
        rsel_clk <= rsel_clk_n;
        wsel_clk <= wsel_clk_n;
        wdata_clk <= wdata_clk_n;
    end
end

// FF synchronizer (and edge detector) for request signal
always_ff @(posedge clk) begin
    if (~nrst) begin
        cdc_req_sync[0] <= 1'b0;
        cdc_req_sync[1] <= 1'b0;
        cdc_req_sync[2] <= 1'b0;
    end else begin
        cdc_req_sync[0] <= cdc_req_hclk;
        cdc_req_sync[1] <= cdc_req_sync[0];
        cdc_req_sync[2] <= cdc_req_sync[1];
    end
end

// FF synchronizer (and edge detector) for acknowledge signal
always_ff @(posedge hclk) begin
    if (~nrst) begin
        cdc_ack_sync[0] <= 1'b0;
        cdc_ack_sync[1] <= 1'b0;
        cdc_ack_sync[2] <= 1'b0;
    end else begin
        cdc_ack_sync[0] <= cdc_ack_clk;
        cdc_ack_sync[1] <= cdc_ack_sync[0];
        cdc_ack_sync[2] <= cdc_ack_sync[1];
    end
end

// Request and data latching from hclk to clk domain
always_comb begin
    cdc_req_clk_n = ~cdc_req_sync[2] & cdc_req_sync[1]; // Edge detector for request signal in clk domain
    rsel_clk_n = rsel_clk;
    wsel_clk_n = wsel_clk;
    wdata_clk_n = wdata_clk;

    // If request signal is going high, latch rsel_hclk to clk domain
    if (~cdc_req_clk & cdc_req_clk_n) begin
        rsel_clk_n = rsel_hclk;
        wsel_clk_n = wsel_hclk;
        wdata_clk_n = wdata_hclk;
    end

    // Decode rsel to determine the read data
    // and send ack
    rdata_clk_n = rdata_clk;
    cdc_ack_clk_n = 1'b0;

    // Registers
    csr_n = csr;
    snapshot_n = '0;

    if (cdc_req_clk) begin
        cdc_ack_clk_n = 1'b1; // Acknowledge the request signal

        case (rsel_clk)
            2'd1: begin
                rdata_clk_n = csr;
            end
            2'd2: begin
                rdata_clk_n = {31'h0, snapshot};
            end
            default: begin
                rdata_clk_n = '0; // Default case
            end
        endcase

        case (wsel_clk)
            2'd1: begin
                csr_n = csr & ~wdata_clk; // Clear bits in CSR register
            end
            2'd2: begin
                snapshot_n = wdata_clk[0]; // Take snapshot if bit 0 is set
            end
            default: begin
                csr_n = csr;
            end
        endcase
    end

    // Override csr value from hardware if an epoch edge is detected
    csr_n = csr_n | (~epoch & epoch_edge); // Set positions in the CSR register where epoch edge is detected

    cdc_ack_hclk_n = ~cdc_ack_sync[2] & cdc_ack_sync[1]; // Edge detector for acknowledge signal in hclk domain
    rdata_hclk_n = rdata_hclk;

    // If ack signal is going high, latch rdata_clk to hclk domain
    if (~cdc_ack_hclk & cdc_ack_hclk_n) begin
        rdata_hclk_n = rdata_clk;
    end
end

endmodule