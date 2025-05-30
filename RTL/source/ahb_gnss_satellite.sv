`timescale 1ns/1ns

`include "common_types.vh"
import common_types_pkg::*;
`include "common_gnss_types.vh"
import common_gnss_types_pkg::*;

`include "ahb_bus_if.vh"

module ahb_gnss_satellite (
    input logic clk, hclk, nrst,
    ahb_bus_if.satellite_to_mux abif,
    input logic search_busy,
    input logic fine_search_busy,
    output logic search_start,
    output sv_t search_sv,
    input word_t search_dop,
    input word_t search_code,
    input word_t search_corr
);

typedef enum logic [1:0] {
    CTRL_IDLE,
    CTRL_REG_ADDR,
    CTRL_REQ,
    CTRL_WAIT
} controller_state_t;

typedef enum logic {
    SAT_IDLE,
    SAT_ACK
} satellite_state_t;

controller_state_t controller_state, next_controller_state;
satellite_state_t satellite_state, next_satellite_state;

// Save address phase signals
word_t haddr_reg_hclk;
logic hwrite_reg_hclk, hsel_reg_hclk, hready_reg_hclk;
htrans_t htrans_reg_hclk;
logic [2:0] hburst_reg_hclk;
logic [1:0] hsize_reg_hclk;

word_t hwdata_reg;
word_t haddr_reg;
logic hwrite_reg, hsel_reg, hready_reg;
htrans_t htrans_reg;
logic [2:0] hburst_reg;
logic [1:0] hsize_reg;
word_t next_hrdata;
logic next_hreadyout;
logic next_hresp;

logic [1:0] req_reg;
logic next_req;
logic [1:0] ack_reg;
logic next_ack;

logic next_search_start;
logic fine_search_busy_reg;
logic search_done_reg, next_search_done_reg;
sv_t next_search_sv;

// Controller state machine
always_ff @(posedge hclk) begin
    if (!nrst) begin
        controller_state <= CTRL_IDLE;
        abif.hrdata <= 32'b0;
        abif.hreadyout <= 1'b0;
        abif.hresp <= 1'b0;
        ack_reg <= '0;

        haddr_reg_hclk <= 32'b0;
        hwrite_reg_hclk <= 1'b0;
        hsel_reg_hclk <= 1'b0;
        hready_reg_hclk <= 1'b0;
        htrans_reg_hclk <= HTRANS_IDLE;
        hburst_reg_hclk <= 3'b0;
        hsize_reg_hclk <= 2'b0;
    end else begin
        controller_state <= next_controller_state;
        abif.hrdata <= next_hrdata;
        abif.hreadyout <= next_hreadyout;
        abif.hresp <= next_hresp;
        ack_reg <= {ack_reg[0], next_ack};

        if (abif.htrans != HTRANS_IDLE && abif.hsel && abif.hready) begin
            haddr_reg_hclk <= abif.haddr;
            hwrite_reg_hclk <= abif.hwrite;
            hsel_reg_hclk <= abif.hsel;
            hready_reg_hclk <= abif.hready;
            htrans_reg_hclk <= abif.htrans;
            hburst_reg_hclk <= abif.hburst;
            hsize_reg_hclk <= abif.hsize;
        end
    end
end

// Satellite state machine
always_ff @(posedge clk) begin
    if (!nrst) begin
        satellite_state <= SAT_IDLE;
        hwdata_reg <= 32'b0;
        haddr_reg <= 32'b0;
        hwrite_reg <= 1'b0;
        hsel_reg <= 1'b0;
        hready_reg <= 1'b0;
        htrans_reg <= HTRANS_IDLE;
        hburst_reg <= 3'b0;
        hsize_reg <= 2'b0;
        req_reg <= '0;
    end else begin
        satellite_state <= next_satellite_state;
        hwdata_reg <= abif.hwdata;
        haddr_reg <= haddr_reg_hclk;
        hwrite_reg <= hwrite_reg_hclk;
        hsel_reg <= hsel_reg_hclk;
        hready_reg <= hready_reg_hclk;
        htrans_reg <= htrans_reg_hclk;
        hburst_reg <= hburst_reg_hclk;
        hsize_reg <= hsize_reg_hclk;
        req_reg <= {req_reg[0], next_req};
    end
end

// Next state logic
always_comb begin
    next_controller_state = controller_state;
    next_satellite_state = satellite_state;

    case (controller_state)
        CTRL_IDLE: begin
            if (abif.htrans == HTRANS_NONSEQ && abif.hsel && abif.hready) begin
                next_controller_state = CTRL_REG_ADDR;
            end
        end
        CTRL_REG_ADDR: begin
            next_controller_state = CTRL_REQ;
        end
        CTRL_REQ: begin
            if (ack_reg[1]) begin
                next_controller_state = CTRL_WAIT;
            end
        end
        CTRL_WAIT: begin
            if (ack_reg == 2'b00) begin
                next_controller_state = CTRL_IDLE;
            end
        end
    endcase
    
    case (satellite_state)
        SAT_IDLE: begin
            if (req_reg[1]) begin
                next_satellite_state = SAT_ACK;
            end
        end
        SAT_ACK: begin
            if (!req_reg[1]) begin
                next_satellite_state = SAT_IDLE;
            end
        end
    endcase
end

// AHB output logic
always_comb begin
    next_hrdata = abif.hrdata;
    next_hreadyout = abif.hreadyout;
    next_hresp = abif.hresp;
    next_req = 1'b0;
    next_ack = 1'b0;

    case (controller_state)
        CTRL_IDLE: begin
            if (abif.htrans == HTRANS_NONSEQ && abif.hsel && abif.hready) begin
                next_hreadyout = 1'b0;
                next_hresp = 1'b0;
                next_req = 1'b1;
            end else begin
                next_hreadyout = 1'b1;
                next_req = 1'b0;
            end
        end
        CTRL_REG_ADDR: begin
            next_hreadyout = 1'b0;
            next_req = 1'b0;
        end
        CTRL_REQ: begin
            if (ack_reg[1]) begin
                next_req = 1'b0;
                next_hreadyout = 1'b0;
            end else begin
                next_req = 1'b1;
                next_hreadyout = 1'b0;
            end
        end
        CTRL_WAIT: begin
            if (ack_reg == 2'b00) begin
                next_req = 1'b0;
                next_hreadyout = 1'b1;
            end else begin
                next_req = 1'b0;
                next_hreadyout = 1'b0;
            end
        end
    endcase

    case (satellite_state)
        SAT_IDLE: begin
            if (req_reg[1]) begin
                next_ack = 1'b1;
            end else begin
                next_ack = 1'b0;
            end
        end
        SAT_ACK: begin
            if (!req_reg[1]) begin
                next_ack = 1'b0;
            end else begin
                next_ack = 1'b1;
            end
        end
    endcase

    // Register Read Logic
    case (haddr_reg)
        32'h2004_0100: begin // Status/Start
            next_hrdata = {30'b0, search_done_reg, search_start};
        end
        32'h2004_0104: begin // Search SV
            next_hrdata = {27'b0, search_sv};
        end
        32'h2004_0108: begin // Search DOP
            next_hrdata = search_dop;
        end
        32'h2004_010C: begin // Search Code
            next_hrdata = search_code;
        end
        32'h2004_0110: begin // Search Correlation
            next_hrdata = search_corr;
        end
        default: begin
            next_hrdata = 32'b0;
        end
    endcase
end

// Search Register Write Logic
always_ff @(posedge clk) begin
    if (!nrst) begin
        fine_search_busy_reg <= 1'b0;
        search_done_reg <= 1'b0;
        search_start <= 1'b0;
        search_sv <= '0;
    end else begin
        fine_search_busy_reg <= fine_search_busy;
        search_done_reg <= next_search_done_reg;
        search_start <= next_search_start;
        search_sv <= next_search_sv;
    end
end

// Search Register Write Logic
always_comb begin
    // Assert done when the fine search completes
    next_search_done_reg = (search_done_reg == 1'b0) ? !fine_search_busy && fine_search_busy_reg : search_done_reg;

    // Disable the start signal when the coarse search starts
    next_search_start = (search_start == 1'b1) ? !search_busy : 1'b0;
    
    next_search_sv = search_sv;

    if (satellite_state == SAT_IDLE && next_satellite_state == SAT_ACK) begin
        // Do the write here
        if (hwrite_reg && haddr_reg == 32'h2004_0100) begin
            // Status/Start
            next_search_start = hwdata_reg[0]; // Start search if bit 0 is set
            next_search_done_reg = (|hwdata_reg[1:0]) ? 1'b0 : search_done_reg; // Clear done if bit 1 or 0 is set
        end
        
        if (hwrite_reg && haddr_reg == 32'h2004_0104) begin
            // Configuration
            next_search_sv = hwdata_reg[4:0];
        end
    end
end

endmodule 