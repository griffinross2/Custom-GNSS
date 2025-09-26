`timescale 1ns/1ns

`include "common_types.vh"
import common_types_pkg::*;
`include "common_gnss_types.vh"
import common_gnss_types_pkg::*;
`include "gnss_channel_satellite_if.vh"

`include "ahb_bus_if.vh"

module ahb_gnss_satellite (
    input logic clk, hclk, nrst,
    ahb_bus_if.satellite_to_mux abif,
    input logic search_busy,
    input word_t search_dop,
    input word_t search_code,
    input word_t search_subcode,
    input word_t search_corr,
    input logic [5:0] search_channel_in,
    input logic search_done,
    output logic search_start,
    output sv_t search_sv,
    output logic [5:0] search_channel,
    gnss_channel_satellite_if.satellite channel_0
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

logic next_clear_0;
logic [31:0] next_code_rate_0;
logic [31:0] next_lo_rate_0;

logic [15:0] ie_reg_0, qe_reg_0, ip_reg_0, qp_reg_0, il_reg_0, ql_reg_0;
logic epoch_reg_0, next_epoch_reg_0;

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
logic search_busy_reg;
logic search_done_reg, next_search_done_reg;
sv_t next_search_sv;
logic [5:0] next_search_channel;

// Have the translation from doppler index to code and lo rate
word_t code_nco_for_dop;
word_t lo_nco_for_dop;

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
        32'h2004_0114: begin // Search Sub-Code Chip
            next_hrdata = search_subcode;
        end
        32'h2004_0400: begin // Channel 0 Status and Config Register
            next_hrdata = '0;
            next_hrdata[2] = epoch_reg_0; // Bit 2 : epoch
        end
        32'h2004_0404: begin // Channel 0 Code Rate Register
            next_hrdata = channel_0.code_rate;
        end
        32'h2004_0408: begin // Channel 0 LO Rate Register
            next_hrdata = channel_0.lo_rate;
        end
        32'h2004_040C: begin // Channel 0 Early Accumulator
            next_hrdata = {qe_reg_0, ie_reg_0};
        end
        32'h2004_0410: begin // Channel 0 Late Accumulator
            next_hrdata = {ql_reg_0, il_reg_0};
        end
        32'h2004_0414: begin // Channel 0 Prompt Accumulator
            next_hrdata = {qp_reg_0, ip_reg_0};
        end
        default: begin
            next_hrdata = 32'b0;
        end
    endcase
end

// Search Register Write Logic
always_ff @(posedge clk) begin
    if (!nrst) begin
        search_busy_reg <= 1'b0;
        search_done_reg <= 1'b0;
        search_start <= 1'b0;
        search_sv <= '0;
        search_channel <= '0;
        channel_0.clear <= 1'b0;
        channel_0.code_rate <= '0;
        channel_0.lo_rate <= '0;
        ie_reg_0 <= '0;
        qe_reg_0 <= '0;
        ip_reg_0 <= '0;
        qp_reg_0 <= '0;
        il_reg_0 <= '0;
        ql_reg_0 <= '0;
        epoch_reg_0 <= 1'b0;
    end else begin
        search_busy_reg <= search_busy;
        search_done_reg <= next_search_done_reg;
        search_start <= next_search_start;
        search_sv <= next_search_sv;
        search_channel <= next_search_channel;
        channel_0.clear <= next_clear_0;
        channel_0.code_rate <= next_code_rate_0;
        channel_0.lo_rate <= next_lo_rate_0;
        ie_reg_0 <= (epoch_reg_0 == 0 && channel_0.epoch) ? channel_0.ie : ie_reg_0;
        qe_reg_0 <= (epoch_reg_0 == 0 && channel_0.epoch) ? channel_0.qe : qe_reg_0;
        ip_reg_0 <= (epoch_reg_0 == 0 && channel_0.epoch) ? channel_0.ip : ip_reg_0;
        qp_reg_0 <= (epoch_reg_0 == 0 && channel_0.epoch) ? channel_0.qp : qp_reg_0;
        il_reg_0 <= (epoch_reg_0 == 0 && channel_0.epoch) ? channel_0.il : il_reg_0;
        ql_reg_0 <= (epoch_reg_0 == 0 && channel_0.epoch) ? channel_0.ql : ql_reg_0;
        epoch_reg_0 <= next_epoch_reg_0;
    end
end

// Search Register Write Logic
always_comb begin
    // Assert done when the fine search completes
    next_search_done_reg = (search_done_reg == 1'b0) ? !search_busy && search_busy_reg : search_done_reg;

    // Disable the start signal when the search starts
    next_search_start = (search_start == 1'b1) ? !search_busy : 1'b0;
    
    next_search_sv = search_sv;
    next_search_channel = search_channel;
    next_code_rate_0 = channel_0.code_rate;
    next_lo_rate_0 = channel_0.lo_rate;
    next_clear_0 = 1'b0;  // Always reset after a cycle
    next_epoch_reg_0 = epoch_reg_0;

    if (satellite_state == SAT_IDLE && next_satellite_state == SAT_ACK) begin
        // Do the write here
        if (hwrite_reg && haddr_reg == 32'h2004_0100) begin
            // Status/Start
            next_search_start = hwdata_reg[0]; // Start search if bit 0 is set
            next_search_done_reg = (|hwdata_reg[1:0]) ? 1'b0 : search_done_reg; // Clear done if bit 1 or 0 is set
        end
        
        if (hwrite_reg && haddr_reg == 32'h2004_0104) begin
            // Configuration
            next_search_sv = hwdata_reg[5:0];
            next_search_channel = hwdata_reg[13:8];
        end

        if (hwrite_reg && haddr_reg == 32'h2004_0400) begin
            // Status and Config Register
            next_clear_0 = hwdata_reg[0]; // Bit 0 : clear
            next_epoch_reg_0 = hwdata_reg[2] ? 1'b0 : next_epoch_reg_0; // Bit 2 : epoch (clear if set)
        end

        if (hwrite_reg && haddr_reg == 32'h2004_0404) begin
            // Code Rate Register
            next_code_rate_0 = hwdata_reg;
        end

        if (hwrite_reg && haddr_reg == 32'h2004_0408) begin
            // LO Rate Register
            next_lo_rate_0 = hwdata_reg;
        end
    end

    // Set epoch reg if channel epoch occurs
    next_epoch_reg_0 = channel_0.epoch ? 1'b1 : next_epoch_reg_0;

    // Translate the dop index to code and LO rate
    code_nco_for_dop = 32'd228841226;
    lo_nco_for_dop = 32'd899258778;
    case (signed'(search_dop))
        // Starting code rate
        -32'd20: code_nco_for_dop = 32'd228840500;
        -32'd19: code_nco_for_dop = 32'd228840536;
        -32'd18: code_nco_for_dop = 32'd228840573;
        -32'd17: code_nco_for_dop = 32'd228840609;
        -32'd16: code_nco_for_dop = 32'd228840645;
        -32'd15: code_nco_for_dop = 32'd228840682;
        -32'd14: code_nco_for_dop = 32'd228840718;
        -32'd13: code_nco_for_dop = 32'd228840754;
        -32'd12: code_nco_for_dop = 32'd228840790;
        -32'd11: code_nco_for_dop = 32'd228840827;
        -32'd10: code_nco_for_dop = 32'd228840863;
        -32'd09: code_nco_for_dop = 32'd228840899;
        -32'd08: code_nco_for_dop = 32'd228840936;
        -32'd07: code_nco_for_dop = 32'd228840972;
        -32'd06: code_nco_for_dop = 32'd228841008;
        -32'd05: code_nco_for_dop = 32'd228841045;
        -32'd04: code_nco_for_dop = 32'd228841081;
        -32'd03: code_nco_for_dop = 32'd228841117;
        -32'd02: code_nco_for_dop = 32'd228841154;
        -32'd01: code_nco_for_dop = 32'd228841190;
        32'd00: code_nco_for_dop = 32'd228841226;
        32'd01: code_nco_for_dop = 32'd228841263;
        32'd02: code_nco_for_dop = 32'd228841299;
        32'd03: code_nco_for_dop = 32'd228841335;
        32'd04: code_nco_for_dop = 32'd228841371;
        32'd05: code_nco_for_dop = 32'd228841408;
        32'd06: code_nco_for_dop = 32'd228841444;
        32'd07: code_nco_for_dop = 32'd228841480;
        32'd08: code_nco_for_dop = 32'd228841517;
        32'd09: code_nco_for_dop = 32'd228841553;
        32'd10: code_nco_for_dop = 32'd228841589;
        32'd11: code_nco_for_dop = 32'd228841626;
        32'd12: code_nco_for_dop = 32'd228841662;
        32'd13: code_nco_for_dop = 32'd228841698;
        32'd14: code_nco_for_dop = 32'd228841735;
        32'd15: code_nco_for_dop = 32'd228841771;
        32'd16: code_nco_for_dop = 32'd228841807;
        32'd17: code_nco_for_dop = 32'd228841844;
        32'd18: code_nco_for_dop = 32'd228841880;
        32'd19: code_nco_for_dop = 32'd228841916;
        32'd20: code_nco_for_dop = 32'd228841953;
    endcase
    case (signed'(search_dop))
        // Starting lo rate
        -32'd20: lo_nco_for_dop = 898140297;
        -32'd19: lo_nco_for_dop = 898196221;
        -32'd18: lo_nco_for_dop = 898252145;
        -32'd17: lo_nco_for_dop = 898308069;
        -32'd16: lo_nco_for_dop = 898363993;
        -32'd15: lo_nco_for_dop = 898419917;
        -32'd14: lo_nco_for_dop = 898475841;
        -32'd13: lo_nco_for_dop = 898531765;
        -32'd12: lo_nco_for_dop = 898587689;
        -32'd11: lo_nco_for_dop = 898643613;
        -32'd10: lo_nco_for_dop = 898699537;
        -32'd09: lo_nco_for_dop = 898755461;
        -32'd08: lo_nco_for_dop = 898811385;
        -32'd07: lo_nco_for_dop = 898867309;
        -32'd06: lo_nco_for_dop = 898923233;
        -32'd05: lo_nco_for_dop = 898979157;
        -32'd04: lo_nco_for_dop = 899035081;
        -32'd03: lo_nco_for_dop = 899091005;
        -32'd02: lo_nco_for_dop = 899146929;
        -32'd01: lo_nco_for_dop = 899202854;
        32'd00: lo_nco_for_dop = 899258778;
        32'd01: lo_nco_for_dop = 899314702;
        32'd02: lo_nco_for_dop = 899370626;
        32'd03: lo_nco_for_dop = 899426550;
        32'd04: lo_nco_for_dop = 899482474;
        32'd05: lo_nco_for_dop = 899538398;
        32'd06: lo_nco_for_dop = 899594322;
        32'd07: lo_nco_for_dop = 899650246;
        32'd08: lo_nco_for_dop = 899706170;
        32'd09: lo_nco_for_dop = 899762094;
        32'd10: lo_nco_for_dop = 899818018;
        32'd11: lo_nco_for_dop = 899873942;
        32'd12: lo_nco_for_dop = 899929866;
        32'd13: lo_nco_for_dop = 899985790;
        32'd14: lo_nco_for_dop = 900041714;
        32'd15: lo_nco_for_dop = 900097638;
        32'd16: lo_nco_for_dop = 900153562;
        32'd17: lo_nco_for_dop = 900209487;
        32'd18: lo_nco_for_dop = 900265411;
        32'd19: lo_nco_for_dop = 900321335;
        32'd20: lo_nco_for_dop = 900377259;
    endcase

    // Overwrite the rate registers if a search finished
    if (search_done) begin
        case (search_channel_in)
            6'd0: begin
                next_code_rate_0 = code_nco_for_dop;
                next_lo_rate_0 = lo_nco_for_dop;
            end
            default: begin
                // No action for other channels
            end
        endcase
    end
end

endmodule 