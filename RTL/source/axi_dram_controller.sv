/********************************/
/*  Flash Read-Only Controller  */
/********************************/
`timescale 1ns/1ns

`include "axi_bus_if.vh"

`include "common_types.vh"
import common_types_pkg::*;

module axi_dram_controller (
    input sys_clk_i,
    input ck_rst,

    inout [15:0] ddr3_dq,
    output [1:0] ddr3_dm,
    inout [1:0] ddr3_dqs_p,
    inout [1:0] ddr3_dqs_n,
    output [13:0] ddr3_addr,
    output [2:0] ddr3_ba,
    output [0:0] ddr3_ck_p,
    output [0:0] ddr3_ck_n,
    output ddr3_ras_n,
    output ddr3_cas_n,
    output ddr3_we_n,
    output ddr3_reset_n,
    output [0:0] ddr3_cke,
    output [0:0] ddr3_odt,
    output [0:0] ddr3_cs_n,

    output clk,
    output nrst,
    axi_bus_if.satellite_to_mux abif
);

    logic ui_clk;
    logic ui_clk_sync_rst;
    logic ui_ref_clk;
    logic mmcm_locked;
    logic app_sr_req;
    logic app_ref_req;
    logic app_zq_req;
    logic app_sr_active;
    logic app_ref_ack;
    logic app_zq_ack;
    logic init_calib_complete;
    
    assign clk = ui_clk;
    assign nrst = ~ui_clk_sync_rst & init_calib_complete;

    assign app_sr_req = 1'b0;
    assign app_ref_req = 1'b0;
    assign app_zq_req = 1'b0;

    word_t shifted_waddr;
    word_t shifted_raddr;

    // The DDR3 starts at 0x1000_0000 for the CPU
    // Also align the address to 32 bits
    assign shifted_raddr = {abif.araddr[31:2], 2'b00} - 32'h1000_0000;
    assign shifted_waddr = {abif.awaddr[31:2], 2'b00} - 32'h1000_0000;

    mig_7series_0 u_mig_7series_0 (
        // Memory interface ports
        .ddr3_addr                      (ddr3_addr),  // output [13:0]		ddr3_addr
        .ddr3_ba                        (ddr3_ba),  // output [2:0]		ddr3_ba
        .ddr3_cas_n                     (ddr3_cas_n),  // output			ddr3_cas_n
        .ddr3_ck_n                      (ddr3_ck_n),  // output [0:0]		ddr3_ck_n
        .ddr3_ck_p                      (ddr3_ck_p),  // output [0:0]		ddr3_ck_p
        .ddr3_cke                       (ddr3_cke),  // output [0:0]		ddr3_cke
        .ddr3_ras_n                     (ddr3_ras_n),  // output			ddr3_ras_n
        .ddr3_reset_n                   (ddr3_reset_n),  // output			ddr3_reset_n
        .ddr3_we_n                      (ddr3_we_n),  // output			ddr3_we_n
        .ddr3_dq                        (ddr3_dq),  // inout [15:0]		ddr3_dq
        .ddr3_dqs_n                     (ddr3_dqs_n),  // inout [1:0]		ddr3_dqs_n
        .ddr3_dqs_p                     (ddr3_dqs_p),  // inout [1:0]		ddr3_dqs_p
        .init_calib_complete            (init_calib_complete),  // output			init_calib_complete
        .ddr3_cs_n                      (ddr3_cs_n),  // output [0:0]		ddr3_cs_n
        .ddr3_dm                        (ddr3_dm),  // output [1:0]		ddr3_dm
        .ddr3_odt                       (ddr3_odt),  // output [0:0]		ddr3_odt


        // Application interface ports
        .ui_clk                         (ui_clk),  // output			ui_clk
        .ui_clk_sync_rst                (ui_clk_sync_rst),  // output			ui_clk_sync_rst
        .ui_addn_clk_0                  (ui_ref_clk),  // output			ui_addn_clk_0
        .mmcm_locked                    (mmcm_locked),  // output			mmcm_locked
        .aresetn                        (ck_rst),  // input			aresetn
        .app_sr_req                     (app_sr_req),  // input			app_sr_req
        .app_ref_req                    (app_ref_req),  // input			app_ref_req
        .app_zq_req                     (app_zq_req),  // input			app_zq_req
        .app_sr_active                  (app_sr_active),  // output			app_sr_active
        .app_ref_ack                    (app_ref_ack),  // output			app_ref_ack
        .app_zq_ack                     (app_zq_ack),  // output			app_zq_ack


        // Slave Interface Write Address Ports
        .s_axi_awid                     (abif.awid),  // input [3:0]			s_axi_awid
        .s_axi_awaddr                   (shifted_waddr),  // input [27:0]			s_axi_awaddr
        .s_axi_awlen                    (abif.awlen),  // input [7:0]			s_axi_awlen
        .s_axi_awsize                   (abif.awsize),  // input [2:0]			s_axi_awsize
        .s_axi_awburst                  (abif.awburst),  // input [1:0]			s_axi_awburst
        .s_axi_awlock                   (abif.awlock),  // input [0:0]			s_axi_awlock
        .s_axi_awcache                  (abif.awcache),  // input [3:0]			s_axi_awcache
        .s_axi_awprot                   (abif.awprot),  // input [2:0]			s_axi_awprot
        .s_axi_awqos                    (abif.awqos),  // input [3:0]			s_axi_awqos
        .s_axi_awvalid                  (abif.awvalid),  // input			s_axi_awvalid
        .s_axi_awready                  (abif.awready),  // output			s_axi_awready


        // Slave Interface Write Data Ports
        .s_axi_wdata                    (abif.wdata),  // input [31:0]			s_axi_wdata
        .s_axi_wstrb                    (abif.wstrb),  // input [3:0]			s_axi_wstrb
        .s_axi_wlast                    (abif.wlast),  // input			s_axi_wlast
        .s_axi_wvalid                   (abif.wvalid),  // input			s_axi_wvalid
        .s_axi_wready                   (abif.wready),  // output			s_axi_wready


        // Slave Interface Write Response Ports
        .s_axi_bid                      (abif.bid),  // output [3:0]			s_axi_bid
        .s_axi_bresp                    (abif.bresp),  // output [1:0]			s_axi_bresp
        .s_axi_bvalid                   (abif.bvalid),  // output			s_axi_bvalid
        .s_axi_bready                   (abif.bready),  // input			s_axi_bready


        // Slave Interface Read Address Ports
        .s_axi_arid                     (abif.arid),  // input [3:0]			s_axi_arid
        .s_axi_araddr                   (shifted_raddr),  // input [27:0]			s_axi_araddr
        .s_axi_arlen                    (abif.arlen),  // input [7:0]			s_axi_arlen
        .s_axi_arsize                   (abif.arsize),  // input [2:0]			s_axi_arsize
        .s_axi_arburst                  (abif.arburst),  // input [1:0]			s_axi_arburst
        .s_axi_arlock                   (abif.arlock),  // input [0:0]			s_axi_arlock
        .s_axi_arcache                  (abif.arcache),  // input [3:0]			s_axi_arcache
        .s_axi_arprot                   (abif.arprot),  // input [2:0]			s_axi_arprot
        .s_axi_arqos                    (abif.arqos),  // input [3:0]			s_axi_arqos
        .s_axi_arvalid                  (abif.arvalid),  // input			s_axi_arvalid
        .s_axi_arready                  (abif.arready),  // output			s_axi_arready


        // Slave Interface Read Data Ports
        .s_axi_rid                      (abif.rid),  // output [3:0]			s_axi_rid
        .s_axi_rdata                    (abif.rdata),  // output [31:0]			s_axi_rdata
        .s_axi_rresp                    (abif.rresp),  // output [1:0]			s_axi_rresp
        .s_axi_rlast                    (abif.rlast),  // output			s_axi_rlast
        .s_axi_rvalid                   (abif.rvalid),  // output			s_axi_rvalid
        .s_axi_rready                   (abif.rready),  // input			s_axi_rready


        // System Clock Ports
        .sys_clk_i                       (sys_clk_i),  // input			sys_clk_i


        // Reference Clock Ports
        .clk_ref_i                      (ui_ref_clk),
        .sys_rst                        (ck_rst) // input sys_rst

    );
    
endmodule