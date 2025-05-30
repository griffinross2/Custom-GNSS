`timescale 1ns / 1ns

`include "axi_controller_if.vh"
`include "axi_bus_if.vh"
`include "ram_if.vh"
`include "common_types.vh"
import common_types_pkg::*;

module axi_dram_test_fpga (
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

    output UART_TXD,
    output LED [0:3],
    output led0_r,
    output led0_g,
    output led0_b
);
    logic ui_clk;
    logic ui_clk_sync_rst;
    logic ui_ref_clk;
    logic mmcm_locked;
    logic aresetn;
    logic app_sr_req;
    logic app_ref_req;
    logic app_zq_req;
    logic app_sr_active;
    logic app_ref_ack;
    logic app_zq_ack;
    logic init_calib_complete;
    
    logic clk, nrst;
    assign clk = ui_clk;
    assign nrst = ~ui_clk_sync_rst;

    assign aresetn = 1'b1;
    assign app_sr_req = 1'b0;
    assign app_ref_req = 1'b0;
    assign app_zq_req = 1'b0;

    assign LED[0] = clk;
    assign LED[1] = nrst;
    assign LED[2] = ~UART_TXD;
    assign LED[3] = init_calib_complete;
    assign led0_r = (state == TEST_IDLE || state == TEST_DONE) ? 1'b1 : 1'b0; 
    assign led0_g = (state == TEST_WRITE || state == TEST_DONE) ? 1'b1 : 1'b0; 
    assign led0_b = (state == TEST_READ || state == TEST_DONE) ? 1'b1 : 1'b0; 

    // Interface
    axi_controller_if amif ();
    axi_bus_if abif_controller ();
    ram_if ram_if ();

    axi_controller axi_inst (
        .clk(clk),
        .nrst(nrst),
        .amif(amif),
        .abif(abif_controller)
    );

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
        .s_axi_awid                     (abif_controller.awid),  // input [3:0]			s_axi_awid
        .s_axi_awaddr                   (abif_controller.awaddr[27:0]),  // input [27:0]			s_axi_awaddr
        .s_axi_awlen                    (abif_controller.awlen),  // input [7:0]			s_axi_awlen
        .s_axi_awsize                   (abif_controller.awsize),  // input [2:0]			s_axi_awsize
        .s_axi_awburst                  (abif_controller.awburst),  // input [1:0]			s_axi_awburst
        .s_axi_awlock                   (abif_controller.awlock),  // input [0:0]			s_axi_awlock
        .s_axi_awcache                  (abif_controller.awcache),  // input [3:0]			s_axi_awcache
        .s_axi_awprot                   (abif_controller.awprot),  // input [2:0]			s_axi_awprot
        .s_axi_awqos                    (abif_controller.awqos),  // input [3:0]			s_axi_awqos
        .s_axi_awvalid                  (abif_controller.awvalid),  // input			s_axi_awvalid
        .s_axi_awready                  (abif_controller.awready),  // output			s_axi_awready


        // Slave Interface Write Data Ports
        .s_axi_wdata                    (abif_controller.wdata),  // input [31:0]			s_axi_wdata
        .s_axi_wstrb                    (abif_controller.wstrb),  // input [3:0]			s_axi_wstrb
        .s_axi_wlast                    (abif_controller.wlast),  // input			s_axi_wlast
        .s_axi_wvalid                   (abif_controller.wvalid),  // input			s_axi_wvalid
        .s_axi_wready                   (abif_controller.wready),  // output			s_axi_wready


        // Slave Interface Write Response Ports
        .s_axi_bid                      (abif_controller.bid),  // output [3:0]			s_axi_bid
        .s_axi_bresp                    (abif_controller.bresp),  // output [1:0]			s_axi_bresp
        .s_axi_bvalid                   (abif_controller.bvalid),  // output			s_axi_bvalid
        .s_axi_bready                   (abif_controller.bready),  // input			s_axi_bready


        // Slave Interface Read Address Ports
        .s_axi_arid                     (abif_controller.arid),  // input [3:0]			s_axi_arid
        .s_axi_araddr                   (abif_controller.araddr[27:0]),  // input [27:0]			s_axi_araddr
        .s_axi_arlen                    (abif_controller.arlen),  // input [7:0]			s_axi_arlen
        .s_axi_arsize                   (abif_controller.arsize),  // input [2:0]			s_axi_arsize
        .s_axi_arburst                  (abif_controller.arburst),  // input [1:0]			s_axi_arburst
        .s_axi_arlock                   (abif_controller.arlock),  // input [0:0]			s_axi_arlock
        .s_axi_arcache                  (abif_controller.arcache),  // input [3:0]			s_axi_arcache
        .s_axi_arprot                   (abif_controller.arprot),  // input [2:0]			s_axi_arprot
        .s_axi_arqos                    (abif_controller.arqos),  // input [3:0]			s_axi_arqos
        .s_axi_arvalid                  (abif_controller.arvalid),  // input			s_axi_arvalid
        .s_axi_arready                  (abif_controller.arready),  // output			s_axi_arready


        // Slave Interface Read Data Ports
        .s_axi_rid                      (abif_controller.rid),  // output [3:0]			s_axi_rid
        .s_axi_rdata                    (abif_controller.rdata),  // output [31:0]			s_axi_rdata
        .s_axi_rresp                    (abif_controller.rresp),  // output [1:0]			s_axi_rresp
        .s_axi_rlast                    (abif_controller.rlast),  // output			s_axi_rlast
        .s_axi_rvalid                   (abif_controller.rvalid),  // output			s_axi_rvalid
        .s_axi_rready                   (abif_controller.rready),  // input			s_axi_rready


        // System Clock Ports
        .sys_clk_i                       (sys_clk_i),  // input			sys_clk_i


        // Reference Clock Ports
        .clk_ref_i                      (ui_ref_clk),
        .sys_rst                        (ck_rst) // input sys_rst

    );
    
    localparam int BIT_PERIOD = 705; // Bit period in clk cycles

    logic uart_done;
    logic [7:0] uart_data;
    logic uart_start;
    uart_tx uart_inst (
        .clk(clk),
        .nrst(nrst),
        .bit_period(BIT_PERIOD),
        .data(uart_data),
        .start(uart_start),
        .serial_out(UART_TXD),
        .tx_done(uart_done)
    );

    typedef enum logic [3:0] {
        TEST_IDLE,
        TEST_CLEAR,
        TEST_WRITE,
        TEST_READ,
        TEST_UART_0,
        TEST_UART_1,
        TEST_UART_2,
        TEST_UART_3,
        TEST_DONE
    } test_state_t;

    test_state_t state, next_state;

    always_ff @(posedge clk) begin
        if (~nrst) begin
            state <= TEST_IDLE;
        end else begin
            state <= next_state;
        end
    end

    always_comb begin
        next_state = state;
        case (state)
            TEST_IDLE: begin
                if (init_calib_complete) begin
                    next_state = TEST_CLEAR;
                end
            end

            TEST_CLEAR: begin
                if (amif.ready && amif.done) begin
                    next_state = TEST_WRITE;
                end
            end

            TEST_WRITE: begin
                if (amif.ready && amif.done) begin
                    next_state = TEST_READ;
                end
            end

            TEST_READ: begin
                if (amif.ready) begin
                    next_state = TEST_UART_0;
                end
            end

            TEST_UART_0: begin
                if (uart_done) begin
                    next_state = TEST_UART_1;
                end
            end

            TEST_UART_1: begin
                if (uart_done) begin
                    next_state = TEST_UART_2;
                end
            end

            TEST_UART_2: begin
                if (uart_done) begin
                    next_state = TEST_UART_3;
                end
            end

            TEST_UART_3: begin
                if (uart_done) begin
                    next_state = TEST_DONE;
                end
            end

            TEST_DONE: begin
            end

            default: begin
                next_state = TEST_IDLE;
            end
        endcase
    end

    always_comb begin
        amif.read = 1'b0;
        amif.write = 2'b00;
        amif.addr = 32'h00000000;
        amif.store = 32'h00000000;
        amif.done = 1'b0;

        uart_start = 1'b0;
        uart_data = 8'h00;

        case (state)
            TEST_IDLE: begin
            end

            TEST_CLEAR: begin
                amif.write = 2'b11;
                amif.addr = 32'h00000020;
                amif.store = 32'h00000000;

                if (amif.ready) begin
                    amif.done = 1'b1;
                end
            end

            TEST_WRITE: begin
                amif.write = 2'b10;
                amif.addr = 32'h00000020;
                amif.store = 32'hABCD1234;

                if (amif.ready) begin
                    amif.done = 1'b1;
                end
            end

            TEST_READ: begin
                amif.read = 1'b1;
                amif.addr = 32'h00000020;
            end

            TEST_UART_0: begin
                uart_start = 1'b1;
                uart_data = amif.load[7:0];

                if (uart_done) begin
                    uart_start = 1'b0;
                end
            end

            TEST_UART_1: begin
                uart_start = 1'b1;
                uart_data = amif.load[15:8];

                if (uart_done) begin
                    uart_start = 1'b0;
                end
            end

            TEST_UART_2: begin
                uart_start = 1'b1;
                uart_data = amif.load[23:16];

                if (uart_done) begin
                    uart_start = 1'b0;
                end
            end

            TEST_UART_3: begin
                uart_start = 1'b1;
                uart_data = amif.load[31:24];

                if (uart_done) begin
                    uart_start = 1'b0;
                end
            end

            TEST_DONE: begin
            end

            default: begin
            end
        endcase
    end

endmodule