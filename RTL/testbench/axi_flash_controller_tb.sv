`timescale 1ns / 1ns

`include "axi_controller_if.vh"
`include "axi_bus_if.vh"
`include "ram_if.vh"
`include "common_types.vh"
import common_types_pkg::*;

module axi_flash_controller_tb ();

    // Signals
    logic clk;
    logic nrst;

    wire flash_cs;
    wire [3:0] flash_dq;

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

    axi_flash_controller axi_flash_inst (
        .clk(clk),
        .nrst(nrst),
        .abif(abif_controller),
        .clk_div(4'd2),
        .flash_cs(flash_cs),
        .flash_dq(flash_dq)
    );

    flash_model flash_model_inst (
        .clk(clk),
        .nrst(nrst),
        .clk_div(4'd2),
        .cs(flash_cs),
        .dq(flash_dq)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // Tasks
    task reset_dut;
        begin
            amif.read = 0;
            amif.write = 0;
            amif.addr = 0;
            amif.store = 0;
            amif.done = 0;
            abif_controller.awready = 1;
            abif_controller.wready = 1;
            abif_controller.bvalid = 1;
            abif_controller.bresp = '0;
            abif_controller.arready = 1;
            abif_controller.rvalid = 1;
            abif_controller.rresp = '0;
            abif_controller.rdata = '0;

            nrst = 0;
            #200ns;
            nrst = 1;
            @(posedge clk);
        end
    endtask

    task test_read;
        input word_t addr;
        input word_t exp_rdata;
        begin
            // Set signals to controller
            amif.read = 1;
            amif.addr = addr;

            // Wait for transaction to complete
            wait(amif.ready == 1);
            amif.done = 1;
            @(posedge clk);

            // Set bus signals to idle
            amif.done = 0;
            amif.read = 0;
            amif.addr = 0;
            @(posedge clk);
        end
    endtask

    // Test sequence
    initial begin
        reset_dut;

        @(posedge clk);

        // Store and read from RAM
        test_read(32'h0080_0D90, 32'h0000_0000);
        test_read(32'h0080_167C, 32'h0000_0000);
        test_read(32'h0080_112C, 32'h0000_0000);
        test_read(32'h0080_0140, 32'h0000_0000);

        // Finish simulation
        #50;
        $finish();
    end

endmodule