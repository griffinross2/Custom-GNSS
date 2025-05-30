`timescale 1ns / 1ns

`include "axi_controller_if.vh"
`include "axi_bus_if.vh"
`include "ram_if.vh"
`include "common_types.vh"
import common_types_pkg::*;

module axi_rom_controller_tb ();

    // Signals
    logic clk;
    logic nrst;

    // Interface
    axi_controller_if amif ();
    axi_bus_if abif_controller ();
    ram_if ram_if ();

    rom rom_inst (
        .clk(clk),
        .nrst(nrst),
        .ram_if(ram_if)
    );

    axi_controller axi_inst (
        .clk(clk),
        .nrst(nrst),
        .amif(amif),
        .abif(abif_controller)
    );

    axi_rom_controller axi_rom_inst (
        .clk(clk),
        .nrst(nrst),
        .abif(abif_controller),
        .ram_if(ram_if)
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

            nrst = 0;
            @(posedge clk);
            @(posedge clk);
            nrst = 1;
        end
    endtask

    task test_write;
        input logic [1:0] size;
        input word_t addr;
        input word_t wdata;
        begin
            // Set signals to controller
            amif.write = size;
            amif.addr = addr;
            amif.store = wdata;

            // Wait for transaction to complete
            wait(amif.ready == 1);
            amif.done = 1;
            @(posedge clk);

            // Set bus signals to idle
            amif.done = 0;
            amif.write = '0;
            amif.addr = 0;
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
        test_write(2'b10, 32'h00000022, 32'h56780000);
        test_write(2'b10, 32'h00000020, 32'hABCD1234);
        test_read(32'h00000020, 32'h56781234);

        // Finish simulation
        #50;
        $finish();
    end

endmodule