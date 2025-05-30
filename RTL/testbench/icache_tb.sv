`timescale 1ns / 1ns

`include "axi_controller_if.vh"
`include "cache_if.vh"
`include "axi_bus_if.vh"
`include "ram_if.vh"
`include "common_types.vh"
import common_types_pkg::*;

module icache_tb ();

    // Signals
    logic clk;
    logic nrst;

    // Interface
    axi_controller_if amif ();
    axi_bus_if abif_controller ();
    ram_if ram_if ();
    cache_if cache_if ();

    icache icache_inst (
        .clk(clk),
        .nrst(nrst),
        .amif(amif),
        .cif(cache_if)
    );

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
            cache_if.read = 0;
            cache_if.write = 0;
            cache_if.addr = 0;
            cache_if.store = 0;
            cache_if.done = 0;

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
            cache_if.write = size;
            cache_if.addr = addr;
            cache_if.store = wdata;

            // Wait for transaction to complete
            wait(cache_if.ready == 1);
            @(posedge clk);
            cache_if.done = 1;
            @(posedge clk);

            // Set bus signals to idle
            cache_if.done = 0;
            cache_if.write = '0;
            cache_if.addr = 0;
            @(posedge clk);
        end
    endtask

    task test_read;
        input word_t addr;
        input word_t exp_rdata;
        begin
            // Set signals to controller
            cache_if.read = 1;
            cache_if.addr = addr;

            // Wait for transaction to complete
            wait(cache_if.ready == 1);
            // @(posedge clk);
            cache_if.done = 1;
            @(posedge clk);

            // Set bus signals to idle
            cache_if.done = 0;
            cache_if.read = 0;
            cache_if.addr = 0;
            @(posedge clk);
        end
    endtask

    // Test sequence
    initial begin
        reset_dut;

        @(posedge clk);

        // Store and read from RAM

        // Set 0, block 0 goes valid
        test_read(32'h0000000, 32'h0);

        // Hit on set 0, block 0
        test_read(32'h0000000, 32'h0);

        // Set 0, block 1 goes valid
        test_read(32'h0000800, 32'h0);
        
        // Read other block
        test_read(32'h0000040, 32'h0);
        
        // Hit on set 0, block 0, block 1 gets LRU
        test_read(32'h0000000, 32'h0);

        // Set 0, block 1 gets replaced
        test_read(32'h0001000, 32'h0);

        // Finish simulation
        #50;
        $finish();
    end

endmodule