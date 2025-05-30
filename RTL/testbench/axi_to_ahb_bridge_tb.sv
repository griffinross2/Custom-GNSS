`timescale 1ns / 1ns

`include "axi_controller_if.vh"
`include "axi_bus_if.vh"
`include "ahb_bus_if.vh"
`include "common_types.vh"
import common_types_pkg::*;

module axi_to_ahb_bridge_tb ();

    // Signals
    logic clk;
    logic nrst;
    logic rxd;
    logic txd;
    logic rxi;

    assign rxd = 1'b1;

    // Interface
    axi_controller_if amif ();
    axi_bus_if abif_controller ();
    ahb_bus_if abif_uart ();

    assign abif_uart.hready = abif_uart.hreadyout;
    assign abif_uart.hsel = 1'b1;

    axi_controller axi_inst (
        .clk(clk),
        .nrst(nrst),
        .amif(amif),
        .abif(abif_controller)
    );

    axi_to_ahb_bridge axi_to_ahb_inst (
        .clk(clk),
        .nrst(nrst),
        .axi(abif_controller),
        .ahb(abif_uart)
    );

    ahb_uart_satellite ahb_uart_inst (
        .clk(clk),
        .nrst(nrst),
        .abif(abif_uart),
        .rxd(rxd),
        .txd(txd),
        .rxi(rxi)
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
        output word_t rdata;
        begin
            // Set signals to controller
            amif.read = 1;
            amif.addr = addr;

            // Wait for transaction to complete
            wait(amif.ready == 1);
            amif.done = 1;
            rdata = amif.load;
            @(posedge clk);

            // Set bus signals to idle
            amif.done = 0;
            amif.read = 0;
            amif.addr = 0;
            @(posedge clk);
        end
    endtask

    // Test sequence
    word_t rdata;
    initial begin
        reset_dut;

        @(posedge clk);

        rdata = 0;

        // Store and read from RAM
        test_write(2'b11, 32'h2002_0000, 32'd868);
        test_write(2'b11, 32'h2002_0004, 32'h12);

        // Wait for busy
        while (rdata[0] != 1) begin
            test_read(32'h2002_000C, 32'h0, rdata);
        end

        // Wait for idle
        while (rdata[0] == 1) begin
            test_read(32'h2002_000C, 32'h0, rdata);
        end

        test_write(2'b11, 32'h2002_0004, 32'h34);

        // Wait for busy
        while (rdata[0] != 1) begin
            test_read(32'h2002_000C, 32'h0, rdata);
        end

        // Wait for idle
        while (rdata[0] == 1) begin
            test_read(32'h2002_000C, 32'h0, rdata);
        end

        // Finish simulation
        #50;
        $finish();
    end

endmodule