`timescale 1ns / 1ns

`include "ahb_controller_if.vh"
`include "ahb_bus_if.vh"
`include "ram_if.vh"
`include "common_types.vh"
import common_types_pkg::*;

module ahb_controller_tb ();

    // Signals
    logic clk;
    logic nrst;

    // Interface
    ahb_controller_if amif ();
    ahb_bus_if abif_controller ();
    ahb_bus_if abif_satellite_def ();
    ahb_bus_if abif_satellite_ram ();
    ram_if ram_if ();

    ahb_controller ahb_inst (
        .clk(clk),
        .nrst(nrst),
        .amif(amif),
        .abif(abif_controller)
    );

    ahb_multiplexor ahb_mux (
        .clk(clk),
        .nrst(nrst),
        .abif_to_controller(abif_controller),
        .abif_to_def(abif_satellite_def),
        .abif_to_ram(abif_satellite_ram)
    );

    ahb_default_satellite ahb_satellite_def (
        .clk(clk),
        .nrst(nrst),
        .abif(abif_satellite_def)
    );

    memory_control ahb_satellite_ram (
        .clk(clk),
        .nrst(nrst),
        .ahb_bus_if(abif_satellite_ram),
        .ram_if(ram_if)
    );

    ram ram_inst (
        .clk(clk),
        .nrst(nrst),
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
            amif.iread = 0;
            amif.dread = 0;
            amif.dwrite = 0;
            amif.iaddr = 0;
            amif.daddr = 0;
            amif.dstore = 0;
            abif_controller.hrdata = 0;
            abif_controller.hready = 1;
            abif_controller.hresp = 0;
            nrst = 0;
            @(posedge clk);
            nrst = 1;
        end
    endtask
    
    task test_transfer;
        input logic iread, dread;
        input logic [1:0] dwrite;
        input word_t iaddr, daddr, dstore;
        input word_t rdata_test;
        word_t rdata;
        begin
            // Set signals to controller
            amif.iread = iread;
            amif.dread = dread;
            amif.dwrite = dwrite;
            amif.iaddr = iaddr;
            amif.daddr = daddr;
            amif.dstore = dstore;

            // Go to start of data phase
            @(posedge clk);
            // Set signals to controller to idle
            amif.iread = '0;
            amif.dread = '0;
            amif.dwrite = '0;
            amif.iaddr = '0;
            amif.daddr = '0;
            amif.dstore = '0;
            // Respond
            abif_controller.hrdata = rdata_test;
            // Finish transaction
            @(negedge clk);
            if (iread) begin
                if (amif.iload != rdata_test) begin
                $display("Test failed: Expected 0x%08h, got 0x%08h", rdata_test, abif_controller.hrdata);
            end
            end else begin
                if (amif.dload != rdata_test) begin
                $display("Test failed: Expected 0x%08h, got 0x%08h", rdata_test, abif_controller.hrdata);
            end
            end
            @(posedge clk);
            // Set bus signals to idle
            abif_controller.hready = 1;
            abif_controller.hresp = 0;
        end
    endtask

    // Test sequence
    initial begin
        reset_dut;

        @(posedge clk);

        // Store and read from RAM
        test_transfer(0, 0, 2'b11, 32'h00000000, 32'h00000004, 32'h01234567, 32'h00000000);
        test_transfer(1, 0, 0, 32'h00000004, 32'h00000000, 32'h00000000, 32'h01234567);

        // Finish simulation
        #50;
        $finish();
    end

endmodule