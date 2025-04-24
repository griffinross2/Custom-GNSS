`timescale 1ns / 1ns

`include "ahb_controller_if.vh"
`include "ahb_bus_if.vh"
`include "common_types.vh"
import common_types_pkg::*;

module ahb_gnss_satellite_tb ();

    // Signals
    logic clk;
    logic hclk;
    logic nrst;

    logic [31:0] epoch;
    logic snapshot;

    // Interface
    ahb_controller_if amif ();
    ahb_bus_if abif_controller ();
    ahb_bus_if abif_satellite_def ();
    ahb_bus_if abif_satellite_gnss ();

    ahb_controller ahb_inst (
        .clk(hclk),
        .nrst(nrst),
        .amif(amif),
        .abif(abif_controller)
    );

    ahb_multiplexor ahb_mux (
        .clk(hclk),
        .nrst(nrst),
        .abif_to_controller(abif_controller),
        .abif_to_def(abif_satellite_def),
        .abif_to_gnss(abif_satellite_gnss)
    );

    ahb_default_satellite ahb_satellite_def (
        .clk(hclk),
        .nrst(nrst),
        .abif(abif_satellite_def)
    );

    ahb_gnss_satellite ahb_satellite_gnss (
        .clk(clk),
        .hclk(hclk),
        .nrst(nrst),
        .abif(abif_satellite_gnss),
        .epoch(epoch),
        .snapshot(snapshot)
    );

    // HCLK clock generation (100 MHz)
    initial begin
        hclk = 0;
        forever #5 hclk = ~hclk;
    end

    // CLK clock generation (19.2 MHz)
    initial begin
        clk = 0;
        forever #26.04166667 clk = ~clk; // 19.2 MHz
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
            epoch = '0;
            nrst = 0;
            @(posedge hclk);
            @(posedge clk);
            @(posedge clk);
            nrst = 1;
            @(posedge clk);
            @(posedge hclk);
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
            @(posedge hclk);
            // Set request signals to idle
            amif.iread = '0;
            amif.dread = '0;
            amif.dwrite = '0;
            // Finish transaction
            if (iread) begin
                wait(amif.ihit);
            end else begin
                wait(amif.dhit);
            end
            @(posedge hclk);
            // Set signals to controller to idle
            amif.iaddr = '0;
            amif.daddr = '0;
            amif.dstore = '0;

            if (iread) begin
                if (amif.iload != rdata_test) begin
                $display("Test failed: Expected 0x%08h, got 0x%08h", rdata_test, abif_controller.hrdata);
            end
            end else begin
                if (amif.dload != rdata_test) begin
                $display("Test failed: Expected 0x%08h, got 0x%08h", rdata_test, abif_controller.hrdata);
            end
            end
            @(posedge hclk);
            // Set bus signals to idle
            abif_controller.hready = 1;
            abif_controller.hresp = 0;
        end
    endtask

    // Test sequence
    initial begin
        reset_dut;

        @(posedge hclk);

        // Read from GNSS (expected to be followed with instruction reads)
        test_transfer(0, 1, 0, 32'h00000000, 32'h00040000, 32'h00000000, 32'h00000000);
        test_transfer(1, 0, 0, 32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000);

        test_transfer(0, 1, 0, 32'h00000000, 32'h00040004, 32'h00000000, 32'h00000000);
        test_transfer(1, 0, 0, 32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000);

        // Write to CR register
        test_transfer(0, 0, 2'b11, 32'h00000000, 32'h00040004, 32'h00000001, 32'h00000000);
        test_transfer(1, 0, 0, 32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000);

        // Trigger some epochs

        @(negedge clk);
        epoch = 32'h2139;
        @(negedge clk);
        epoch = 32'h0;
        @(posedge clk);
        @(posedge clk);

        // Read epochs
        test_transfer(0, 1, 0, 32'h00000000, 32'h00040000, 32'h00000000, 32'h2139);
        test_transfer(1, 0, 0, 32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000);

        // Clear epochs
        test_transfer(0, 0, 2'b11, 32'h00000000, 32'h00040000, 32'h00000039, 32'h00000000);
        test_transfer(1, 0, 0, 32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000);

        // Read epochs again
        test_transfer(0, 1, 0, 32'h00000000, 32'h00040000, 32'h00000000, 32'h2100);
        test_transfer(1, 0, 0, 32'h00000000, 32'h00000000, 32'h00000000, 32'h00000000);
 
        // Finish simulation
        #50;
        $finish();
    end

endmodule