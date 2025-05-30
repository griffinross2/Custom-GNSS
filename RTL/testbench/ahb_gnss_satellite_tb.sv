`timescale 1ns / 1ns

`include "axi_controller_if.vh"
`include "axi_bus_if.vh"
`include "ahb_bus_if.vh"
`include "common_types.vh"
import common_types_pkg::*;

module ahb_gnss_satellite_tb ();

    // Signals
    logic clk;
    logic hclk;
    logic nrst;

    logic search_busy;
    logic search_start;
    sv_t search_sv;
    word_t search_dop;
    word_t search_code;

    // Interface
    axi_controller_if amif ();
    axi_bus_if axi_to_bridge ();
    ahb_bus_if abif_bridge ();
    ahb_bus_if abif_satellite_def ();
    ahb_bus_if abif_satellite_uart ();
    ahb_bus_if abif_satellite_gnss ();

    axi_controller axi_inst (
        .clk(hclk),
        .nrst(nrst),
        .amif(amif),
        .abif(axi_to_bridge)
    );

    axi_to_ahb_bridge axi_bridge (
        .clk(hclk),
        .nrst(nrst),
        .axi(axi_to_bridge),
        .ahb(abif_bridge)
    );

    ahb_multiplexor ahb_mux (
        .clk(hclk),
        .nrst(nrst),
        .abif_to_controller(abif_bridge),
        .abif_to_uart(abif_satellite_uart),
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
        .search_busy(search_busy),
        .search_start(search_start),
        .search_sv(search_sv),
        .search_dop(search_dop),
        .search_code(search_code)
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
            search_dop = 1234;
            search_code = 5678;
            search_busy = 1;

            amif.read = 0;
            amif.write = '0;
            amif.addr = 0;
            amif.store = 0;
            amif.done = 0;
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
        input logic read;
        input logic [1:0] write;
        input word_t addr, store;
        input word_t rdata_test;
        word_t rdata;
        begin
            // Set signals to controller
            amif.read = read;
            amif.write = write;
            amif.addr = addr;
            amif.store = store;
            amif.done = 0;

            // Go to start of data phase
            @(posedge hclk);
            // Finish transaction
            wait(amif.ready);
            amif.done = 1;

            @(negedge hclk);
            if (read) begin
                if (amif.load != rdata_test) begin
                    $display("Test failed: Expected 0x%08h, got 0x%08h", rdata_test, amif.load);
                end
            end

            @(negedge hclk);
            // Set signals to controller to idle
            amif.read = '0;
            amif.write = '0;
            amif.addr = '0;
            amif.store = '0;
            amif.done = 0;
            
            @(posedge hclk);
        end
    endtask

    // Test sequence
    initial begin
        reset_dut;

        @(posedge hclk);

        // Read from GNSS (expected to be followed with instruction reads)
        test_transfer(1, 0, 32'h20040100, 32'h00000000, 32'h00000000);

        test_transfer(1, 0, 32'h20040104, 32'h00000000, 32'h00000000);

        test_transfer(0, 2'b11, 32'h20040104, 32'h00000011, 32'h00000000);
        
        test_transfer(1, 0, 32'h20040104, 32'h00000000, 32'h00000011);
 
        // Finish simulation
        #50;
        $finish();
    end

endmodule