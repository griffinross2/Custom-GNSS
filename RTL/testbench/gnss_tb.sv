`timescale 1ns/1ns

`include "common_gnss_types.vh"
import common_gnss_types_pkg::*;
`include "common_types.vh"
import common_types_pkg::*;
`include "ahb_bus_if.vh"
`include "axi_bus_if.vh"
`include "axi_controller_if.vh"

module gnss_tb;
    logic gnssclk, cpuclk, nrst;                    // Clock and reset
    logic signal_in;
    ahb_bus_if ahb_abif (); // AHB bus interface
    axi_bus_if axi_abif (); // AXI bus interface
    axi_controller_if axi_amif ();

    gnss dut (
        .gnssclk(gnssclk), 
        .cpuclk(cpuclk),
        .nrst(nrst),
        .signal_in(signal_in),
        .abif(ahb_abif)
    );

    axi_controller axi (
        .clk(cpuclk),
        .nrst(nrst),
        .abif(axi_abif),
        .amif(axi_amif)
    );

    axi_to_ahb_bridge axi_to_ahb (
        .clk(cpuclk),
        .nrst(nrst),
        .axi(axi_abif),
        .ahb(ahb_abif)
    );

    assign ahb_abif.hready = ahb_abif.hreadyout;
    assign ahb_abif.hsel = 1'b1;

    initial begin
        gnssclk = 0;
        forever #26.0417 gnssclk = ~gnssclk;
    end
    initial begin
        cpuclk = 0;
        forever #10 cpuclk = ~cpuclk;
    end

    task test_write;
        input logic [1:0] size;
        input word_t addr;
        input word_t wdata;
        begin
            // Set signals to controller
            axi_amif.write = size;
            axi_amif.addr = addr;
            axi_amif.store = wdata;

            // Wait for transaction to complete
            wait(axi_amif.ready == 1);
            axi_amif.done = 1;
            @(posedge cpuclk);

            // Set bus signals to idle
            axi_amif.done = 0;
            axi_amif.write = '0;
            axi_amif.addr = 0;
            @(posedge cpuclk);
        end
    endtask

    task test_read;
        input word_t addr;
        input word_t exp_rdata;
        output word_t rdata;
        begin
            // Set signals to controller
            axi_amif.read = 1;
            axi_amif.addr = addr;

            // Wait for transaction to complete
            wait(axi_amif.ready == 1);
            axi_amif.done = 1;
            rdata = axi_amif.load;
            @(posedge cpuclk);

            // Set bus signals to idle
            axi_amif.done = 0;
            axi_amif.read = 0;
            axi_amif.addr = 0;
            @(posedge cpuclk);
        end
    endtask

    integer fd;
    logic [2:0] bit_count;
    logic [7:0] signal_byte;
    initial begin
        nrst = 0;
        signal_in = 0;
        bit_count = 0;

        axi_amif.read = 0;
        axi_amif.write = '0;
        axi_amif.addr = '0;
        axi_amif.store = '0;
        axi_amif.done = 0;

        #200 nrst = 1;
        
        @(posedge cpuclk);

        // Configure search
        test_write(2'b11, 32'h2004_0104, 32'd25 + (32'd0 << 8));
        // Start search
        test_write(2'b11, 32'h2004_0100, 32'd1);

        @(posedge gnssclk);

        fd = $fopen("../../../../signal.bin", "rb");
        if (fd == 0) begin
            $display("Error opening signal.bin");
            $finish;
        end

        // Read the input signal from the binary file
        while (!$feof(fd)) begin
            @(negedge gnssclk);
            if (bit_count == 0) begin
                // Read byte every 8 bits
                signal_byte = $fgetc(fd);
            end

            signal_in = signal_byte[bit_count];
            bit_count = bit_count + 1;
        end
        
        #200ms;

        #10 $finish; // End simulation
    end

endmodule