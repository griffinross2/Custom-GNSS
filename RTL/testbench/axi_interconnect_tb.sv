`timescale 1ns / 1ns

`include "axi_controller_if.vh"
`include "axi_bus_if.vh"
`include "ram_if.vh"
`include "common_types.vh"
import common_types_pkg::*;

module axi_interconnect_tb ();

    // Signals
    logic clk;
    logic nrst;

    // Interface
    axi_controller_if amif_icache ();
    axi_bus_if abif_icache ();
    axi_controller_if amif_dcache ();
    axi_bus_if abif_dcache ();
    axi_controller_if amif_dma ();
    axi_bus_if abif_dma ();
    
    axi_bus_if abif_sram ();
    axi_bus_if abif_flash ();
    axi_bus_if abif_dram ();
    axi_bus_if abif_ahb ();

    axi_controller icache_inst (
        .clk(clk),
        .nrst(nrst),
        .amif(amif_icache),
        .abif(abif_icache)
    );

    axi_controller dcache_inst (
        .clk(clk),
        .nrst(nrst),
        .amif(amif_dcache),
        .abif(abif_dcache)
    );

    axi_controller dma_inst (
        .clk(clk),
        .nrst(nrst),
        .amif(amif_dma),
        .abif(abif_dma)
    );

    axi_interconnect axi_inst (
        .clk(clk),
        .nrst(nrst),
        .abif_to_icache(abif_icache),
        .abif_to_dcache(abif_dcache),
        .abif_to_dma(abif_dma),
        .abif_to_sram(abif_sram),
        .abif_to_flash(abif_flash),
        .abif_to_dram(abif_dram),
        .abif_to_ahb(abif_ahb)
    );

    axi_flash_controller flash_inst (
        .clk(clk),
        .nrst(nrst),
        .abif(abif_flash),
        .clk_div(4'd0)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // Tasks
    initial begin
        amif_icache.read = 0;
        amif_icache.write = 0;
        amif_icache.addr = 0;
        amif_icache.store = 0;
        amif_icache.done = 0;

        amif_dcache.read = 0;
        amif_dcache.write = 0;
        amif_dcache.addr = 0;
        amif_dcache.store = 0;
        amif_dcache.done = 0;

        amif_dma.read = 0;
        amif_dma.write = 0;
        amif_dma.addr = 0;
        amif_dma.store = 0;
        amif_dma.done = 0;

        abif_sram.awready = 1'b0;
        abif_sram.wready = 1'b0;
        abif_sram.bid = 4'b0;
        abif_sram.bresp = 2'b00;
        abif_sram.bvalid = 1'b0;
        abif_sram.arready = 1'b0;
        abif_sram.rid = 4'b0;
        abif_sram.rdata = 32'b0;
        abif_sram.rresp = 2'b00;
        abif_sram.rvalid = 1'b0;
        abif_sram.rlast = 1'b0;

        abif_dram.awready = 1'b0;
        abif_dram.wready = 1'b0;
        abif_dram.bid = 4'b0;
        abif_dram.bresp = 2'b00;
        abif_dram.bvalid = 1'b0;
        abif_dram.arready = 1'b0;
        abif_dram.rid = 4'b0;
        abif_dram.rdata = 32'b0;
        abif_dram.rresp = 2'b00;
        abif_dram.rvalid = 1'b0;
        abif_dram.rlast = 1'b0;

        abif_ahb.awready = 1'b0;
        abif_ahb.wready = 1'b0;
        abif_ahb.bid = 4'b0;
        abif_ahb.bresp = 2'b00;
        abif_ahb.bvalid = 1'b0;
        abif_ahb.arready = 1'b0;
        abif_ahb.rid = 4'b0;
        abif_ahb.rdata = 32'b0;
        abif_ahb.rresp = 2'b00;
        abif_ahb.rvalid = 1'b0;
        abif_ahb.rlast = 1'b0;

        nrst = 0;
        @(posedge clk);
        @(posedge clk);
        @(posedge clk);
        nrst = 1;

        amif_icache.read = 1;
        amif_icache.write = 0;
        amif_icache.addr = 32'h0081_0000;
        amif_icache.store = 32'h0000_0000;
        amif_icache.done = 0;

        amif_dcache.read = 1;
        amif_dcache.write = 0;
        amif_dcache.addr = 32'h0082_0000;
        amif_dcache.store = 32'h0000_0000;
        amif_dcache.done = 0;

        amif_dma.read = 1;
        amif_dma.write = 0;
        amif_dma.addr = 32'h0083_0000;
        amif_dma.store = 32'h0000_0000;
        amif_dma.done = 0;

        // Wait for first request to finish
        wait (amif_icache.ready == 1 || 
                amif_dcache.ready == 1 || 
                amif_dma.ready == 1);

        if (amif_icache.ready == 1) begin
            $display("First request finished for ICache");
            amif_icache.done = 1;
            @(posedge clk);
            amif_icache.done = 0;
            @(posedge clk);
        end

        if (amif_dcache.ready == 1) begin
            $display("First request finished for DCache");
            amif_dcache.done = 1;
            @(posedge clk);
            amif_dcache.done = 0;
            @(posedge clk);
        end

        if (amif_dma.ready == 1) begin
            $display("First request finished for DMA");
            amif_dma.done = 1;
            @(posedge clk);
            amif_dma.done = 0;
            @(posedge clk);
        end

        // Wait for second request to finish
        wait (amif_icache.ready == 1 || 
                amif_dcache.ready == 1 || 
                amif_dma.ready == 1);

        if (amif_icache.ready == 1) begin
            $display("Second request finished for ICache");
            amif_icache.done = 1;
            @(posedge clk);
            amif_icache.done = 0;
            @(posedge clk);
        end

        if (amif_dcache.ready == 1) begin
            $display("Second request finished for DCache");
            amif_dcache.done = 1;
            @(posedge clk);
            amif_dcache.done = 0;
            @(posedge clk);
        end

        if (amif_dma.ready == 1) begin
            $display("Second request finished for DMA");
            amif_dma.done = 1;
            @(posedge clk);
            amif_dma.done = 0;
            @(posedge clk);
        end

        // Wait for third request to finish
        wait (amif_icache.ready == 1 || 
                amif_dcache.ready == 1 || 
                amif_dma.ready == 1);

        if (amif_icache.ready == 1) begin
            $display("Third request finished for ICache");
            amif_icache.done = 1;
            @(posedge clk);
            amif_icache.done = 0;
            @(posedge clk);
        end

        if (amif_dcache.ready == 1) begin
            $display("Third request finished for DCache");
            amif_dcache.done = 1;
            @(posedge clk);
            amif_dcache.done = 0;
            @(posedge clk);
        end

        if (amif_dma.ready == 1) begin
            $display("Third request finished for DMA");
            amif_dma.done = 1;
            @(posedge clk);
            amif_dma.done = 0;
            @(posedge clk);
        end

        #10ns;
        $finish;
    end

endmodule