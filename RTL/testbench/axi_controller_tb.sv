`timescale 1ns / 1ns

`include "axi_controller_if.vh"
`include "axi_bus_if.vh"
`include "ram_if.vh"
`include "common_types.vh"
import common_types_pkg::*;

module axi_controller_tb ();

    // Signals
    logic clk;
    logic nrst;

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
            @(posedge clk);
            @(posedge clk);
            nrst = 1;
        end
    endtask
    
    task test_write;
        input logic [1:0] size;
        input word_t addr;
        input word_t store;
        input integer delay_addr;
        input integer delay_data;
        logic [2:0] exp_wstrb;
        begin
            // Set signals to controller
            amif.write = size;
            amif.addr = addr;
            amif.store = store;

            // Figure out expected write strobe
            case (size)
                2'b01: exp_wstrb = 4'b0001; // byte
                2'b10: exp_wstrb = 4'b0011; // halfword
                2'b11: exp_wstrb = 4'b1111; // word
                default: exp_wstrb = 4'b0000; // invalid
            endcase

            // After 1 clock cycles, should be in write address state
            @(posedge clk);
            @(negedge clk);
            if (abif_controller.awvalid) begin
                if (abif_controller.awaddr != addr) begin
                    $display("Write failed: awaddr expected 0x%08h, got 0x%08h", addr, abif_controller.awaddr);
                end
            end else begin
                $display("Write failed: awvalid not asserted");
            end

            // After 1 + delay_addr clock cycles, should be in write data state
            if (delay_addr > 0) begin
                abif_controller.awready = 0;
                for (int i = 0; i < delay_addr; i++) begin
                    @(negedge clk);
                end
                abif_controller.awready = 1;
            end
            @(negedge clk);
            if (abif_controller.wvalid) begin
                if (abif_controller.wdata != store) begin
                    $display("Write failed: wdata expected 0x%08h, got 0x%08h", store, abif_controller.wdata);
                end
                if (abif_controller.wstrb != exp_wstrb) begin
                    $display("Write failed: wstrb expected 0x%04b, got 0x%04b", exp_wstrb, abif_controller.wstrb);
                end
            end else begin
                $display("Write failed: wvalid not asserted");
            end

            // After 1 + delay_data clock cycles, should be in write response state
            if (delay_data > 0) begin
                abif_controller.wready = 0;
                for (int i = 0; i < delay_data; i++) begin
                    @(negedge clk);
                end
                abif_controller.wready = 1;
            end
            @(negedge clk);
            // Respond
            if (~abif_controller.bready) begin
                $display("Write failed: bready not asserted");
            end
            @(negedge clk);
            // Signal done
            if (!amif.ready) begin
                $display("Write failed: did not assert ready to controller");
            end
            amif.done = 1;

            @(negedge clk);
            // Should be in idle state
            if (amif.ready) begin
                $display("Write failed: did not deassert ready to controller");
            end

            // Set bus signals to idle
            amif.done = 0;
            amif.write = 0;
            amif.addr = 0;
            amif.store = 0;
        end
    endtask

    task test_read;
        input word_t addr;
        input integer delay_addr;
        input integer delay_data;
        input word_t exp_rdata;
        begin
            // Set signals to controller
            amif.read = 1;
            amif.addr = addr;

            // After 1 clock cycles, should be in read address state
            @(posedge clk);
            @(negedge clk);
            if (abif_controller.arvalid) begin
                if (abif_controller.araddr != addr) begin
                    $display("Read failed: araddr expected 0x%08h, got 0x%08h", addr, abif_controller.araddr);
                end
            end else begin
                $display("Read failed: arvalid not asserted");
            end

            // After 1 + delay_addr clock cycles, should be in read data state
            if (delay_addr > 0) begin
                abif_controller.arready = 0;
                for (int i = 0; i < delay_addr; i++) begin
                    @(negedge clk);
                end
                abif_controller.arready = 1;
            end
            @(negedge clk);
            if (abif_controller.rready) begin
                if (abif_controller.rdata != exp_rdata) begin
                    $display("Read failed: rdata expected 0x%08h, got 0x%08h", exp_rdata, abif_controller.rdata);
                end
            end else begin
                $display("Read failed: rready not asserted");
            end

            // After 1 + delay_data clock cycles, should be in write response state
            if (delay_data > 0) begin
                abif_controller.rvalid = 0;
                for (int i = 0; i < delay_data; i++) begin
                    @(negedge clk);
                end
                abif_controller.rvalid = 1;
            end
            @(negedge clk);
            // Signal done
            if (!amif.ready) begin
                $display("Read failed: did not assert ready to controller");
            end
            amif.done = 1;

            @(negedge clk);
            // Should be in idle state
            if (amif.ready) begin
                $display("Read failed: did not deassert ready to controller");
            end

            // Set bus signals to idle
            amif.done = 0;
            amif.read = 0;
            amif.addr = 0;
        end
    endtask

    // Test sequence
    initial begin
        reset_dut;

        @(posedge clk);

        // Store and read from RAM
        test_write(2'b10, 32'h00000020, 32'h1234ABCD, 1, 2);
        test_read(32'h00000020, 0, 0, 32'h0000ABCD);

        // Finish simulation
        #50;
        $finish();
    end

endmodule