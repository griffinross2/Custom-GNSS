`include "ram_if.vh"
`include "common_types.vh"
import common_types_pkg::*;

module ram_tb;

    // Signals
    logic clk;
    logic nrst;

    // Interface
    ram_if ram_if();

    // Instantiate the RAM module
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
    task write_ram;
        input word_t addr;
        input word_t value;
        begin
            // Read data fram RAM
            while(!ram_if.ready) begin
                @(posedge clk);
            end
            ram_if.addr = addr;
            ram_if.store = value;
            ram_if.wen = 1;
            @(posedge clk);
            while(!ram_if.ready) begin
                @(posedge clk);
            end
            ram_if.wen = 0;
        end
    endtask

    task read_ram;
        input word_t addr;
        input word_t exp_value;
        begin
            // Read data fram RAM
            while(!ram_if.ready) begin
                @(posedge clk);
            end
            ram_if.addr = addr;
            ram_if.ren = 1;
            @(posedge clk);
            while(!ram_if.ready) begin
                @(posedge clk);
            end
            ram_if.ren = 0;
            assert (ram_if.load == exp_value) 
            else   $fatal("Data mismatch at address %04x, got %08x expected %08x", addr, ram_if.load, exp_value);
        end
    endtask

    // Test sequence
    integer i;
    initial begin
        // Initialize signals
        ram_if.addr = '0;
        ram_if.store = '0;
        ram_if.wen = 0;
        ram_if.ren = 0;

        // Reset the DUT
        #10 nrst = 0;
        #10 nrst = 1;

        @(posedge clk);

        // First fill RAM
        for(i=0; i<ram_inst.RAM_SIZE*4; i+=4) begin
            write_ram(i, i);
        end

        // Then read RAM
        for(i=0; i<ram_inst.RAM_SIZE*4; i+=4) begin
            read_ram(i, i);
        end

        // Finish simulation
        #10 $finish;
    end

    // Monitor signals
    initial begin
        $monitor("Time: %0t | addr: %h | data_out: %h", $time, ram_if.addr, ram_if.load, ram_if.store);
    end

endmodule