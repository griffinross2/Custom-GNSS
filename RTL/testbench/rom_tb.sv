`include "ram_if.vh"
`include "common_types.vh"
import common_types_pkg::*;

module rom_tb;

    // Signals
    logic clk;
    logic nrst;

    // Interface
    ram_if rom_if();

    // Instantiate the rom module
    rom rom_inst (
        .clk(clk),
        .nrst(nrst),
        .ram_if(rom_if)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // Tasks
    task write_rom;
        input word_t addr;
        input word_t value;
        begin
            // Read data from rom
            while(rom_if.state == RAM_WAIT) begin
                @(posedge clk);
            end
            rom_if.addr = addr;
            rom_if.store = value;
            rom_if.wen = '1;
            @(posedge clk);
            while(rom_if.state == RAM_WAIT) begin
                @(posedge clk);
            end
            rom_if.wen = '0;
        end
    endtask

    task read_rom;
        input word_t addr;
        input word_t exp_value;
        begin
            // Read data from rom
            while(rom_if.state == RAM_WAIT) begin
                @(posedge clk);
            end
            rom_if.addr = addr;
            rom_if.ren = 1;
            @(posedge clk);
            while(rom_if.state == RAM_WAIT) begin
                @(posedge clk);
            end
            rom_if.ren = 0;
            assert (rom_if.load == exp_value) 
            else   $fatal("Data mismatch at address %04x, got %08x expected %08x", addr, rom_if.load, exp_value);
        end
    endtask

    // Test sequence
    integer i;
    initial begin
        // Initialize signals
        rom_if.addr = '0;
        rom_if.store = '0;
        rom_if.wen = 0;
        rom_if.ren = 0;

        // Reset the DUT
        #10 nrst = 0;
        #10 nrst = 1;

        @(posedge clk);

        // First fill rom
        for(i=0; i<rom_inst.RAM_SIZE*4; i+=4) begin
            write_rom(i, i);
        end

        // Then read rom
        for(i=0; i<rom_inst.RAM_SIZE*4; i+=4) begin
            read_rom(i, i);
        end

        // Finish simulation
        #10 $finish;
    end

    // Monitor signals
    initial begin
        $monitor("Time: %0t | addr: %h | data_out: %h", $time, rom_if.addr, rom_if.load, rom_if.store);
    end

endmodule