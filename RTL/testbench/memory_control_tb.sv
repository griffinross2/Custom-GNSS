/*
  Memory control test bench
*/

`include "cpu_ram_if.vh"
`include "ram_if.vh"
`include "common_types.vh"
import common_types_pkg::*;

`timescale 1 ns / 1 ns

module memory_control_tb;
  // clock period
  parameter PERIOD = 20;

  // signals
  logic clk = 1, nrst;

  // clock
  always #(PERIOD/2) clk++;

  // interface
  ram_if ramif ();
  cpu_ram_if cpuramif ();
  // RAM
  ram ram0 (clk, nrst, ramif);
  // test program
  test PROG (clk, nrst, cpuramif);
  // DUT
  memory_control DUT(nrst, cpuramif, ramif);

endmodule

program test(input logic clk, output logic nrst, cpu_ram_if.cpu cpuramif);

  string test_name = "";

  // TASKS

  // Reset the device
  task reset_dut();
    cpuramif.iren = 0;
    cpuramif.dren = 0;
    cpuramif.dwen = 0;
    cpuramif.iaddr = 32'h0;
    cpuramif.daddr = 32'h0;
    cpuramif.dstore = 32'h0;

    nrst = 1;
    @(posedge clk);
    @(negedge clk);
    nrst = 0;
    @(posedge clk);
    @(negedge clk);
    nrst = 1;
    @(posedge clk);
  endtask

  // Read as the i-cache
  task icache_read(
    input word_t addr,
    input word_t exp_out
  );
    cpuramif.iaddr = addr;
    cpuramif.iren = 1;
    @(negedge clk);
    wait(~cpuramif.iwait);
    @(posedge clk);
    cpuramif.iren = 0;
    if(cpuramif.iload != exp_out) begin
      $display("%s - I-Cache read from address: %04x expected %08x, got %08x", test_name, addr, exp_out, cpuramif.iload);
    end
  endtask

  // Read as the d-cache
  task dcache_read(
    input word_t addr,
    input word_t exp_out
  );
    cpuramif.daddr = addr;
    cpuramif.dren = 1;
    @(negedge clk);
    wait(~cpuramif.dwait)
    @(posedge clk);
    cpuramif.dren = 0;
    if(cpuramif.dload != exp_out) begin
      $display("%s - D-Cache read from address: %04x expected %08x, got %08x", test_name, addr, exp_out, cpuramif.dload);
    end
  endtask

  // Write as the d-cache
  task dcache_write(
    input word_t addr,
    input word_t data
  );
    cpuramif.daddr = addr;
    cpuramif.dstore = data;
    cpuramif.dwen = 1;
    @(negedge clk);
    wait(~cpuramif.dwait)
    @(posedge clk);
    cpuramif.dwen = 0;
  endtask

  // Dump the contents of the RAM into a hex file
  task automatic dump_memory();
    `ifndef SIMULATOR
      string filename = "../../../../ramcpu.hex";
    `else
      string filename = "ramcpu.hex";
    `endif
    int memfd;

    cpu_ram_if.iaddr = 0;
    cpu_ram_if.iren = 0;

    memfd = $fopen(filename,"w");
    if (memfd)
      $display("Starting memory dump.");
    else
      begin $display("Failed to open %s.",filename); $finish; end

    for (int unsigned i = 0; memfd && i < 16384; i++)
    begin
      int chksum = 0;
      bit [7:0][7:0] values;
      string ihex;

      cpu_ram_if.iaddr = i << 2;
      @(posedge clk);
      cpu_ram_if.iren = 1;
      @(negedge clk);
      wait(~cpu_ram_if.iwait);
      if (cpu_ram_if.iload != 0) begin
        values = {8'h04,16'(i<<2),8'h00,cpu_ram_if.iload};
        foreach (values[j])
          chksum += 32'(values[j]);
        chksum = 32'h100 - chksum;
        ihex = $sformatf(":04%h00%h%h",16'(i<<2),cpu_ram_if.iload,8'(chksum));
        $fdisplay(memfd,"%s",ihex.toupper());
      end
    end //for
    if (memfd)
    begin
      cpu_ram_if.iren = 0;
      $fdisplay(memfd,":00000001FF");
      $fclose(memfd);
      $display("Finished memory dump.");
    end
  endtask

  // TEST SEQUENCE
  initial begin
    
    reset_dut();
    
    test_name = "Test 1: I-Cache Read Test";
    icache_read(32'h0, 32'h00010137);
    icache_read(32'h4, 32'h00500293);
    icache_read(32'h8, 32'hFFC10113);
    icache_read(32'hC, 32'h00512023);

    test_name = "Test 2: D-Cache Read Test";
    dcache_read(32'h10, 32'h00400293);
    dcache_read(32'h14, 32'hFFC10113);
    dcache_read(32'h18, 32'h00512023);
    dcache_read(32'h1C, 32'h00300293);

    test_name = "Test 3: D-Cache Write Test";
    dcache_write(32'h10, 32'hE225F7FA);
    dcache_write(32'h14, 32'h72F09CAF);
    dcache_write(32'h18, 32'h79751D39);
    dcache_write(32'h1C, 32'h4CF29807);

    test_name = "Test 4: D-Cache Post-Write Read Test";
    dcache_read(32'h10, 32'hE225F7FA);
    dcache_read(32'h14, 32'h72F09CAF);
    dcache_read(32'h18, 32'h79751D39);
    dcache_read(32'h1C, 32'h4CF29807);

    // Dump memory
    dump_memory();

    #20ns;
    $finish();
  end
endprogram
