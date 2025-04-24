`timescale 1ns/1ns

`include "common_types.vh"
import common_types_pkg::*;
`include "ram_dump_if.vh"

module system_tb;
  parameter CLOCK_PERIOD = 10ns;

  // Clock and reset
  logic clk;
  logic nrst;

  // Halt signal
  logic halt;

  // RAM dump IF
  ram_dump_if cpu_ram_if();

  // UART
  logic rxd;
  logic txd;

  // System
  system system_inst (
    .clk(clk),
    .nrst(nrst),
    .rxd(rxd),
    .txd(txd),
    .halt(halt),
    .cpu_ram_debug_if(cpu_ram_if)
  );

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
    if (memfd != 0)
      $display("Starting memory dump.");
    else
      begin $display("Failed to open %s.",filename); $finish; end

    for (int unsigned i = 0; memfd != 0 && i < 16384; i++)
    begin
      int chksum = 0;
      bit [7:0][7:0] values;
      string ihex;

      cpu_ram_if.iaddr = i << 2;
      @(posedge clk);
      cpu_ram_if.iren = 1;
      @(posedge clk);
      cpu_ram_if.iren = 0;
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
    if (memfd != 0)
    begin
      cpu_ram_if.iren = 0;
      $fdisplay(memfd,":00000001FF");
      $fclose(memfd);
      $display("Finished memory dump.");
    end
  endtask

  // Clock generation
  initial begin
    clk = 0;
    forever #(CLOCK_PERIOD/2) clk = ~clk;
  end

  // Run CPU
  integer num_cycles = 0;
  initial begin
    rxd = 1;
    cpu_ram_if.override_ctrl = 0;
    nrst = 1;
    #10;
    nrst = 0;
    #10;
    nrst = 1;

    num_cycles = 0;
  
    while(!halt) begin
      // if(num_cycles == 1000) begin
      //   rxd = 0; // Send start bit to trigger interrupt
      // end else begin
      //   rxd = 1;
      // end

      @(posedge clk);
      num_cycles++;
    end

    // Zero memory dump IF
    cpu_ram_if.iren = 0;
    cpu_ram_if.dren = 0;
    cpu_ram_if.dwen = 0;
    cpu_ram_if.dstore = 0;
    cpu_ram_if.iaddr = 0;
    cpu_ram_if.daddr = 0;

    // Take control of the memory
    cpu_ram_if.override_ctrl = 1;

    // Print cycles and time
    $display("CPU halted after %d cycles, %.2f ns",num_cycles, $realtime());

    dump_memory();

    $finish;
  end

endmodule