`timescale 1ns/1ns

`include "common_types.vh"
import common_types_pkg::*;
`include "axi_controller_if.vh"

module system_tb;
  parameter CLOCK_PERIOD = 10ns;
  parameter GNSS_CLOCK_PERIOD = 52.08333333ns;

  // Clock and reset
  logic clk;
  logic gnssclk;
  logic nrst;

  reg cpuclk;

  initial begin
    clk = 0;
    forever #(CLOCK_PERIOD/2) clk = ~clk;
  end

  initial begin
    gnssclk = 0;
    forever #(GNSS_CLOCK_PERIOD/2) gnssclk = ~gnssclk;
  end

  initial begin
    nrst = 0;
    #200;
    nrst = 1;
  end

  // Clock division
  initial begin
    cpuclk = 1'b0;
    forever begin
      @(posedge clk);
      cpuclk = ~cpuclk;
    end
  end

  // Halt signal
  logic halt;

  // RAM dump IF
  axi_controller_if debug_amif();

  // UART
  logic rxd;
  logic txd;

  // Flash wires
  wire flash_cs;
  wire [3:0] flash_dq;

  // Fake flash
  flash_model flash_inst (
    .clk(cpuclk),
    .nrst(nrst),
    .clk_div(4'd1),
    .cs(flash_cs),
    .dq(flash_dq)
  );

  // System
  system system_inst (
    .clk(cpuclk),
    .nrst(nrst),
    .gnssclk(gnssclk),
    .signal_in(1'b0),
    .rxd(rxd),
    .txd(txd),
    .halt(halt),
    .debug_amif(debug_amif),
    .flash_cs(flash_cs),
    .flash_dq(flash_dq)
  );

  task automatic dump_memory();
  `ifndef SIMULATOR
    string filename = "../../../../cpudump.hex";
  `else
    string filename = "cpudump.hex";
  `endif
    int memfd;

    debug_amif.addr = 0;
    debug_amif.read = 0;

    memfd = $fopen(filename,"w");
    if (memfd != 0)
      $display("Starting memory dump.");
    else
      begin $display("Failed to open %s.",filename); $finish; end

    for (int unsigned i = 0; memfd != 0 && i < 1024; i++)
    begin
      int chksum = 0;
      bit [7:0][7:0] values;
      string ihex;

      debug_amif.addr = i << 2;
      @(posedge cpuclk);
      debug_amif.read = 1;
      wait(debug_amif.ready == 1);
      debug_amif.done = 1;
      if (debug_amif.load != 0) begin
        values = {8'h04,16'(i<<2),8'h00,debug_amif.load};
        foreach (values[j])
          chksum += 32'(values[j]);
        chksum = 32'h100 - chksum;
        ihex = $sformatf(":04%h00%h%h",16'(i<<2),debug_amif.load,8'(chksum));
        $fdisplay(memfd,"%s",ihex.toupper());
      end
      @(posedge cpuclk);
      @(negedge cpuclk);
      debug_amif.read = 0;
      debug_amif.done = 0;
      @(posedge cpuclk);
    end //for
    if (memfd != 0)
    begin
      debug_amif.read = 0;
      $fdisplay(memfd,":00000001FF");
      $fclose(memfd);
      $display("Finished memory dump.");
    end
  endtask

  // Run CPU
  integer num_cycles = 0;
  initial begin
    @(posedge cpuclk);
    @(posedge nrst);

    num_cycles = 0;
  
    while(!halt) begin
      @(posedge cpuclk);
      num_cycles++;
    end

    // Zero memory dump IF
    debug_amif.read = 0;
    debug_amif.write = '0;
    debug_amif.store = '0;
    debug_amif.addr = '0;
    debug_amif.done = '0;

    // Print cycles and time
    $display("CPU halted after %d cycles, %.2f ns", num_cycles, $realtime());

    dump_memory();

    $finish;
  end

endmodule