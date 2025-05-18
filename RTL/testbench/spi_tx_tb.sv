`timescale 1ns/1ns

module spi_tx_tb (
);
    logic clk, nrst, nrst_gen;
    logic [31:0] data;
    logic start;
    logic sdo;
    logic sck;
    logic cs;
    logic busy;

    localparam int BIT_PERIOD = 1000; // Bit period

    spi_tx dut (
        .clk(clk),
        .nrst(nrst),
        .bit_period(BIT_PERIOD[15:0]),
        .data_width(5'd7),
        .data(data),
        .start(start),
        .sdo(sdo),
        .sck(sck),
        .cs(cs),
        .busy(busy)
    );

    test PROG (
        .clk(clk),
        .sdo(sdo),
        .sck(sck),
        .cs(cs),
        .busy(busy),
        .nrst_gen(nrst_gen),
        .data(data),
        .start(start)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // Reset generation
    assign nrst = nrst_gen;

endmodule

program test (input logic clk, input logic sdo, input logic sck, input logic cs, input logic busy, output logic nrst_gen, output logic [31:0] data, output logic start);
    string test_name = "";

    task reset_dut();
        data = 32'h0;
        start = 0;

        nrst_gen = 0;
        @(posedge clk);
        @(posedge clk);
        nrst_gen = 1;
        @(posedge clk);
    endtask

    task send_data(input logic [7:0] data_in);
        data = {24'd0, data_in};
        start = 1;
        wait(busy);
        wait(~busy);
        start = 0;
        @(posedge clk);
    endtask
        
    initial begin
        test_name = "SPI TX Test";
        
        // Reset the DUT
        reset_dut();

        // Send data
        send_data(8'hA5);
        
        send_data(8'h1F);

        send_data(8'h00);

        send_data(8'hFF);

        $display("%s completed successfully", test_name);

        $finish;
    end

endprogram