`timescale 1ns/1ns

module uart_tx_tb (
);
    logic clk, nrst, nrst_gen;
    logic [7:0] data;
    logic start;
    logic serial_out;
    logic tx_done;

    localparam int BIT_PERIOD = 10417; // Bit period in clk cycles

    uart_tx dut (
        .clk(clk),
        .nrst(nrst),
        .bit_period(BIT_PERIOD),
        .data(data),
        .start(start),
        .serial_out(serial_out),
        .tx_done(tx_done)
    );

    test PROG (
        .clk(clk),
        .serial_out(serial_out),
        .tx_done(tx_done),
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

    assign start = 1'b1;

endmodule

program test (input logic clk, input logic serial_out, input logic tx_done, output logic nrst_gen, output logic [7:0] data, output logic start);
    string test_name = "";

    task reset_dut();
        data = 8'h00;
        start = 0;

        nrst_gen = 0;
        @(posedge clk);
        @(posedge clk);
        nrst_gen = 1;
        @(posedge clk);
    endtask

    task send_data(input logic [7:0] data_in);
        data = data_in;
        start = 1;
        wait(~tx_done);
        wait(tx_done);
        start = 0;
        @(posedge clk);
    endtask
        
    initial begin
        test_name = "UART TX Test";
        
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