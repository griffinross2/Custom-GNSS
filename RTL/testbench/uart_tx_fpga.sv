`timescale 1ns/1ns

module uart_tx_fpga (
    input logic CLK,
    input logic ck_rst,
    output logic UART_TXD
);
    logic clk, nrst;
    logic [7:0] data;
    logic start;
    logic serial_out;
    logic tx_done;

    // 100M / 10416 = 9600 baud rate
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

    // Clock generation
    assign clk = CLK;

    // Reset generation
    assign nrst = ck_rst;

    // Assign ouput to FPGA board
    assign UART_TXD = serial_out;

    assign start = 1'b1;

    // Increment data
    always_ff @(posedge tx_done, negedge nrst) begin
        if (~nrst) begin
            data = 8'h00;
        end else begin
            data = data + 8'd1;
        end
    end

endmodule