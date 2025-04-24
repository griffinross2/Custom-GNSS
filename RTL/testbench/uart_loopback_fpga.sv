`timescale 1ns/1ns

module uart_loopback_fpga (
    input logic CLK,
    input logic ck_rst,
    input logic uart_txd_in,
    output logic UART_TXD,
    output logic LED [0:3]
);
    logic clk, nrst;
    logic [7:0] rx_data;
    logic [7:0] rx_data_lat;
    logic start;
    logic tx_done, tx_busy;
    logic rx_done, rx_busy;
    logic rx_done_last;
    logic rx_done_reg;

    // 100M / 10416 = 9600 baud rate
    localparam int BIT_PERIOD = 10417; // Bit period in clk cycles

    uart_tx dut_tx (
        .clk(clk),
        .nrst(nrst),
        .bit_period(BIT_PERIOD),
        .data(rx_data_lat),
        .start(start),
        .serial_out(UART_TXD),
        .tx_busy(tx_busy),
        .tx_done(tx_done)
    );

    uart_rx dut_rx (
        .clk(clk),
        .nrst(nrst),
        .bit_period(BIT_PERIOD),
        .data(rx_data),
        .serial_in(uart_txd_in),
        .rx_busy(rx_busy),
        .rx_done(rx_done)
    );

    // Clock generation
    assign clk = CLK;

    // Reset generation
    assign nrst = ck_rst;

    assign LED[0] = tx_done;
    assign LED[1] = tx_busy;
    assign LED[2] = rx_done;
    assign LED[3] = rx_busy;

    always_ff @(posedge clk) begin
        if (~nrst) begin
            rx_done_last <= 1'b0;
            rx_done_reg <= 1'b0;
        end else begin
            rx_done_last <= rx_done;

            if (rx_done & ~rx_done_last) begin
                // Capture the rx_done signal on the rising edge
                rx_done_reg <= 1'b1;
            end else if (start) begin
                // Clear the rx_done_reg a tx is started (we acknowledged it)
                rx_done_reg <= 1'b0;
            end else begin
                rx_done_reg <= rx_done_reg;
            end
        end
    end

    always_ff @(posedge clk) begin
        if (~nrst) begin
            start <= 0;
        end else begin
            if (rx_done_reg & ~tx_busy & ~start) begin
                // Start when data is received
                start <= 1'b1;
            end else if (tx_busy & start) begin
                // Stop asserting start when transmission is busy
                start <= 1'b0;
            end else begin
                start <= start;
            end
        end
    end

    always_ff @(posedge clk) begin
        if (~nrst) begin
            rx_data_lat <= 8'h00;
        end else begin
            if (rx_done) begin
                // Capture received data
                rx_data_lat <= rx_data;
            end else begin
                rx_data_lat <= rx_data_lat;
            end
        end
    end

endmodule