`timescale 1ns / 1ns

`include "axi_controller_if.vh"
`include "axi_bus_if.vh"
`include "ram_if.vh"
`include "common_types.vh"
import common_types_pkg::*;

module axi_flash_test_fpga (
    input CLK12MHZ,
    input ck_rst,

    inout [3:0] qspi_dq,
    output qspi_cs,

    output UART_TXD,
    output LED [0:3],
    output led0_r,
    output led0_g,
    output led0_b
);
    logic clk, nrst;
    assign clk = CLK12MHZ;
    assign nrst = ck_rst;

    assign LED[0] = clk;
    assign LED[1] = nrst;
    assign LED[2] = ~UART_TXD;
    assign led0_r = (state == TEST_IDLE || state == TEST_DONE) ? 1'b1 : 1'b0; 
    assign led0_g = (state == TEST_READ || state == TEST_DONE) ? 1'b1 : 1'b0; 
    assign led0_b = (state == TEST_UART_0 || state == TEST_UART_1 || state == TEST_UART_2 || state == TEST_UART_3 || TEST_DONE) ? 1'b1 : 1'b0; 

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
    
    axi_flash_controller axi_flash_inst (
        .clk(clk),
        .nrst(nrst),
        .abif(abif_controller),
        .clk_div(4'd0),
        .flash_cs(qspi_cs),
        .flash_dq(qspi_dq)
    );

    localparam int BIT_PERIOD = 104; // Bit period in clk cycles

    logic uart_done;
    logic [7:0] uart_data;
    logic uart_start;
    uart_tx uart_inst (
        .clk(clk),
        .nrst(nrst),
        .bit_period(BIT_PERIOD),
        .data(uart_data),
        .start(uart_start),
        .serial_out(UART_TXD),
        .tx_done(uart_done)
    );

    typedef enum logic [3:0] {
        TEST_IDLE,
        TEST_READ,
        TEST_UART_0,
        TEST_UART_1,
        TEST_UART_2,
        TEST_UART_3,
        TEST_DONE
    } test_state_t;

    test_state_t state, next_state;

    always_ff @(posedge clk) begin
        if (~nrst) begin
            state <= TEST_IDLE;
        end else begin
            state <= next_state;
        end
    end

    always_comb begin
        next_state = state;
        case (state)
            TEST_IDLE: begin
                next_state = TEST_READ;
            end

            TEST_READ: begin
                if (amif.ready) begin
                    next_state = TEST_UART_0;
                end
            end

            TEST_UART_0: begin
                if (uart_done) begin
                    next_state = TEST_UART_1;
                end
            end

            TEST_UART_1: begin
                if (uart_done) begin
                    next_state = TEST_UART_2;
                end
            end

            TEST_UART_2: begin
                if (uart_done) begin
                    next_state = TEST_UART_3;
                end
            end

            TEST_UART_3: begin
                if (uart_done) begin
                    next_state = TEST_DONE;
                end
            end

            TEST_DONE: begin
            end

            default: begin
                next_state = TEST_IDLE;
            end
        endcase
    end

    always_comb begin
        amif.read = 1'b0;
        amif.write = 2'b00;
        amif.addr = 32'h00000000;
        amif.store = 32'h00000000;
        amif.done = 1'b0;

        uart_start = 1'b0;
        uart_data = 8'h00;

        case (state)
            TEST_IDLE: begin
            end

            TEST_READ: begin
                amif.read = 1'b1;
                amif.addr = 32'h0080_0000;

                if (amif.ready) begin
                    amif.done = 1'b1;
                end
            end

            TEST_UART_0: begin
                uart_start = 1'b1;
                uart_data = amif.load[7:0];

                if (uart_done) begin
                    uart_start = 1'b0;
                end
            end

            TEST_UART_1: begin
                uart_start = 1'b1;
                uart_data = amif.load[15:8];

                if (uart_done) begin
                    uart_start = 1'b0;
                end
            end

            TEST_UART_2: begin
                uart_start = 1'b1;
                uart_data = amif.load[23:16];

                if (uart_done) begin
                    uart_start = 1'b0;
                end
            end

            TEST_UART_3: begin
                uart_start = 1'b1;
                uart_data = amif.load[31:24];

                if (uart_done) begin
                    uart_start = 1'b0;
                end
            end

            TEST_DONE: begin
            end

            default: begin
            end
        endcase
    end

endmodule