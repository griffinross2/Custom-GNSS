`timescale 1ns/1ns

`include "common_types.vh"
import common_types_pkg::*;

module fem_record_fpga (
    input logic CLK,
    input logic ck_rst,
    input logic jb_0, jb_1, jb_2, jb_3, jb_6,
    output logic jb_4, jb_5, jb_7,
    output logic LED [0:3],
    output logic UART_TXD,
    output logic led0_r
);

    logic [1:0] inp_i;
    logic [1:0] inp_q;

    logic sck, sda, cs;

    // assign {jb_5, jb_4, jb_7} = {sck, sda, cs};
    assign {jb_5, jb_4, jb_7} = 3'b100;

    assign inp_i = {jb_3, jb_2};
    assign inp_q = {jb_1, jb_0};
    assign LED[0:3] = {inp_i[0], inp_i[1], inp_q[0], inp_q[1]};
    
    integer div;
    logic clk_1hz;
    
    always_ff @(posedge jb_6) begin
        if (~ck_rst) begin
            div = 0;
            clk_1hz = 0;
        end else begin
            if (div < 9600000) begin
                div = div + 1;
            end else begin
                div = 0;
                clk_1hz = ~clk_1hz;
            end
        end
    end

    // SPI configuration

    logic [31:0] delay_send;
    logic send_done;
    logic start;
    logic busy;
    logic [2:0] spi_word_num;
    logic [27:0] conf1;
    logic [27:0] conf2;
    logic [27:0] conf3;
    logic [27:0] pllconf;
    logic [27:0] pllint;
    // assign conf1 = {1'b1, 1'b0, 4'b1111, 2'b11, 2'b11, 2'b11, 1'b0, 2'b00, 1'b1, 1'b1, 6'b001011, 2'b10, 1'b0, 1'b1, 1'b1};
    assign conf1 = {1'b1, 1'b0, 4'b1111, 2'b11, 2'b11, 2'b11, 1'b0, 2'b00, 1'b1, 1'b1, 6'b001101, 2'b00, 1'b0, 1'b1, 1'b1};
    assign conf2 = {1'b0, 12'b000010101010, 2'b00, 2'b00, 2'b01, 3'b010, 2'b00, 1'b1, 1'b0, 2'b00};
    assign conf3 = {6'b111010, 1'b1, 1'b0, 1'b1, 1'b1, 1'b1, 1'b1, 1'b0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0, 3'b111, 2'b11, 1'b0, 1'b0, 1'b0, 1'b0};
    assign pllconf = {1'b1, 1'b0, 1'b0, 1'b1, 1'b1, 2'b11, 2'b01, 5'b10000, 4'b0000, 1'b1, 1'b0, 1'b0, 3'b000, 1'b1, 1'b0, 1'b0, 1'b0};
    assign pllint = {15'd7857, 10'd96, 3'b000};

    logic [31:0] serial_word;

    always_ff @(posedge jb_6) begin
        if (~ck_rst) begin
            delay_send <= 32'h0;
            spi_word_num <= 3'd0;
            start <= 1'b1;
        end else begin
            if (delay_send < 32'd1920000) begin
                delay_send <= delay_send + 32'd1;
            end
            if(send_done) begin
                spi_word_num <= spi_word_num + 3'd1;
                start <= 1'b1;
            end else if (busy) begin
                start <= 1'b0;
            end
        end
    end

    spi_tx spi_tx_inst (
        .clk(jb_6),
        .nrst(ck_rst),
        .bit_period(16'd1),
        .data_width(5'd31),
        .data(serial_word),
        .start(start && delay_send == 32'd1920000 && spi_word_num < 3'd5),
        .sdo(sda),
        .sck(sck),
        .cs(cs),
        .done(send_done),
        .busy(busy)
    );

    always_comb begin
        serial_word = 32'h0;
        case (spi_word_num)
            3'd0: serial_word = {conf1, 4'b0000};
            3'd1: serial_word = {conf2, 4'b0001};
            3'd2: serial_word = {conf3, 4'b0010};
            3'd3: serial_word = {pllconf, 4'b0011};
            3'd4: serial_word = {pllint, 4'b0100};
        endcase
    end
    
    assign led0_r = clk_1hz;

    localparam RAM_SIZE = 24000; // Number of words

    // 32-bit shift reg
    logic [4:0] bit_counter;
    logic [29:0] word_counter;
    logic [31:0] shift_reg;
    logic sample_done;
    always_ff @(negedge jb_6) begin
        if (~ck_rst) begin
            bit_counter <= 5'h0;
            word_counter <= 30'h0;
            shift_reg <= 32'h0;
            sample_done <= 1'b0;
        end else if (spi_word_num == 3'd5 && word_counter != RAM_SIZE) begin
            // Shift in sign of signal
            shift_reg <= {inp_i[1], shift_reg[31:1]};
            bit_counter <= bit_counter + 5'd1;
            if (bit_counter == 5'h1F) begin
                word_counter <= word_counter + 30'd1;
            end
        end else if (spi_word_num == 3'd5) begin
            // Done
            sample_done <= 1'b1;
        end
    end

    typedef enum logic [3:0] {
        UART_READ,
        UART_READ_WAIT_0,
        UART_READ_WAIT_1,
        UART_SEND_0,
        UART_WAIT_0,
        UART_SEND_1,
        UART_WAIT_1,
        UART_SEND_2,
        UART_WAIT_2,
        UART_SEND_3,
        UART_WAIT_3,
        UART_DONE
    } uart_state_t;

    // RAM output
    logic [31:0] ram_out;
    logic [31:0] ram_word, ram_word_n;
    logic [29:0] ram_addr, ram_addr_n;
    uart_state_t uart_state, uart_state_n;
    logic uart_start;
    logic [7:0] uart_byte, uart_byte_n;
    logic uart_busy;

    always_ff @(posedge jb_6) begin
        if (~ck_rst || ~sample_done) begin
            uart_state <= UART_READ;
            ram_addr <= 30'h0;
            ram_word <= 32'h0;
            uart_byte <= 8'h0;
        end else begin
            uart_state <= uart_state_n;
            ram_addr <= ram_addr_n;
            ram_word <= ram_word_n;
            uart_byte <= uart_byte_n;
        end
    end

    xpm_memory_sdpram #(
    .ADDR_WIDTH_A(30),                                          // 32 bit but aligned
    .ADDR_WIDTH_B(30),                                          // 32 bit but aligned
    .MEMORY_SIZE(RAM_SIZE * 32),                                // RAM_SIZE words of 32 bits
    .WRITE_DATA_WIDTH_A(32),                                    // 32 bits data width
    .BYTE_WRITE_WIDTH_A(32),                                    // Word-wide width
    .READ_DATA_WIDTH_B(32),                                     // 32 bits data width
    .READ_LATENCY_B(1),                                         // Latency
    .RST_MODE_A("ASYNC"),                                       // Asynchronous reset
    .RST_MODE_B("ASYNC"),                                       // Asynchronous reset
    .MEMORY_PRIMITIVE("block")                                  // Block RAM
    ) ram_inst (
    .clka(jb_6),
    .clkb(jb_6),
    .rstb(~ck_rst),
    .ena(1'b1),
    .enb(1'b1),
    .wea(bit_counter == '0 && !sample_done),
    .addra(word_counter),
    .addrb(ram_addr),
    .dina(shift_reg),
    .doutb(ram_out),
    .injectdbiterra(1'b0),
    .injectsbiterra(1'b0),
    .regceb(1'b1),
    .sleep(1'b0)
    );

    uart_tx uart_inst (
        .clk(jb_6),
        .bit_period(16'd167),
        .nrst(ck_rst),
        .serial_out(UART_TXD),
        .start(uart_start),
        .data(uart_byte),
        .tx_busy(uart_busy)
    );

    always_comb begin
        ram_addr_n = ram_addr;
        ram_word_n = ram_word;
        uart_state_n = uart_state;
        uart_start = 1'b0;
        uart_byte_n = uart_byte;

        case (uart_state)
            UART_READ: begin
                uart_state_n = UART_READ_WAIT_0;
            end
            UART_READ_WAIT_0: begin
                uart_state_n = UART_READ_WAIT_1;
            end
            UART_READ_WAIT_1: begin
                uart_state_n = UART_SEND_0;
                ram_addr_n = ram_addr + 30'd1;
                ram_word_n = ram_out;
                uart_byte_n = ram_out[7:0];
            end
            UART_SEND_0: begin
                uart_start = 1'b1;
                if (uart_busy) begin
                    uart_state_n = UART_WAIT_0;
                end
            end
            UART_WAIT_0: begin
                if (!uart_busy) begin
                    uart_byte_n = ram_word[15:8];
                    uart_state_n = UART_SEND_1;
                end
            end
            UART_SEND_1: begin
                uart_start = 1'b1;
                if (uart_busy) begin
                    uart_state_n = UART_WAIT_1;
                end
            end
            UART_WAIT_1: begin
                if (!uart_busy) begin
                    uart_byte_n = ram_word[23:16];
                    uart_state_n = UART_SEND_2;
                end
            end
            UART_SEND_2: begin
                uart_start = 1'b1;
                if (uart_busy) begin
                    uart_state_n = UART_WAIT_2;
                end
            end
            UART_WAIT_2: begin
                if (!uart_busy) begin
                    uart_byte_n = ram_word[31:24];
                    uart_state_n = UART_SEND_3;
                end
            end
            UART_SEND_3: begin
                uart_start = 1'b1;
                if (uart_busy) begin
                    uart_state_n = UART_WAIT_3;
                end
            end
            UART_WAIT_3: begin
                if (!uart_busy) begin
                    if (ram_addr == RAM_SIZE) begin
                        uart_state_n = UART_DONE;
                    end else begin
                        uart_state_n = UART_READ;
                    end
                end
            end
        endcase
    end

endmodule