`timescale 1ns/1ns

module uart_tx (
    input logic clk, nrst,
    input logic [15:0] bit_period,
    input logic [7:0] data,
    input logic start,
    output logic serial_out,
    output logic tx_busy,
    output logic tx_done
);

typedef enum logic [1:0] {
    UART_TX_IDLE,
    UART_TX_START,
    UART_TX_DATA,
    UART_TX_STOP
} uart_tx_state_t;

uart_tx_state_t state, state_n;
logic [2:0] bit_count, bit_count_n;

logic serial_out_n;

// Bit strobe generation
logic bit_strobe;
logic [15:0] bit_strobe_count;
always_ff @(posedge clk) begin
    if (!nrst) begin
        bit_strobe_count = 0;
    end else if (bit_period == 0) begin
        bit_strobe = 1;
        bit_strobe_count = 0;
    end else if (bit_strobe_count == bit_period - 16'd1) begin
        bit_strobe = 1;
        bit_strobe_count = 0;
    end else begin
        bit_strobe = 0;
        bit_strobe_count = bit_strobe_count + 16'd1;
    end
end

always_ff @(posedge clk) begin
    if (!nrst) begin
        state <= UART_TX_IDLE;
        bit_count <= 0;
        serial_out <= 1'b1;
    end else begin
        state <= state_n;
        bit_count <= bit_count_n;
        serial_out <= serial_out_n;
    end
end

// Next state logic
always_comb begin
    state_n = state;

    casez (state)
        UART_TX_IDLE: begin
            if (start && bit_strobe) begin
                state_n = UART_TX_START;
            end
        end
        UART_TX_START: begin
            // Start bit lasts one bit period
            if (bit_strobe) begin
                state_n = UART_TX_DATA;
            end
        end
        UART_TX_DATA: begin
            // End of last bit period
            if (bit_count == 3'd7 && bit_strobe) begin
                state_n = UART_TX_STOP;
            end
        end
        UART_TX_STOP: begin
            // Stop bit lasts one bit period
            if (bit_strobe) begin
                state_n = UART_TX_IDLE;
            end
        end
        default: begin
            state_n = UART_TX_IDLE; // Default to IDLE state
        end
    endcase
end

// State output logic
always_comb begin
    serial_out_n = serial_out;
    bit_count_n = 3'd0;
    tx_busy = 1'b1;
    tx_done = 1'b0;

    casez (state)
        UART_TX_IDLE: begin
            // Idle
            tx_busy = 1'b0;

            // Idle line high
            serial_out_n = 1'b1;
        end
        UART_TX_START: begin
            serial_out_n = 1'b0; // Start bit is low
        end
        UART_TX_DATA: begin
            serial_out_n = data[bit_count]; // Send data bits
            bit_count_n = bit_count; // Keep the same bit count until a bit transition
            if (bit_strobe) begin
                bit_count_n = bit_count + 3'd1;
            end
        end
        UART_TX_STOP: begin
            serial_out_n = 1'b1;    // Stop bit is high
            if (state_n == UART_TX_IDLE) begin
                tx_done = 1'b1;     // Transmission done
            end
        end
        default: begin
            serial_out_n = 1'b1;    // Default to idle state
            bit_count_n = 3'd0;     // Reset bit count
        end
    endcase
end

endmodule