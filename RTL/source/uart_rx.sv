`timescale 1ns/1ns

module uart_rx (
    input logic clk, nrst,
    input logic [15:0] bit_period,
    input logic serial_in,
    output logic [7:0] data,
    output logic rx_busy,
    output logic rx_done
);

typedef enum logic [1:0] {
    UART_RX_IDLE,
    UART_RX_SYNC,
    UART_RX_DATA,
    UART_RX_STOP
} uart_rx_state_t;

uart_rx_state_t state, state_n;
logic [2:0] bit_count, bit_count_n;

logic data_strobe;

always_ff @(posedge clk) begin
    if (!nrst) begin
        state <= UART_RX_IDLE;
        bit_count <= 0;
    end else begin
        state <= state_n;
        bit_count <= bit_count_n;
    end
end

// Serial in synchronizer
logic serial_reg [1:0];
always_ff @(posedge clk) begin
    if (!nrst) begin
        serial_reg[0] <= 1'b1;
        serial_reg[1] <= 1'b1;
    end else begin
        serial_reg[0] <= serial_in;
        serial_reg[1] <= serial_reg[0];
    end
end
logic serial_in_sync;
assign serial_in_sync = serial_reg[1];

// Sync/bit period counter
logic [15:0] sync_bit_counter;
logic [15:0] sync_bit_counter_n;
always_ff @(posedge clk) begin
    if (!nrst) begin
        sync_bit_counter <= 16'd0;
    end else begin
        sync_bit_counter <= sync_bit_counter_n;
    end
end

// Serial shift register
always_ff @(posedge clk) begin
    if (!nrst) begin
        data <= 8'd0;
    end else if (data_strobe) begin
        data <= {serial_in_sync, data[7:1]};
    end
end

// Next state logic
always_comb begin
    state_n = state;

    casez (state)
        UART_RX_IDLE: begin
            if (~serial_in_sync) begin
                // Start bit detected
                state_n = UART_RX_SYNC;
            end
        end
        UART_RX_SYNC: begin
            // Sync to center of sample point
            if (sync_bit_counter == {1'b0, bit_period[15:1]} - 16'd1) begin
                state_n = UART_RX_DATA;
            end
        end
        UART_RX_DATA: begin
            // End of last bit period
            if (bit_count == 3'd7 && data_strobe) begin
                state_n = UART_RX_STOP;
            end
        end
        UART_RX_STOP: begin
            // Stop bit lasts one bit period
            if (bit_period == 16'd0 || sync_bit_counter == bit_period - 16'd1) begin
                state_n = UART_RX_IDLE;
            end
        end
        default: begin
            state_n = UART_RX_IDLE; // Default to IDLE state
        end
    endcase
end

// State output logic
always_comb begin
    bit_count_n = bit_count;
    sync_bit_counter_n = sync_bit_counter;
    data_strobe = 1'b0;
    rx_done = 1'b0;
    rx_busy = 1'b1;

    casez (state)
        UART_RX_IDLE: begin
            // Idle
            rx_busy = 1'b0;                             // Reset rx_busy in IDLE state
        end
        UART_RX_SYNC: begin
            sync_bit_counter_n = sync_bit_counter + 1;  // Increment sync counter
            if ({1'b0, bit_period[15:1]} == 16'd0 || sync_bit_counter == {1'b0, bit_period[15:1]} - 16'd1) begin
                sync_bit_counter_n = 16'd0;             // Reset sync counter when entering DATA state
                bit_count_n = 3'd0;                     // Reset bit count
            end
        end
        UART_RX_DATA: begin
            sync_bit_counter_n = sync_bit_counter + 1;  // Increment sync counter
            if (bit_period == 16'd0 || sync_bit_counter == bit_period - 16'd1) begin
                sync_bit_counter_n = 16'd0;             // Reset sync counter when finishing a bit
                bit_count_n = bit_count + 3'd1;         // Increment bit count
                data_strobe = 1'b1;                     // Set data strobe to indicate new data
            end
        end
        UART_RX_STOP: begin
            sync_bit_counter_n = sync_bit_counter + 1;  // Increment sync counter
            if (bit_period == 16'd0 || sync_bit_counter == bit_period - 16'd1) begin
                sync_bit_counter_n = 16'd0;             // Reset sync counter when finishing stop bit
                rx_done = 1'b1;                         // Set rx_done to indicate data received
            end
        end
        default: begin
            sync_bit_counter_n = 16'd0;         // Default to reset sync counter
            bit_count_n = 3'd0;                 // Default to reset bit count
            data_strobe = 1'b0;                 // Default to no data strobe
        end
    endcase
end

endmodule