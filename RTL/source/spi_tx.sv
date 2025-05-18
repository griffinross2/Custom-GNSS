`timescale 1ns / 1ps

module spi_tx (
    input  logic                  clk,          // Clock signal
    input  logic                  nrst,         // Active low reset
    input  logic [15:0]           bit_period,   // SPI clock divider
    input  logic [4:0]            data_width,   // Data width (0-31)->(1 to 32 bits)
    input  logic                  start,        // Start transmission signal
    input  logic [31:0]           data,         // Data to transmit (up to 32 bits)
    output logic                  sck,          // SPI clock
    output logic                  sdo,          // Serial data out
    output logic                  cs,           // Chip select (active low)
    output logic                  busy,         // Busy signal
    output logic                  done          // Transmission done signal
);

    // States
    typedef enum logic [1:0] {
        IDLE,   // Idle
        START,  // Sets busy and latch data
        DATA,   // Send data
        STOP    // Keep CS raised for half a period
    } state_t;

    state_t state, next_state;

    logic [4:0] bit_counter, next_bit_counter;          // Bit counter for data transmission
    logic [31:0] shift_reg, next_shift_reg;             // Shift register for data transmission
    logic [15:0] clk_div_counter, next_clk_div_counter; // Clock divider counter
    logic next_sck;
    logic next_cs;
    logic next_sdo;
    logic next_done;

    always_ff @(posedge clk) begin
        if (!nrst) begin
            state <= IDLE;
            bit_counter <= 5'h0;
            shift_reg <= 32'h0;
            clk_div_counter <= 16'h0;
            sck <= 0;
            cs <= 1;
            sdo <= 0;
            done <= 0;
        end else begin
            state <= next_state;
            bit_counter <= next_bit_counter;
            shift_reg <= next_shift_reg;
            clk_div_counter <= next_clk_div_counter;
            sck <= next_sck;
            cs <= next_cs;
            sdo <= next_sdo;
            done <= next_done;
        end
    end

    // State transition
    always_comb begin
        next_state = state;

        case (state)
            IDLE: begin
                if (start) begin
                    // If start requested
                    next_state = START;
                end
            end
            START: begin
                // After half a bit period, go to DATA state
                if (clk_div_counter == bit_period) begin
                    next_state = DATA;
                end
            end
            DATA: begin
                // After all data sent, go to STOP state
                if (bit_counter == data_width && clk_div_counter == bit_period && sck) begin
                    next_state = STOP;
                end
            end
            STOP: begin
                // After half a bit period, go back to IDLE
                if (clk_div_counter == bit_period) begin
                    next_state = IDLE;
                end
            end
        endcase
    end

    // State output
    always_comb begin
        next_bit_counter = bit_counter;
        next_shift_reg = shift_reg;
        next_clk_div_counter = clk_div_counter;
        next_sck = sck;
        next_sdo = sdo;
        next_cs = cs;

        next_done = 1'b0;
        busy = 1'b1;

        case (state)
            IDLE: begin
                // Load shift reg when leaving state
                if (start) begin
                    next_shift_reg = data;
                end

                // Lower busy
                busy = 1'b0;
            end
            START: begin
                // Generate internal clock signal
                if (clk_div_counter < bit_period) begin
                    next_clk_div_counter = clk_div_counter + 16'd1;
                end else begin
                    // Set first SDO (MSB) when going to DATA and lower CS
                    next_sdo = shift_reg[data_width];
                    next_cs = 0;
                    next_clk_div_counter = 16'd0;
                end
            end
            DATA: begin
                // Generate clock signal
                if (clk_div_counter < bit_period) begin
                    next_clk_div_counter = clk_div_counter + 16'd1;
                end else begin
                    next_sck = ~sck; // Toggle clock signal
                    next_clk_div_counter = 16'd0;

                    // If this is the end of a bit period, shift the data
                    if (sck) begin
                        next_bit_counter = bit_counter + 5'd1;
                        next_shift_reg = {shift_reg[30:0], 1'b0};   // Shift left
                        next_sdo = next_shift_reg[data_width];      // Send data MSB first
                        
                        if (bit_counter == data_width) begin
                            next_bit_counter = 5'd0;
                            // Raise CS when exiting data
                            next_cs = 1;
                        end
                    end
                end
            end
            STOP: begin
                // Generate clock signal
                if (clk_div_counter < bit_period) begin
                    next_clk_div_counter = clk_div_counter + 16'd1;
                end else begin
                    // Raise done when going to IDLE
                    next_done = 1'b1;
                    next_clk_div_counter = 16'd0;
                end
            end
        endcase
    end

endmodule