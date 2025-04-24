`timescale 1ns/1ns

`include "common_types.vh"
import common_types_pkg::*;
`include "divider_if.vh"

// Long Division Divider

module divider (
    input logic clk, nrst,
    divider_if.div divider_if
);

typedef enum logic [2:0] {
    IDLE = 3'b000,
    PRECHECK = 3'b001,
    DIVIDE = 3'b010,
    DONE = 3'b011,
    DONE_DIV_BY_ZERO = 3'b100,
    DONE_OVERFLOW = 3'b101
} divider_state_t;

divider_state_t state, next_state;

logic [31:0] n, d;
logic [31:0] next_n, next_d;

logic [31:0] q, r;
logic [31:0] next_q, next_r;

logic [4:0] div_counter;
logic [4:0] next_div_counter;

always_ff @(posedge clk) begin
    if (!nrst) begin
        n <= 32'd0;
        d <= 32'd0;
        q <= 32'd0;
        r <= 32'd0;
        div_counter <= '1;
        state <= IDLE;
    end else begin
        n <= next_n;
        d <= next_d;
        q <= next_q;
        r <= next_r;
        div_counter <= next_div_counter;
        state <= next_state;
    end
end

// Next state
always_comb begin
    next_state = state;

    case (state)
        IDLE: begin
            if (divider_if.en) begin
                next_state = PRECHECK;
            end
        end

        PRECHECK: begin
            if (d == 32'd0) begin
                next_state = DONE_DIV_BY_ZERO;  // Division by zero, go to done
            end else if (divider_if.is_signed && divider_if.a == 32'h80000000 && divider_if.b == 32'hFFFFFFFF) begin
                next_state = DONE_OVERFLOW;     // Overflow, go to done
            end else begin
                next_state = DIVIDE;
            end
        end

        DIVIDE: begin
            if (div_counter == 5'd0) begin
                next_state = DONE; // Finished division, go to DONE
            end else begin
                next_state = DIVIDE; // Continue dividing
            end
        end
            
        DONE: begin
            next_state = IDLE; // Division done, go back to IDLE
        end

        default: begin
        end
    endcase
end

// Logic
always_comb begin
    next_n = n;
    next_d = d;
    next_q = q;
    next_r = r;
    next_div_counter = div_counter;

    divider_if.div_by_zero = 1'b0;
    divider_if.overflow = 1'b0;
    divider_if.ready = 1'b0;

    divider_if.q = q;
    if (divider_if.is_signed) begin
        divider_if.q = (divider_if.a[31] ^ divider_if.b[31]) ? ~q + 32'd1 : q;  // Correct sign
    end

    divider_if.r = r;
    if (divider_if.is_signed) begin
        divider_if.r = divider_if.a[31] ? ~r + 32'd1 : r;   // Correct sign
    end

    case (state)
        IDLE: begin
            if (divider_if.en) begin
                next_n = divider_if.a;      // Load numerator
                next_d = divider_if.b;      // Load denominator
                next_q = 32'd0;             // Reset quotient
                next_r = 32'd0;             // Reset remainder
                next_div_counter = 5'd31;   // Set counter for 32 iterations

                // Prepare signed inputs
                if (divider_if.is_signed) begin
                    if (divider_if.a[31] == 1'b1) begin
                        next_n = ~divider_if.a + 32'd1; // Negate numerator
                    end
                    if (divider_if.b[31] == 1'b1) begin
                        next_d = ~divider_if.b + 32'd1; // Negate denominator
                    end
                end else begin
                    next_n = divider_if.a;
                    next_d = divider_if.b;
                end
            end else begin
                divider_if.ready = 1'b1;
            end
        end

        PRECHECK: begin
        end

        DIVIDE: begin
            next_r = {r[30:0], n[31]};          // Shift in the next bit of the numerator
            next_n = {n[30:0], 1'b0};           // Shift the numerator left

            if (next_r >= d) begin
                next_r = next_r - d;            // Subtract the denominator from the remainder
                next_q = {q[30:0], 1'b1};       // Shift in 1
            end else begin
                next_q = {q[30:0], 1'b0};       // Shift in 0
            end

            next_div_counter = div_counter - 5'd1;  // Decrement counter
        end

        DONE: begin
            divider_if.ready = 1'b1;    // Division done, set ready flag
        end

        DONE_DIV_BY_ZERO: begin
            divider_if.div_by_zero = 1'b1;  // Division by zero
            divider_if.ready = 1'b1;        // We are done, so set ready

            // Set the quotient to all ones and remainder to dividend
            divider_if.q = 32'hFFFFFFFF;
            divider_if.r = divider_if.a;
        end

        DONE_OVERFLOW: begin
            divider_if.overflow = 1'b1;     // Overflow condition
            divider_if.ready = 1'b1;        // We are done, so set ready

            // Set the quotient to dividend and remainder to 0
            divider_if.q = 32'h80000000;
            divider_if.r = 32'd0;
        end

        default: begin
        end
    endcase
end

endmodule