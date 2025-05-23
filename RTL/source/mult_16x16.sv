/*******************************/
/* Sequential 16x16 Multiplier */
/*******************************/

`timescale 1ns/1ns

`include "common_types.vh"
import common_types_pkg::*;

module mult_16x16 (
    input logic clk, nrst,
    input logic start,
    input logic [15:0] a, b,
    output logic [31:0] p,
    output logic busy
);

typedef enum logic {
    IDLE = 1'b0,
    BUSY = 1'b1
} state_t;

state_t state, next_state;
logic [15:0] a_reg, next_a_reg;
logic [15:0] a_inv_reg, next_a_inv_reg;
logic [32:0] p_reg, next_p_reg;
logic [3:0] op_counter, next_op_counter;

always_ff @(posedge clk) begin
    if (!nrst) begin
        state <= IDLE;
        a_reg <= 16'd0;
        a_inv_reg <= 16'd0;
        p_reg <= 33'd0;
        op_counter <= 4'd0;
    end else begin
        state <= next_state;
        a_reg <= next_a_reg;
        a_inv_reg <= next_a_inv_reg;
        p_reg <= next_p_reg;
        op_counter <= next_op_counter;
    end
end

// Next state logic
always_comb begin
    case (state)
        IDLE: begin
            if (start) begin
                next_state = BUSY;
            end else begin
                next_state = IDLE;
            end
        end
        BUSY: begin
            if (op_counter == 4'd15) begin
                next_state = IDLE;
            end else begin
                next_state = BUSY;
            end
        end
        default: next_state = IDLE;
    endcase
end

// Output logic
always_comb begin
    busy = 1'b0;
    p = p_reg[32:1];
    next_a_reg = a_reg;
    next_a_inv_reg = a_inv_reg;
    next_p_reg = p_reg; 
    next_op_counter = op_counter;

    case (state)
        IDLE: begin
            if (start) begin
                next_a_reg = a;
                next_a_inv_reg = ~a + 1'b1; // Two's complement
                next_p_reg = {16'd0, b, 1'b0};
                next_op_counter = '0;
            end
        end
        BUSY: begin
            busy = 1'b1;

            case (p_reg[1:0])
                2'b00: begin
                    // Nothing
                end
                2'b01: begin
                    next_p_reg = p_reg + {a_reg, 17'b0}; // Add a to p
                end
                2'b10: begin
                    next_p_reg = p_reg + {a_inv_reg, 17'b0}; // Subtract a from p
                end
                2'b11: begin
                    // Nothing
                end
            endcase

            // Shift the product
            next_p_reg = {next_p_reg[32], next_p_reg[32:1]};

            // Increment the operation counter
            next_op_counter = op_counter + 4'd1;
        end
        default: begin
            next_a_reg = 16'd0;
            next_a_inv_reg = 16'd0;
            next_p_reg = 33'd0;
        end
    endcase
end

endmodule