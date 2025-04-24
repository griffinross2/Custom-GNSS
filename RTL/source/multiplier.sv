`timescale 1ns/1ns

`include "common_types.vh"
import common_types_pkg::*;
`include "multiplier_if.vh"

// Booth Multiplier

module multiplier (
    input logic clk, nrst,
    multiplier_if.mult multiplier_if
);

logic [64:0] m, m_load, m_inv;
logic [33:0] r, r_load;
logic [64:0] p, p_load;

// Goes high once the multiplier starts
logic busy;

always_ff @(posedge clk) begin
    if (!nrst) begin
        busy <= 1'b0;
        m <= 65'd0;
        m_inv <= 65'd0;
        r <= 34'd0;
        p <= 65'd0;
    end else if (busy) begin
        // Shift the multiplier and multiplicand
        m <= {m[62:0], 2'b0};           // Shift the multiplicand left 2
        m_inv <= {m_inv[62:0], 2'b0};   // Shift the multiplicand and its inverse left 2
        r <= {{2{r[33]}}, r[33:2]};     // Arithmetic shift right the multiplier 2

        // Update the product
        p <= p_load;

        // End condition
        busy <= (r == '0 || r == '1) ? 1'b0 : 1'b1;
    end else if (multiplier_if.en) begin
        busy <= 1'b1;

        // Load the multiplier and multiplicand when starting
        m <= busy ? m : m_load;
        m_inv <= busy ? m_inv : (~m_load + 65'd1);
        r <= busy ? r : r_load;

        // Reset the product
        p <= 65'd0;
    end
end

always_comb begin
    m_load = 65'd0;
    r_load = 34'd0;
    m_load[31:0] = multiplier_if.a;     // Multiplicand
    r_load[32:1] = multiplier_if.b;     // Multiplier (leave lower 0 for recoding)

    // Extend the sign bit of the multiplier, if signed
    if (multiplier_if.is_signed_a) begin
        m_load[64:32] = {33{m_load[31]}};
    end

    if (multiplier_if.is_signed_b) begin
        r_load[33] = r_load[32];
    end
end

logic [64:0] pp;
always_comb begin
    // Generate the partial product according to the recoding
    case(r[2:0])
        3'b000: pp = 65'd0;                 // 0
        3'b001: pp = m;                     // +1
        3'b010: pp = m;                     // +1
        3'b011: pp = {m[63:0], 1'b0};       // +2
        3'b100: pp = {m_inv[63:0], 1'b0};   // -2
        3'b101: pp = m_inv;                 // -1
        3'b110: pp = m_inv;                 // -1
        3'b111: pp = 65'd0;                 // 0
    endcase

    // Update the product
    p_load = p + pp;
end

assign multiplier_if.out = p[63:0];  // Output the lower 64 bits of the product
assign multiplier_if.ready = ~busy; // Indicate that the multiplier is ready

endmodule