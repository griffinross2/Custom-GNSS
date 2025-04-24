`timescale 1ns/1ns

module half_adder (
    input logic a, b,
    output logic sum, cout
);

assign sum = a ^ b;
assign cout = a & b;

endmodule