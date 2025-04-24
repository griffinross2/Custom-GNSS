`timescale 1ns/1ns

module full_adder (
    input logic a, b, cin,
    output logic sum, cout
);

logic s1, c1, c2;
half_adder ha1(.a(a), .b(b), .sum(s1), .cout(c1));
half_adder ha2(.a(s1), .b(cin), .sum(sum), .cout(c2));

assign cout = c1 | c2;

endmodule