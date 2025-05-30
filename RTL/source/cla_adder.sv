/***************************/
/*  Carry Lookahead Adder  */
/***************************/
`timescale 1ns/1ns

`include "common_types.vh"
import common_types_pkg::*;

module cla_4_bit (
    input logic [3:0] a, b,
    input logic cin,
    output logic [3:0] sum,
    output logic cout, pg, gg
);

    logic [3:0] p, g;
    logic [3:0] c;

    // Generate propagate and generate signals
    assign p = a ^ b;  // Propagate
    assign g = a & b;  // Generate

    // Carry lookahead logic
    assign c[0] = cin;
    assign c[1] = g[0] | (p[0] & cin);
    assign c[2] = g[1] | (p[1] & g[0]) | (p[1] & p[0] & cin);
    assign c[3] = g[2] | (p[2] & g[1]) | (p[2] & p[1] & g[0]) | (p[2] & p[1] & p[0] & cin);
    assign cout = g[3] | (p[3] & g[2]) | (p[3] & p[2] & g[1]) | (p[3] & p[2] & p[1] & g[0]) | (p[3] & p[2] & p[1] & p[0] & cin);

    // Sum calculation
    assign sum = p ^ c;

    // Propagate and generate outputs
    assign pg = &p;
    assign gg = g[3] | (p[3] & g[2]) | (p[3] & p[2] & g[1]) | (p[3] & p[2] & p[1] & g[0]);

endmodule

module cla_16_bit (
    input logic [15:0] a, b,
    input logic cin,
    output logic [15:0] sum,
    output logic cout, pg, gg
);

    logic [3:0] sum_4bit[0:3];
    logic [0:3] pg_4bit;
    logic [0:3] gg_4bit;
    logic [3:0] c;

    // Instantiate four 4-bit CLA adders
    cla_4_bit cla0 (
        .a(a[3:0]),
        .b(b[3:0]),
        .cin(c[0]),
        .sum(sum_4bit[0]),
        .pg(pg_4bit[0]),
        .gg(gg_4bit[0])
    );

    cla_4_bit cla1 (
        .a(a[7:4]),
        .b(b[7:4]),
        .cin(c[1]),
        .sum(sum_4bit[1]),
        .pg(pg_4bit[1]),
        .gg(gg_4bit[1])
    );

    cla_4_bit cla2 (
        .a(a[11:8]),
        .b(b[11:8]),
        .cin(c[2]),
        .sum(sum_4bit[2]),
        .pg(pg_4bit[2]),
        .gg(gg_4bit[2])
    );

    cla_4_bit cla3 (
        .a(a[15:12]),
        .b(b[15:12]),
        .cin(c[3]),
        .sum(sum_4bit[3]),
        .pg(pg_4bit[3]),
        .gg(gg_4bit[3])
    );

    // Combine results
    assign sum = {sum_4bit[3], sum_4bit[2], sum_4bit[1], sum_4bit[0]};
    
    assign c[0] = cin;
    assign c[1] = gg_4bit[0] | (pg_4bit[0] & cin);
    assign c[2] = gg_4bit[1] | (pg_4bit[1] & gg_4bit[0]) | (pg_4bit[1] & pg_4bit[0] & cin);
    assign c[3] = gg_4bit[2] | (pg_4bit[2] & gg_4bit[1]) | (pg_4bit[2] & pg_4bit[1] & gg_4bit[0]) | (pg_4bit[2] & pg_4bit[1] & pg_4bit[0] & cin);
    assign cout = gg_4bit[3] | (pg_4bit[3] & gg_4bit[2]) | (pg_4bit[3] & pg_4bit[2] & gg_4bit[1]) | (pg_4bit[3] & pg_4bit[2] & pg_4bit[1] & gg_4bit[0]) | (pg_4bit[3] & pg_4bit[2] & pg_4bit[1] & pg_4bit[0] & cin);

    assign pg = &pg_4bit;
    assign gg = gg_4bit[3] | (pg_4bit[3] & gg_4bit[2]) | (pg_4bit[3] & pg_4bit[2] & gg_4bit[1]) | (pg_4bit[3] & pg_4bit[2] & pg_4bit[1] & gg_4bit[0]);

endmodule

module cla_32_bit (
    input word_t a, b,
    input logic cin,
    output word_t sum,
    output logic cout, pg, gg
);

    logic [15:0] sum_16bit[0:1];
    logic [0:1] pg_16bit;
    logic [0:1] gg_16bit;
    logic [1:0] c;

    // Instantiate two 16-bit CLA adders
    cla_16_bit cla0 (
        .a(a[15:0]),
        .b(b[15:0]),
        .cin(c[0]),
        .sum(sum_16bit[0]),
        .pg(pg_16bit[0]),
        .gg(gg_16bit[0])
    );

    cla_16_bit cla1 (
        .a(a[31:16]),
        .b(b[31:16]),
        .cin(c[1]),
        .sum(sum_16bit[1]),
        .pg(pg_16bit[1]),
        .gg(gg_16bit[1])
    );

    // Combine results
    assign sum = {sum_16bit[1], sum_16bit[0]};
    
    assign c[0] = cin;
    assign c[1] = gg_16bit[0] | (pg_16bit[0] & cin);
    assign cout = gg_16bit[1] | (pg_16bit[1] & gg_16bit[0]) | (pg_16bit[1] & pg_16bit[0] & cin);
    
    assign pg = &pg_16bit;
    assign gg = gg_16bit[1] | (pg_16bit[1] & gg_16bit[0]);

endmodule