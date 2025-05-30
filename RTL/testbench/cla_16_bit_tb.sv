module cla_16_bit_tb;

logic clk;
logic [15:0] a, b, sum;
logic cin, cout, pg, gg;

cla_16_bit cla_adder(
    .a(a),
    .b(b),
    .sum(sum),
    .cin(cin),
    .cout(cout),
    .pg(pg),
    .gg(gg)
);

task test_add;
    input logic [15:0] a_i, b_i;
    input logic cin_i;
    logic [16:0] sum_ext;
begin
    a = a_i;
    b = b_i;
    cin = cin_i;
    sum_ext = {1'b0,a} + {1'b0,b} + {16'b0,cin};

    @(posedge clk);

    if(sum_ext != {cout,sum}) begin
        $fatal("Test failed: %d + %d = {%d,%d}, expected {%d,%d}", a, b, cout, sum, sum_ext[16], sum_ext[15:0]);
    end else begin
        $display("Test passed: %d + %d = {%d,%d}", a, b, cout, sum);
    end
end
endtask

integer a_test, b_test;
integer i;

initial begin
    clk = 0;
    forever #5 clk = ~clk;
end

initial begin
    a = 0;
    b = 0;
    @(posedge clk);

    // Test many combinations without carry
    for (i = 0; i <= 100000; i+=1) begin
        a_test = $urandom;
        b_test = $urandom;
        test_add(a_test[15:0], b_test[15:0], 1'b0);
    end
    
    // Test many combinations with carry
    for (i = 0; i <= 100000; i+=1) begin
        a_test = $urandom;
        b_test = $urandom;
        test_add(a_test[15:0], b_test[15:0], 1'b1);
    end

    // Special tests
    test_add(0, 0, 1'b0);
    test_add('1, '1, 1'b0);
    test_add(0, 0, 1'b1);
    test_add('1, '1, 1'b1);

    $finish;
end

endmodule
