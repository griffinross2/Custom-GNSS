module cla_4_bit_tb;

logic clk;
logic [3:0] a, b, sum;
logic cin, cout, pg, gg;

cla_4_bit cla_adder(
    .a(a),
    .b(b),
    .sum(sum),
    .cin(cin),
    .cout(cout),
    .pg(pg),
    .gg(gg)
);

task test_add;
    input logic [3:0] a_i, b_i;
    input logic cin_i;
    logic [4:0] sum_ext;
begin
    a = a_i;
    b = b_i;
    cin = cin_i;
    sum_ext = {1'b0,a} + {1'b0,b} + {4'b0,cin};

    @(posedge clk);

    if(sum_ext != {cout,sum}) begin
        $fatal("Test failed: %d + %d = {%d,%d}, expected {%d,%d}", a, b, cout, sum, sum_ext[4], sum_ext[3:0]);
    end else begin
        $display("Test passed: %d + %d = {%d,%d}", a, b, cout, sum);
    end
end
endtask

integer a_test, b_test;

initial begin
    clk = 0;
    forever #5 clk = ~clk;
end

initial begin
    a = 0;
    b = 0;
    @(posedge clk);

    // Test all combinations without carry
    for (a_test = 4'h0; a_test <= 4'hF; a_test+=4'd1) begin
        for (b_test = 4'h0; b_test <= 4'hF; b_test+=4'd1) begin
            test_add(a_test[3:0], b_test[3:0], 1'b0);
        end
    end
    
    // Test all combinations with carry
    for (a_test = 4'h0; a_test <= 4'hF; a_test+=4'd1) begin
        for (b_test = 4'h0; b_test <= 4'hF; b_test+=4'd1) begin
            test_add(a_test[3:0], b_test[3:0], 1'b1);
        end
    end

    $finish;
end

endmodule
