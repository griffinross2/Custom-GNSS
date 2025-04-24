`include "multiplier_if.vh"

module multiplier_tb;

logic clk, nrst;

multiplier_if multiplier_if();

multiplier multiplier(
    .clk(clk),
    .nrst(nrst),
    .multiplier_if(multiplier_if)
);

task test_multiply;
    input logic [31:0] a, b;
begin
    multiplier_if.a = a;
    multiplier_if.b = b;
    multiplier_if.is_signed_a = 0;
    multiplier_if.is_signed_b = 0;

    @(negedge clk);
    multiplier_if.en = 1;
    @(negedge clk);
    multiplier_if.en = 0;

    wait(multiplier_if.ready);

    if(multiplier_if.out != a*b) begin
        $fatal("Test failed: %d * %d = %d, expected %d", a, b, multiplier_if.out, a*b);
    end else begin
        $display("Test passed: %d * %d = %d", a, b, multiplier_if.out);
    end
end
endtask

task test_multiply_signed;
    input logic [31:0] a, b;
begin
    multiplier_if.a = a;
    multiplier_if.b = b;
    multiplier_if.is_signed_a = 1;
    multiplier_if.is_signed_b = 1;
    
    @(negedge clk);
    multiplier_if.en = 1;
    @(negedge clk);
    multiplier_if.en = 0;

    wait(multiplier_if.ready);

    if($signed(multiplier_if.out) != $signed(a)*$signed(b)) begin
        $fatal("Test failed: %d * %d = %d, expected %d", $signed(a), $signed(b), $signed(multiplier_if.out), $signed(a)*$signed(b));
    end else begin
        $display("Test passed: %d * %d = %d", $signed(a), $signed(b), $signed(multiplier_if.out));
    end
end
endtask

task test_multiply_a_signed;
    input logic [31:0] a, b;
    logic [32:0] a_signed;
    logic [32:0] b_fake_signed;
begin
    multiplier_if.a = a;
    multiplier_if.b = b;
    multiplier_if.is_signed_a = 1;
    multiplier_if.is_signed_b = 0;
    
    @(negedge clk);
    multiplier_if.en = 1;
    @(negedge clk);
    multiplier_if.en = 0;

    wait(multiplier_if.ready);

    a_signed = {a[31], a};
    b_fake_signed = {1'b0, b};

    if($signed(multiplier_if.out) != $signed(a_signed)*$signed(b_fake_signed)) begin
        $fatal("Test failed: %d * %d = %d, expected %d", $signed(a_signed), $signed(b_fake_signed), $signed(multiplier_if.out), $signed(a_signed)*$signed(b_fake_signed));
    end else begin
        $display("Test passed: %d * %d = %d", $signed(a_signed), $signed(b_fake_signed), $signed(multiplier_if.out));
    end
end
endtask

task test_multiply_b_signed;
    input logic [31:0] a, b;
    logic [32:0] a_fake_signed;
    logic [32:0] b_signed;
begin
    multiplier_if.a = a;
    multiplier_if.b = b;
    multiplier_if.is_signed_a = 0;
    multiplier_if.is_signed_b = 1;
    
    @(negedge clk);
    multiplier_if.en = 1;
    @(negedge clk);
    multiplier_if.en = 0;

    wait(multiplier_if.ready);

    a_fake_signed = {1'b0, a};
    b_signed = {b[31], b};

    if($signed(multiplier_if.out) != $signed(a_fake_signed)*$signed(b_signed)) begin
        $fatal("Test failed: %d * %d = %d, expected %d", $signed(a_fake_signed), $signed(b_signed), $signed(multiplier_if.out), $signed(a_fake_signed)*$signed(b_signed));
    end else begin
        $display("Test passed: %d * %d = %d", $signed(a_fake_signed), $signed(b_signed), $signed(multiplier_if.out));
    end
end
endtask

logic [31:0] a, b;
integer i;

initial begin
    clk = 0;
    forever #5 clk = ~clk;
end

initial begin
    multiplier_if.en = 0;
    multiplier_if.is_signed_a = 0;
    multiplier_if.is_signed_b = 0;
    nrst = 0;
    #30;
    nrst = 1;

    a = 0;
    b = 0;

    for (i = 0; i <= 100000; i+=1) begin
        a = $urandom();
        b = $urandom();
        test_multiply(a, b);
    end

    for (i = 0; i <= 100000; i+=1) begin
        a = $random();
        b = $random();
        test_multiply_signed(a, b);
    end

    for (i = 0; i <= 100000; i+=1) begin
        a = $random();
        b = $urandom();
        test_multiply_a_signed(a, b);
    end

    for (i = 0; i <= 100000; i+=1) begin
        a = $urandom();
        b = $random();
        test_multiply_b_signed(a, b);
    end

    $finish;
end

endmodule
