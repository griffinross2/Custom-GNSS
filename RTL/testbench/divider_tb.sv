`include "divider_if.vh"

module divider_tb;

logic clk, nrst;

divider_if divider_if();

divider divider(
    .clk(clk),
    .nrst(nrst),
    .divider_if(divider_if)
);

task test_divide;
    input logic [31:0] a, b;
begin
    divider_if.a = a;
    divider_if.b = b;
    divider_if.is_signed = 0;

    @(negedge clk);
    divider_if.en = 1;
    @(negedge clk);
    divider_if.en = 0;

    wait(divider_if.ready);

    if(b == 0) begin
        if(divider_if.div_by_zero == 0 || divider_if.q != '1 || divider_if.r != divider_if.a) begin
            $fatal("Unsigned division test failed: %d / %d = %d R %d, expected div_by_zero and %d R %d", a, b, divider_if.q, divider_if.r, 32'hFFFFFFFF, divider_if.a);
        end else begin
            $display("Unsigned division test passed: %d / %d = %d R %d -> div_by_zero and %d R %d", a, b, divider_if.q, divider_if.r, 32'hFFFFFFFF, divider_if.a);
        end
    end else begin
        if(divider_if.q != a/b || divider_if.r != a%b) begin
            $fatal("Unsigned division test failed: %d / %d = %d R %d, expected %d R %d", a, b, divider_if.q, divider_if.r, a/b, a%b);
        end else begin
            $display("Unsigned division test passed: %d / %d = %d R %d", a, b, divider_if.q, divider_if.r);
        end
    end

    @(posedge clk);
end
endtask

task test_divide_signed;
    input logic [31:0] a, b;
    logic signed [31:0] expected_q, expected_r;
begin
    divider_if.a = a;
    divider_if.b = b;
    divider_if.is_signed = 1;
    if(b == 0) begin
        expected_q = '1;
        expected_r = a;
    end else if (a == 32'h80000000 && b == 32'hFFFFFFFF) begin
        expected_q = 32'h80000000;
        expected_r = '0;
    end else begin
        expected_q = $signed(a)/$signed(b);
        expected_r = $signed(a)%$signed(b);
    end

    @(negedge clk);
    divider_if.en = 1;
    @(negedge clk);
    divider_if.en = 0;

    wait(divider_if.ready);

    if(b == 0) begin
        if(divider_if.div_by_zero == 0 || divider_if.q != '1 || divider_if.r != divider_if.a) begin
            $fatal("Signed division test failed: %d / %d = %d R %d, expected div_by_zero and %d R %d", $signed(a), $signed(b), $signed(divider_if.q), $signed(divider_if.r), $signed(32'hFFFFFFFF), $signed(divider_if.a));
        end else begin
            $display("Signed division test passed: %d / %d = %d R %d -> div_by_zero and %d R %d", $signed(a), $signed(b), $signed(divider_if.q), $signed(divider_if.r), $signed(32'hFFFFFFFF), $signed(divider_if.a));
        end
    end else if (divider_if.a == 32'h80000000 && divider_if.b == 32'hFFFFFFFF) begin
        if(divider_if.overflow == 0 || divider_if.q != 32'h80000000 || divider_if.r != '0) begin
            $fatal("Signed division test failed: %d / %d = %d R %d, expected overflow and %d R %d", $signed(a), $signed(b), $signed(divider_if.q), $signed(divider_if.r), $signed(32'h80000000), '0);
        end else begin
            $display("Signed division test passed: %d / %d = %d R %d -> overflow and %d R %d", $signed(a), $signed(b), $signed(divider_if.q), $signed(divider_if.r), $signed(32'h80000000), '0);
        end
    end else begin
        if($signed(divider_if.q) != expected_q || $signed(divider_if.r) != expected_r) begin
            $fatal("Signed division test failed: %d / %d = %d R %d, expected %d R %d", $signed(a), $signed(b), $signed(divider_if.q), $signed(divider_if.r), expected_q, expected_r);
        end else begin
            $display("Signed division test passed: %d / %d = %d R %d", $signed(a), $signed(b), $signed(divider_if.q), $signed(divider_if.r));
        end
    end

    @(posedge clk);
end
endtask

logic [31:0] a, b;
integer i;

initial begin
    clk = 0;
    forever #5 clk = ~clk;
end

initial begin
    divider_if.en = 0;
    divider_if.is_signed = 0;
    nrst = 0;
    #30;
    nrst = 1;

    a = 0;
    b = 0;

    // Divide by zero
    test_divide(32'd10, 32'd0);

    // Divide by one
    test_divide(32'h8210AB90, 32'd1);

    // Divide all ones by 1
    test_divide('1, 32'd1);

    for (i = 0; i <= 1000; i+=1) begin
        a = $urandom();
        b = $urandom();
        test_divide(a, b);
    end

    // Divide by zero
    test_divide_signed(32'd10, 32'd0);

    // Divide by one
    test_divide_signed(32'h8210AB90, 32'd1);

    // Divide all ones by 1
    test_divide_signed('1, 32'd1);

    // Overflow
    test_divide_signed(32'h80000000, 32'hFFFFFFFF);

    // Test case
    test_divide_signed(32'b10000000000000000101011000100111, 32'b01000000000000000010101101000100);

    for (i = 0; i <= 1000; i+=1) begin
        a = $random();
        b = $random();
        test_divide_signed(a, b);
    end

    $finish;
end

endmodule
