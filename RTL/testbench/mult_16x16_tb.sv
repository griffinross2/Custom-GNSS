module mult_16x16_tb;

logic clk, nrst;

logic signed [15:0] a_in, b_in;
logic signed [31:0] p_out;
logic busy;
logic start;

mult_16x16 mult_inst (
    .clk(clk),
    .nrst(nrst),
    .a(a_in),
    .b(b_in),
    .p(p_out),
    .busy(busy),
    .start(start)
);

task test_multiply_signed;
    input logic [15:0] a, b;
begin
    a_in = a;
    b_in = b;
    
    @(negedge clk);
    start = 1;
    @(negedge clk);
    start = 0;

    wait(busy == 1'b1);
    wait(busy == 1'b0);
    @(negedge clk);

    if($signed(p_out) != $signed({{16{a_in[15]}}, a_in})*$signed({{16{b_in[15]}}, b_in})) begin
        $fatal("Test failed: %d * %d = %d, expected %d", $signed({{16{a_in[15]}}, a_in}), $signed({{16{b_in[15]}}, b_in}), $signed(p_out), $signed({{16{a_in[15]}}, a_in})*$signed({{16{b_in[15]}}, b_in}));
    end else begin
        $display("Test passed: %d * %d = %d", $signed({{16{a_in[15]}}, a_in}), $signed({{16{b_in[15]}}, b_in}), $signed(p_out));
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
    start = 0;
    nrst = 0;
    @(posedge clk);
    @(posedge clk);
    nrst = 1;

    a = 0;
    b = 0;

    for (i = 0; i <= 100000; i+=1) begin
        a = $random();
        b = $random();
        test_multiply_signed(a[15:0], b[15:0]);
    end

    $finish;
end

endmodule
