`timescale 1ns/1ns

`include "multiplier_if.vh"
`include "common_types.vh"
import common_types_pkg::*;
`include "common_gnss_types.vh"
import common_gnss_types_pkg::*;

localparam N_DOP = 21;
localparam N_CODE = 2046;

module l1ca_search (
    input logic clk, nrst,                      // Clock and reset
    input logic signal_in,                      // Input signal
    input logic start,                          // Start acquisition
    input sv_t sv,                              // SV number to search
    output word_t acc_out,                      // Maximum correlation
    output logic [10:0] code_index,             // Chip index of maximum correlation
    output logic [4:0] dop_index,               // Doppler index of maximum correlation
    output logic busy                           // Busy signal
);

logic signed [15:0] acc_i [0:N_DOP-1];          // Accumulator for I channel (each dop bin)
logic signed [15:0] next_acc_i [0:N_DOP-1];     // Next accumulator for I channel (each dop bin)
logic signed [15:0] acc_q [0:N_DOP-1];          // Accumulator for Q channel (each dop bin)
logic signed [15:0] next_acc_q [0:N_DOP-1];     // Next accumulator for Q channel (each dop bin)
logic signed [31:0] i_sqr, next_i_sqr;          // I channel squared
logic signed [31:0] q_sqr, next_q_sqr;          // Q channel squared

typedef enum logic [2:0] {
    IDLE,
    SAMPLE,
    CODE_WIND,
    DOP_SEARCH,
    MAX_SEARCH
} search_state_t;

search_state_t state, next_state;

logic sample_wen, sample_ren;                   // Write and read enable for sample memory
logic [14:0] sample_ctr, next_sample_ctr;       // Sample counter
logic sample_out;

logic [4:0] dop_ctr, next_dop_ctr;              // Doppler bin counter
logic [10:0] code_ctr, next_code_ctr;           // Code phase bin counter

logic [31:0] next_acc_out;                      // Maximum correlation output
logic [10:0] next_code_index;                   // Next maximum code bin
logic [4:0] next_dop_index;                     // Next maximum dop bin

logic [1:0] max_calc_step, next_max_calc_step;  // Step for maximum calc: 0 - start, 1 - I*I, 2 - Q*Q, 3 - I+Q and compare

logic next_done;

// Sample memory
xpm_memory_spram #(
    .ADDR_WIDTH_A(15),
    .MEMORY_SIZE(19200),
    .WRITE_DATA_WIDTH_A(1),
    .BYTE_WRITE_WIDTH_A(1),
    .READ_DATA_WIDTH_A(1),
    .READ_LATENCY_A(1),
    .RST_MODE_A("ASYNC"),
    .MEMORY_PRIMITIVE("block")
) ram_inst (
    .clka(clk),
    .rsta(~nrst),
    .ena(sample_wen | sample_ren),
    .wea(sample_wen),
    .addra(sample_ctr),
    .dina(signal_in),
    .douta(sample_out),
    .injectdbiterra(1'b0),
    .injectsbiterra(1'b0),
    .regcea(1'b1),
    .sleep(1'b0)
);

logic code_strobe;
logic code_clear;
logic code;
logic [9:0] code_num;

// Code generator
l1ca_code code_gen (
    .clk(clk),
    .nrst(nrst),
    .en(code_strobe),
    .clear(code_clear),
    .sv(sv),
    .code(code),
    .chip(code_num)
);

// Multiplier
logic [15:0] mult_a, mult_b;
logic [31:0] mult_p;
logic mult_start;
logic mult_busy;

mult_16x16 mult (
    .clk(clk),
    .nrst(nrst),
    .start(mult_start),
    .busy(mult_busy),
    .a(mult_a),
    .b(mult_b),
    .p(mult_p)
);

// Code NCOs
logic [31:0] code_phase;
logic [32:0] next_code_phase;

localparam CODE_RATE = 32'd228841226; // Ignore doppler for such a short sample

// LO NCOs
logic [15:0] lo_phase [0:N_DOP-1];
logic [15:0] next_lo_phase [0:N_DOP-1];

localparam logic [15:0] LO_RATE [0:N_DOP-1] = {
    16'd13704,
    16'd13706,
    16'd13707,
    16'd13709,
    16'd13711,
    16'd13713,
    16'd13714,
    16'd13716,
    16'd13718,
    16'd13719,
    16'd13721,
    16'd13723,
    16'd13725,
    16'd13726,
    16'd13728,
    16'd13730,
    16'd13731,
    16'd13733,
    16'd13735,
    16'd13736,
    16'd13738
};

localparam LO_SIN = 4'b0011;
localparam LO_COS = 4'b1001;

always_ff @(posedge clk) begin
    if (!nrst) begin
        state <= IDLE;
        for (int i = 0; i < N_DOP; i++) begin
            acc_i[i] <= 0;
            acc_q[i] <= 0;
            lo_phase[i] <= 0;
        end
        sample_ctr <= 0;
        dop_ctr <= 0;
        code_ctr <= 0;
        code_phase <= 0;
        acc_out <= 0;
        code_index <= 0;
        dop_index <= 0;
        max_calc_step <= 0;
        i_sqr <= 0;
        q_sqr <= 0;
    end else begin
        state <= next_state;
        for (int i = 0; i < N_DOP; i++) begin
            acc_i[i] <= next_acc_i[i];
            acc_q[i] <= next_acc_q[i];
            lo_phase[i] <= next_lo_phase[i];
        end
        sample_ctr <= next_sample_ctr;
        dop_ctr <= next_dop_ctr;
        code_ctr <= next_code_ctr;
        code_phase <= next_code_phase[31:0];
        acc_out <= next_acc_out;
        code_index <= next_code_index;
        dop_index <= next_dop_index;
        max_calc_step <= next_max_calc_step;
        i_sqr <= next_i_sqr;
        q_sqr <= next_q_sqr;
    end
end

// Next state logic
always_comb begin
    next_state = state;
    case (state)
        IDLE: begin
            if (start) begin
                next_state = SAMPLE;
            end
        end
        SAMPLE: begin
            if (sample_ctr == 15'd19200-15'd1) begin
                next_state = CODE_WIND;
            end
        end
        CODE_WIND: begin
            if (code_num == code_ctr[10:1]) begin
                next_state = DOP_SEARCH;
            end
        end
        DOP_SEARCH: begin
            if (sample_ctr == 15'd19200-15'd1) begin
                next_state = MAX_SEARCH;
            end
        end
        MAX_SEARCH: begin
            if (dop_ctr == N_DOP-5'd1 && max_calc_step == 2'd3) begin
                // Next code phase or done
                if (code_ctr == 11'd2046-11'd1) begin
                    next_state = IDLE;
                end else begin
                    next_state = CODE_WIND;
                end
            end
        end
    endcase
end

// State output logic
always_comb begin
    sample_wen = 0;
    sample_ren = 0;
    next_sample_ctr = sample_ctr;
    next_dop_ctr = dop_ctr;
    next_code_ctr = code_ctr;

    next_code_phase = {1'b0, code_phase};
    
    for (int i = 0; i < N_DOP; i++) begin
        next_lo_phase[i] = lo_phase[i];
        next_acc_i[i] = acc_i[i];
        next_acc_q[i] = acc_q[i];
    end

    code_strobe = 0;
    code_clear = 0;

    next_code_index = code_index;
    next_dop_index = dop_index;

    next_max_calc_step = max_calc_step;

    next_acc_out = acc_out;

    next_i_sqr = i_sqr;
    next_q_sqr = q_sqr;

    // Multiplier
    mult_start = 0;
    mult_a = 0;
    mult_b = 0;

    busy = 1'b1;

    case (state)
        IDLE: begin
            next_sample_ctr = 0;
            next_dop_ctr = 0;
            next_code_ctr = 0;

            code_clear = 1;

            busy = 1'b0;
        end
        SAMPLE: begin
            sample_wen = 1;
            next_sample_ctr = sample_ctr + 1;

            if (sample_ctr == 15'd19200-15'd1) begin
                // Clear NCOs
                for (int i = 0; i < N_DOP; i++) begin
                    lo_phase[i] = 0;
                    next_acc_i[i] = 0;
                    next_acc_q[i] = 0;
                end

                // Clear sample counter
                next_sample_ctr = 0;

                // Clear max, doppler and code max index
                next_acc_out = 0;
                next_dop_index = 0;
                next_code_index = 0;
            end
        end
        CODE_WIND: begin
            code_strobe = 1;

            if (code_num == code_ctr[10:1]) begin
                // Half step
                if (code_ctr[0]) begin
                    next_code_phase = {1'b0, code_phase} + {1'b0, 32'h8000_0000};
                end else begin
                    next_code_phase = '0;
                end
                
                // Stop incrementing
                code_strobe = 0;

                // Start to read sample
                sample_ren = 1;
                next_sample_ctr = sample_ctr + 1;
            end
        end

        DOP_SEARCH: begin
            // Read sample
            sample_ren = 1;

            // Accumulate
            for (int i = 0; i < N_DOP; i++) begin
                next_acc_i[i] = acc_i[i] + ((sample_out ^ code ^ LO_SIN[lo_phase[i][15:14]]) ? 16'd1 : -16'd1);
                next_acc_q[i] = acc_q[i] + ((sample_out ^ code ^ LO_COS[lo_phase[i][15:14]]) ? 16'd1 : -16'd1);
                next_lo_phase[i] = lo_phase[i] + LO_RATE[i];
            end
            // Increment code phase
            next_code_phase = {1'b0, code_phase} + {1'b0, CODE_RATE};

            // If code phase is overflowing, strobe the code generator
            if (next_code_phase[32]) begin
                code_strobe = 1;
            end

            // Increment sample counter
            next_sample_ctr = sample_ctr + 1;
        end
        
        MAX_SEARCH: begin
            // Steps
            case (max_calc_step)
                2'd0: begin
                    // Start
                    mult_start = 1;
                    mult_a = {{16{acc_i[dop_ctr][15]}}, acc_i[dop_ctr]};
                    mult_b = {{16{acc_i[dop_ctr][15]}}, acc_i[dop_ctr]};
                    next_max_calc_step = 2'd1;
                end
                2'd1: begin
                    // I*I
                    mult_start = 1;
                    mult_a = {{16{acc_i[dop_ctr][15]}}, acc_i[dop_ctr]};
                    mult_b = {{16{acc_i[dop_ctr][15]}}, acc_i[dop_ctr]};

                    if (!mult_busy) begin
                        mult_a = {{16{acc_q[dop_ctr][15]}}, acc_q[dop_ctr]};
                        mult_b = {{16{acc_q[dop_ctr][15]}}, acc_q[dop_ctr]};
                        next_max_calc_step = 2'd2;
                        next_i_sqr = mult_p;
                    end
                end
                2'd2: begin
                    // Q*Q
                    mult_start = 1;
                    mult_a = {{16{acc_q[dop_ctr][15]}}, acc_q[dop_ctr]};
                    mult_b = {{16{acc_q[dop_ctr][15]}}, acc_q[dop_ctr]};

                    if (!mult_busy) begin
                        mult_start = 0;
                        next_max_calc_step = 2'd3;
                        next_q_sqr = mult_p;
                    end
                end
                2'd3: begin
                    // I+Q and compare
                    if (i_sqr + q_sqr > acc_out) begin
                        next_acc_out = i_sqr + q_sqr;
                        next_code_index = code_ctr;
                        next_dop_index = dop_ctr;
                    end

                    // Next doppler bin
                    next_dop_ctr = dop_ctr + 5'd1;
                    // Next step
                    next_max_calc_step = 2'd0;

                    if (dop_ctr == N_DOP-5'd1) begin
                        for (int i = 0; i < N_DOP; i++) begin
                            // Clear NCO
                            next_lo_phase[i] = 0;
                            // Clear accumulators
                            next_acc_i[i] = 0;
                            next_acc_q[i] = 0;
                        end
                        // Clear NCO
                        next_code_phase = 0;

                        // Clear sample counter
                        next_sample_ctr = 0;

                        // Clear doppler counter
                        next_dop_ctr = 0;

                        // Clear code generator
                        code_clear = 1;

                        // Next code phase
                        next_code_ctr = code_ctr + 1;
                    end
                end
            endcase
        end
    endcase
end

endmodule