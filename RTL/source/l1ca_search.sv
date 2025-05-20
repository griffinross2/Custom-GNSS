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
    output logic done                           // Done signal
);

logic signed [31:0] acc_i [0:N_DOP-1];                // Accumulator for I channel (each dop bin)
logic signed [31:0] next_acc_i [0:N_DOP-1];           // Next accumulator for I channel (each dop bin)
logic signed [31:0] acc_q [0:N_DOP-1];                // Accumulator for Q channel (each dop bin)
logic signed [31:0] next_acc_q [0:N_DOP-1];           // Next accumulator for Q channel (each dop bin)

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

logic [31:0] next_acc_out;                           // Maximum correlation output
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
multiplier_if mulif ();

multiplier mult (
    .clk(clk),
    .nrst(nrst),
    .multiplier_if(mulif)
);

// Code NCOs
logic [31:0] code_phase;
logic [32:0] next_code_phase;

localparam CODE_RATE = 32'd228841226; // Ignore doppler for such a short sample

// LO NCOs
word_t lo_phase [0:N_DOP-1];
word_t next_lo_phase [0:N_DOP-1];

localparam word_t LO_RATE [0:N_DOP-1] = {
    32'd898140296,
    32'd898252144,
    32'd898363992,
    32'd898475840,
    32'd898587688,
    32'd898699537,
    32'd898811385,
    32'd898923233,
    32'd899035081,
    32'd899146929,
    32'd899258777,
    32'd899370625,
    32'd899482473,
    32'd899594321,
    32'd899706170,
    32'd899818018,
    32'd899929866,
    32'd900041714,
    32'd900153562,
    32'd900265410,
    32'd900377258
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
        done <= 0;
    end else begin
        state <= next_state;
        for (int i = 0; i < N_DOP; i++) begin
            acc_i[i] <= next_acc_i[i];
            acc_q[i] <= next_acc_q[i];
            lo_phase[i] = next_lo_phase[i];
        end
        sample_ctr <= next_sample_ctr;
        dop_ctr <= next_dop_ctr;
        code_ctr <= next_code_ctr;
        code_phase <= next_code_phase[31:0];
        acc_out <= next_acc_out;
        code_index <= next_code_index;
        dop_index <= next_dop_index;
        max_calc_step <= next_max_calc_step;
        done <= next_done;
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
    next_lo_phase = lo_phase;

    next_acc_i = acc_i;
    next_acc_q = acc_q;

    code_strobe = 0;
    code_clear = 0;

    next_code_index = code_index;
    next_dop_index = dop_index;

    next_max_calc_step = max_calc_step;

    next_acc_out = acc_out;

    // Multiplier
    mulif.en = 0;
    mulif.a = 0;
    mulif.b = 0;
    mulif.is_signed_a = 1;
    mulif.is_signed_b = 1;

    next_done = done;

    case (state)
        IDLE: begin
            next_sample_ctr = 0;
            next_dop_ctr = 0;
            next_code_ctr = 0;

            code_clear = 1;

            if (start) begin
                next_done = 0;
            end
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
                next_acc_i[i] = acc_i[i] + ((sample_out ^ code ^ LO_SIN[lo_phase[i][1:0]]) ? 32'd1 : -32'd1);
                next_acc_q[i] = acc_q[i] + ((sample_out ^ code ^ LO_COS[lo_phase[i][1:0]]) ? 32'd1 : -32'd1);
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
                    mulif.en = 1;
                    mulif.a = acc_i[dop_ctr];
                    mulif.b = acc_i[dop_ctr];
                    next_max_calc_step = 2'd1;
                end
                2'd1: begin
                    // I*I
                    mulif.en = 1;
                    mulif.a = acc_i[dop_ctr];
                    mulif.b = acc_i[dop_ctr];

                    if (mulif.ready) begin
                        mulif.a = acc_q[dop_ctr];
                        mulif.b = acc_q[dop_ctr];
                        next_max_calc_step = 2'd2;
                        next_acc_i[dop_ctr] = mulif.out[31:0];
                    end
                end
                2'd2: begin
                    // Q*Q
                    mulif.en = 1;
                    mulif.a = acc_q[dop_ctr];
                    mulif.b = acc_q[dop_ctr];

                    if (mulif.ready) begin
                        mulif.en = 0;
                        next_max_calc_step = 2'd3;
                        next_acc_q[dop_ctr] = mulif.out[31:0];
                    end
                end
                2'd3: begin
                    // I+Q and compare
                    if (acc_i[dop_ctr] + acc_q[dop_ctr] > acc_out) begin
                        next_acc_out = acc_i[dop_ctr] + acc_q[dop_ctr];
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

                        if (code_ctr == 11'd2046-11'd1) begin
                            // Next code phase
                            next_code_ctr = 0;

                            // Done
                            next_done = 1;
                        end
                    end
                end
            endcase
        end
    endcase
end

endmodule