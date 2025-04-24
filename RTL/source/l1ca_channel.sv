`timescale 1ns/1ns

`include "common_gnss_types.vh"
import common_gnss_types_pkg::*;

module l1ca_channel (
    input logic clk, nrst,                  // Clock and reset
    input logic en, clear,                  // Module enable and clear
    input sv_t sv,                          // SV number for code generation
    input logic delay_code, delay_cycles,   // Code delay control
    input logic [31:0] code_rate, lo_rate,  // Code and local oscillator rates
    input logic signal_in,                  // Signal input
    output logic epoch,                     // Epoch signal
    output gps_chip_t chip,                 // Chip index from code generator
    output acc_t ie, qe,                    // Early in-phase and quadrature phase output accumulator
    output acc_t ip, qp,                    // Prompt in-phase and quadrature phase output accumulator
    output acc_t il, ql,                    // Late in-phase and quadrature phase output accumulator
    output logic [31:0] code_phase, lo_phase    // Code and local oscillator phases
);
    logic epoch_reg;

    acc_t next_ie, next_qe; // Next state for accumulators
    acc_t next_ip, next_qp; // Next state for accumulators
    acc_t next_il, next_ql; // Next state for accumulators
    logic die, dip, dil;    // in-phase DLL signals
    logic dqe, dqp, dql;    // quadrature phase DLL signals

    logic [32:0] next_code_phase;   // Next state for code phase + overflow bit
    logic code_strobe;      // Code strobe from NCO
    logic delay_strobe;     // Delay strobe from NCO (twice rate of code strobe)

    logic code_early, code_prompt, code_late; // Code samples for early, prompt, and late
    logic next_code_prompt, next_code_late; // Next state for code samples

    logic [31:0] next_lo_phase; // Next state for local oscillator phase
    logic lo_cos, lo_sin;       // Local oscillator outputs

    // Accumulator and DLL
    always_ff @(posedge clk) begin
        if (~nrst) begin
            ip <= '0;
            qp <= '0;
            ie <= '0;
            qe <= '0;
            il <= '0;
            ql <= '0;
            code_prompt <= '0;
            code_late <= '0;
            epoch_reg <= '0;
        end else begin
            ip <= next_ip;
            qp <= next_qp;
            ie <= next_ie;
            qe <= next_qe;
            il <= next_il;
            ql <= next_ql;
            code_late <= next_code_late;
            code_prompt <= next_code_prompt;
            epoch_reg <= epoch;
        end
    end

    // Accumulator and DLL
    always_comb begin
        next_ip = ip;
        next_qp = qp;
        next_ie = ie;
        next_qe = qe;
        next_il = il;
        next_ql = ql;
        next_code_prompt = code_prompt;
        next_code_late = code_late;

        // Mixers
        die = code_early ^ signal_in ^ lo_sin;
        dip = code_prompt ^ signal_in ^ lo_sin;
        dil = code_late ^ signal_in ^ lo_sin;
        dqe = code_early ^ signal_in ^ lo_cos;
        dqp = code_prompt ^ signal_in ^ lo_cos;
        dql = code_late ^ signal_in ^ lo_cos;

        if (clear) begin
            next_ip = '0;
            next_qp = '0;
            next_code_prompt = '0;
            next_code_late = '0;
        end else if (en) begin
            if (delay_strobe) begin
                // Shift code samples
                next_code_late = code_prompt;
                next_code_prompt = code_early;
            end

            // Accumulate in-phase and quadrature components
            next_ie = ie + (die ? 16'd1 : -16'd1);
            next_qe = qe + (dqe ? 16'd1 : -16'd1);
            next_ip = ip + (dip ? 16'd1 : -16'd1);
            next_qp = qp + (dqp ? 16'd1 : -16'd1);
            next_il = il + (dil ? 16'd1 : -16'd1);
            next_ql = ql + (dql ? 16'd1 : -16'd1);

            // On rising edge of epoch, clear the accumulators
            if (epoch & ~epoch_reg) begin
                next_ie = '0;
                next_qe = '0;
                next_ip = '0;
                next_qp = '0;
                next_il = '0;
                next_ql = '0;
            end
        end
    end

    // Code generator
    l1ca_code code_gen (
        .clk(clk),
        .nrst(nrst),
        .en(code_strobe & en),
        .clear(clear),
        .sv(sv),
        .code(code_early),
        .epoch(epoch),
        .chip(chip)
    );

    // Code NCO
    always_ff @(posedge clk) begin
        if (~nrst) begin
            code_phase = '0;
        end else begin
            code_phase <= next_code_phase[31:0];
        end
    end

    // Code NCO
    always_comb begin
        next_code_phase = {1'b0, code_phase};

        code_strobe = 1'b0;
        delay_strobe = 1'b0;

        if (clear) begin
            next_code_phase = '0;
        end else if (en) begin
            next_code_phase = {1'b0, code_phase} + {1'b0, code_rate};
        end

        // Overflow of code phase should trigger the code strobe
        if (next_code_phase[32]) begin
            code_strobe = 1'b1;
        end

        // Delay strobe is twice the rate of code strobe
        if (next_code_phase[31] ^ code_phase[31]) begin
            delay_strobe = 1'b1;
        end
    end

    // Local oscillator NCO
    always_ff @(posedge clk) begin
        if (~nrst) begin
            lo_phase = '0;
        end else begin
            lo_phase <= next_lo_phase;
        end
    end

    // Local oscillator NCO
    always_comb begin
        next_lo_phase = lo_phase;
        lo_cos = 1'b0;
        lo_sin = 1'b0;

        if (clear) begin
            next_lo_phase = '0;
        end else if (en) begin
            next_lo_phase = lo_phase + lo_rate;

            // Generate local oscillator output
            case (lo_phase[31:30])
                2'b00: begin
                    lo_cos = 1'b1;
                    lo_sin = 1'b1;
                end
                2'b01: begin
                    lo_cos = 1'b0;
                    lo_sin = 1'b1;
                end
                2'b10: begin
                    lo_cos = 1'b0;
                    lo_sin = 1'b0;
                end
                2'b11: begin
                    lo_cos = 1'b1;
                    lo_sin = 1'b0;
                end
            endcase
        end
    end

endmodule