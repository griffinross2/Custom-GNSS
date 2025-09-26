`timescale 1ns/1ns

`include "common_gnss_types.vh"
import common_gnss_types_pkg::*;

module l1ca_channel (
    input logic clk, nrst,                      // Clock and reset
    input logic start, clear,                   // Start the channel
    input sv_t sv,                              // SV number for code generation
    input logic [31:0] code_rate, lo_rate,      // Code and local oscillator rates
    input logic signal_in,                      // Signal input
    input logic [9:0] code_index,               // Chip index of maximum correlation -> 0 thru 1022
    input logic [4:0] start_index,              // Sample start index of maximum correlation -> 0 thru 18
    output logic epoch,                         // Epoch signal
    output gps_chip_t chip,                     // Chip index from code generator
    output acc_t ie, qe,                        // Early in-phase and quadrature phase output accumulator
    output acc_t ip, qp,                        // Prompt in-phase and quadrature phase output accumulator
    output acc_t il, ql,                        // Late in-phase and quadrature phase output accumulator
    output logic [31:0] code_phase, lo_phase    // Code and local oscillator phases
);

    typedef enum logic [1:0] {
        IDLE = 2'b00,       // Idle state
        CODE_WIND = 2'b01,  // Code wind
        ACTIVE = 2'b10      // Tracking
    } channel_state_t;

    channel_state_t state, next_state;

    logic code_epoch, epoch_reg;
    logic [9:0] code_index_reg;
    logic [4:0] start_index_reg;

    logic [9:0] code_wind_slip, next_code_wind_slip; // Code slip incurred during code wind

    sv_t sv_reg, next_sv;

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
            sv_reg <= '0;
            code_index_reg <= '0;
            start_index_reg <= '0;
            code_wind_slip <= '0;
            state <= IDLE;
            code_phase = '0;
            lo_phase = '0;
        end else begin
            ip <= next_ip;
            qp <= next_qp;
            ie <= next_ie;
            qe <= next_qe;
            il <= next_il;
            ql <= next_ql;
            code_late <= next_code_late;
            code_prompt <= next_code_prompt;
            epoch_reg <= code_epoch;
            sv_reg <= next_sv;
            if (start) begin
                code_index_reg <= code_index;
            end else begin
                code_index_reg <= code_index_reg;
            end
            start_index_reg <= start ? start_index : start_index_reg;
            code_wind_slip <= next_code_wind_slip;
            state <= next_state;
            code_phase <= next_code_phase[31:0];
            lo_phase <= next_lo_phase;
        end
    end

    // Next state logic
    always_comb begin
        next_state = state;

        case (state)
            IDLE: begin
                if (start && !clear) begin
                    next_state = CODE_WIND;
                end
            end

            CODE_WIND: begin
                if ((chip + code_wind_slip) % 10'd1023 == code_index_reg) begin
                    next_state = ACTIVE;
                end

                if (clear) begin
                    next_state = IDLE;
                end
            end

            ACTIVE: begin
                if (clear) begin
                    next_state = IDLE;
                end
            end

            default: begin
                next_state = IDLE; // Default to IDLE state for safety
            end
        endcase
    end

    // State output
    always_comb begin
        next_ip = ip;
        next_qp = qp;
        next_ie = ie;
        next_qe = qe;
        next_il = il;
        next_ql = ql;
        next_code_prompt = code_prompt;
        next_code_late = code_late;
        next_sv = sv_reg;
        next_code_wind_slip = code_wind_slip;

        // Code NCO
        next_code_phase = {1'b0, code_phase} + {1'b0, code_rate};
        code_strobe = 1'b0;
        delay_strobe = 1'b0;

        // Overflow of code phase should trigger the code strobe
        if (next_code_phase[32]) begin
            code_strobe = 1'b1;
        end

        // Delay strobe is twice the rate of code strobe
        if (next_code_phase[31] ^ code_phase[31]) begin
            delay_strobe = 1'b1;
        end

        // Local oscillator NCO
        next_lo_phase = lo_phase + lo_rate;
        lo_cos = 1'b0;
        lo_sin = 1'b0;

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

        // Mixers
        die = code_early ^ signal_in ^ lo_sin;
        dip = code_prompt ^ signal_in ^ lo_sin;
        dil = code_late ^ signal_in ^ lo_sin;
        dqe = code_early ^ signal_in ^ lo_cos;
        dqp = code_prompt ^ signal_in ^ lo_cos;
        dql = code_late ^ signal_in ^ lo_cos;

        case (state)
            IDLE: begin
                next_ip = '0;
                next_qp = '0;
                next_ie = '0;
                next_qe = '0;
                next_il = '0;
                next_ql = '0;
                next_code_wind_slip = '0;
                next_code_prompt = '0;
                next_code_late = '0;
                next_code_phase = '0;
                next_lo_phase = '0;
            end

            CODE_WIND: begin
                // Constantly wind the code here
                code_strobe = 1'b1;

                if (next_code_phase[32]) begin
                    next_code_wind_slip = code_wind_slip + 10'd1;
                end

                if ((chip + code_wind_slip) % 10'd1023 == code_index_reg) begin
                    // Set subcode slip as starting code phase offset
                    case (start_index)
                        5'd0: next_code_phase = next_code_phase + 33'd0;
                        5'd1: next_code_phase = next_code_phase + 33'd228820847;
                        5'd2: next_code_phase = next_code_phase + 33'd457641694;
                        5'd3: next_code_phase = next_code_phase + 33'd686462541;
                        5'd4: next_code_phase = next_code_phase + 33'd915283388;
                        5'd5: next_code_phase = next_code_phase + 33'd1144104234;
                        5'd6: next_code_phase = next_code_phase + 33'd1372925081;
                        5'd7: next_code_phase = next_code_phase + 33'd1601745928;
                        5'd8: next_code_phase = next_code_phase + 33'd1830566775;
                        5'd9: next_code_phase = next_code_phase + 33'd2059387622;
                        5'd10: next_code_phase = next_code_phase + 33'd2288208469;
                        5'd11: next_code_phase = next_code_phase + 33'd2517029316;
                        5'd12: next_code_phase = next_code_phase + 33'd2745850163;
                        5'd13: next_code_phase = next_code_phase + 33'd2974671009;
                        5'd14: next_code_phase = next_code_phase + 33'd3203491856;
                        5'd15: next_code_phase = next_code_phase + 33'd3432312703;
                        5'd16: next_code_phase = next_code_phase + 33'd3661133550;
                        5'd17: next_code_phase = next_code_phase + 33'd3889954397;
                        5'd18: next_code_phase = next_code_phase + 33'd4118775244;
                    endcase
                end
            end

            ACTIVE: begin
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
                if (code_epoch & ~epoch_reg) begin
                    next_ie = '0;
                    next_qe = '0;
                    next_ip = '0;
                    next_qp = '0;
                    next_il = '0;
                    next_ql = '0;
                end
            end
        endcase
        
        if (clear) begin
            next_ip = '0;
            next_qp = '0;
            next_ie = '0;
            next_qe = '0;
            next_il = '0;
            next_ql = '0;
            next_code_wind_slip = '0;
            next_code_prompt = '0;
            next_code_late = '0;
            next_code_phase = '0;
            next_lo_phase = '0;
        end

        // Rising edge of code epoch, assert epoch output
        epoch = code_epoch & ~epoch_reg;
    end

    // Code generator
    l1ca_code code_gen (
        .clk(clk),
        .nrst(nrst),
        .en(code_strobe),
        .clear(state == IDLE),
        .sv(sv_reg),
        .code(code_early),
        .epoch(code_epoch),
        .chip(chip)
    );

endmodule