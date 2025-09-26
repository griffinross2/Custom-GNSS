/**************************************************************************************/
/*                        Averaging Correlation PCA Search                            */
/*                             Using 4096-length FFT                                  */
/*                                   Based on:                                        */
/*                                                                                    */
/*                             J. A. Starzyk and Z. Zhu,                              */
/* "Averaging correlation for C/A code acquisition and tracking in frequency domain," */
/*    Proceedings of the 44th IEEE 2001 Midwest Symposium on Circuits and Systems.    */
/*                 MWSCAS 2001 (Cat. No.01CH37257), Dayton, OH, USA,                  */
/*                            2001, pp. 905-908 vol.2,                                */
/*                        doi: 10.1109/MWSCAS.2001.986334.                            */
/**************************************************************************************/

`include "common_types.vh"
import common_types_pkg::*;
`include "common_gnss_types.vh"
import common_gnss_types_pkg::*;

// Takes 0.224068832 s to complete
module ac_pca_search (
    input logic clk, nrst,                      // Clock and reset
    input logic signal_in,                      // Input signal
    input logic start,                          // Start acquisition
    input sv_t sv,                              // SV number to search
    input logic [5:0] channel_in,               // Channel number to initialize after acquisition
    output word_t acc_out,                      // Maximum correlation
    output logic [9:0] code_index,              // Chip index of maximum correlation -> 0 thru 1022
    output logic [4:0] start_index,             // Sample start index of maximum correlation -> 0 thru 18
    output logic [5:0] dop_index,               // Doppler index of maximum correlation 0 thru 40 -> -5000 thru 5000 Hz in 250 Hz steps
    output logic [5:0] channel_out,             // Channel number to initialize after acquisition
    output sv_t sv_out,                         // SV number to initialize after acquisition
    output logic start_out,                     // Start signal for the channel
    output logic busy                           // Busy signal
);

typedef enum logic [2:0] {
    IDLE,
    SAMPLE,
    CODE_FFT,
    SAMPLE_FFT,
    PRODUCT_IFFT
} state_t;

typedef enum logic [1:0] {
    FFT_CONF,
    FFT_LOAD,
    FFT_WAIT
} fft_state_t;

state_t state, next_state;
fft_state_t fft_state, next_fft_state;

logic [15:0] s_axis_config_tdata;
logic s_axis_config_tvalid;
logic s_axis_config_tready;
logic [31:0] s_axis_data_tdata;
logic s_axis_data_tvalid;
logic s_axis_data_tready;
logic s_axis_data_tlast;
logic [31:0] m_axis_data_tdata;
logic [15:0] m_axis_data_tuser;
logic m_axis_data_tvalid;
logic m_axis_data_tready;
logic m_axis_data_tlast;
logic event_frame_started;
logic event_tlast_unexpected;
logic event_tlast_missing;
logic event_status_channel_halt;
logic event_data_in_channel_halt;
logic event_data_out_channel_halt;

xfft_0 fft_inst (
    .aclk(clk),
    .aresetn(nrst),
    .s_axis_config_tdata(s_axis_config_tdata),
    .s_axis_config_tvalid(s_axis_config_tvalid),
    .s_axis_config_tready(s_axis_config_tready),
    .s_axis_data_tdata(s_axis_data_tdata),
    .s_axis_data_tvalid(s_axis_data_tvalid),
    .s_axis_data_tready(s_axis_data_tready),
    .s_axis_data_tlast(s_axis_data_tlast),
    .m_axis_data_tdata(m_axis_data_tdata),
    .m_axis_data_tuser(m_axis_data_tuser),
    .m_axis_data_tvalid(m_axis_data_tvalid),
    .m_axis_data_tready(m_axis_data_tready),
    .m_axis_data_tlast(m_axis_data_tlast),
    .event_frame_started(event_frame_started),
    .event_tlast_unexpected(event_tlast_unexpected),
    .event_tlast_missing(event_tlast_missing),
    .event_status_channel_halt(event_status_channel_halt),
    .event_data_in_channel_halt(event_data_in_channel_halt),
    .event_data_out_channel_halt(event_data_out_channel_halt)
);

logic [16:0] sample_addr, next_sample_addr;
logic [11:0] sample_fft_addr, next_sample_fft_addr;
logic [11:0] code_fft_addr, next_code_fft_addr;
logic [11:0] sample_fft_addr_override;
logic [11:0] code_fft_addr_override;
logic signed [5:0] sample_i_avg, next_sample_i_avg, sample_i_avg_plus_sample;
logic signed [5:0] sample_q_avg, next_sample_q_avg, sample_q_avg_plus_sample;
logic sample_downsample_i, sample_downsample_q;
logic sample_i_in;
logic sample_q_in;
logic sample_i_out;
logic sample_q_out;
logic sample_wen;
logic sample_ren;
logic sample_fft_wen;
logic sample_fft_ren;
logic code_fft_wen;
logic code_fft_ren;
logic signed [15:0] sample_fft_i_out, sample_fft_q_out;
logic signed [15:0] code_fft_i_out, code_fft_q_out;
logic signed [31:0] si_ci_prod, sq_cq_prod, si_cq_prod, sq_ci_prod;
logic signed [5:0] doppler_step, next_doppler_step; // We will step from -20 to 20
logic [4:0] start_step, next_start_step; // Start step from 0 to 18
word_t acc_magnitude;
word_t next_acc_out;
logic [9:0] next_code_index;
logic [4:0] next_start_index;
logic [5:0] next_dop_index;
logic [5:0] next_channel_out;
sv_t next_sv_out;
logic next_start_out;

// Code NCOs
logic [31:0] code_phase;
logic [32:0] next_code_phase;
localparam CODE_RATE = 33'd228841226;

// LO NCO
logic [17:0] lo_phase;
logic [17:0] next_lo_phase;
localparam logic [17:0] LO_RATE = 18'd54886;

localparam LO_SIN = 4'b0011;
localparam LO_COS = 4'b1001;

// Sample memory
xpm_memory_spram #(
    .ADDR_WIDTH_A(17),
    .MEMORY_SIZE(76800), // 4ms at 19200000 Hz
    .WRITE_DATA_WIDTH_A(1),
    .BYTE_WRITE_WIDTH_A(1),
    .READ_DATA_WIDTH_A(1),
    .READ_LATENCY_A(1),
    .RST_MODE_A("ASYNC"),
    .MEMORY_PRIMITIVE("block")
) sample_i (
    .clka(clk),
    .rsta(~nrst),
    .ena(sample_wen | sample_ren),
    .wea(sample_wen),
    .addra(sample_addr),
    .dina(sample_i_in),
    .douta(sample_i_out),
    .injectdbiterra(1'b0),
    .injectsbiterra(1'b0),
    .regcea(1'b1),
    .sleep(1'b0)
);
xpm_memory_spram #(
    .ADDR_WIDTH_A(17),
    .MEMORY_SIZE(76800),
    .WRITE_DATA_WIDTH_A(1),
    .BYTE_WRITE_WIDTH_A(1),
    .READ_DATA_WIDTH_A(1),
    .READ_LATENCY_A(1),
    .RST_MODE_A("ASYNC"),
    .MEMORY_PRIMITIVE("block")
) sample_q (
    .clka(clk),
    .rsta(~nrst),
    .ena(sample_wen | sample_ren),
    .wea(sample_wen),
    .addra(sample_addr),
    .dina(sample_q_in),
    .douta(sample_q_out),
    .injectdbiterra(1'b0),
    .injectsbiterra(1'b0),
    .regcea(1'b1),
    .sleep(1'b0)
);

// Sample FFT results
xpm_memory_spram #(
    .ADDR_WIDTH_A(12),
    .MEMORY_SIZE(4096*32),
    .WRITE_DATA_WIDTH_A(32),
    .BYTE_WRITE_WIDTH_A(32),
    .READ_DATA_WIDTH_A(32),
    .READ_LATENCY_A(1),
    .RST_MODE_A("ASYNC"),
    .MEMORY_PRIMITIVE("block")
) sample_fft (
    .clka(clk),
    .rsta(~nrst),
    .ena(sample_fft_wen | sample_fft_ren),
    .wea(sample_fft_wen),
    .addra(sample_fft_addr_override),
    .dina(m_axis_data_tdata),
    .douta({sample_fft_q_out, sample_fft_i_out}),
    .injectdbiterra(1'b0),
    .injectsbiterra(1'b0),
    .regcea(1'b1),
    .sleep(1'b0)
);

// Code FFT results
xpm_memory_spram #(
    .ADDR_WIDTH_A(12),
    .MEMORY_SIZE(4096*32),
    .WRITE_DATA_WIDTH_A(32),
    .BYTE_WRITE_WIDTH_A(32),
    .READ_DATA_WIDTH_A(32),
    .READ_LATENCY_A(1),
    .RST_MODE_A("ASYNC"),
    .MEMORY_PRIMITIVE("block")
) code_fft (
    .clka(clk),
    .rsta(~nrst),
    .ena(code_fft_wen | code_fft_ren),
    .wea(code_fft_wen),
    .addra(code_fft_addr_override),
    .dina(m_axis_data_tdata),
    .douta({code_fft_q_out, code_fft_i_out}),
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

always_ff @(posedge clk) begin
    if (~nrst) begin
        state <= IDLE;
        sample_addr <= '0;
        sample_fft_addr <= '0;
        code_fft_addr <= '0;
        sample_i_avg <= '0;
        sample_q_avg <= '0;
        code_phase <= '0;
        lo_phase <= '0;
        fft_state <= FFT_CONF;
        doppler_step <= -6'd20;
        start_step <= '0;
        acc_out <= '0;
        code_index <= '0;
        start_index <= '0;
        dop_index <= '0;
        channel_out <= '0;
        sv_out <= '0;
        start_out <= 1'b0;
    end else begin
        state <= next_state;
        sample_addr <= next_sample_addr;
        sample_fft_addr <= next_sample_fft_addr;
        code_fft_addr <= next_code_fft_addr;
        sample_i_avg <= next_sample_i_avg;
        sample_q_avg <= next_sample_q_avg;
        code_phase <= next_code_phase[31:0];
        lo_phase <= next_lo_phase;
        fft_state <= next_fft_state;
        doppler_step <= next_doppler_step;
        start_step <= next_start_step;
        acc_out <= next_acc_out;
        code_index <= next_code_index;
        start_index <= next_start_index;
        dop_index <= next_dop_index;
        channel_out <= next_channel_out;
        sv_out <= sv;
        start_out <= next_start_out;
    end
end

always_comb begin
    next_state = state;
    case (state)
        IDLE: begin
            if (start) begin
                next_state = SAMPLE;
            end
        end
        SAMPLE: begin
            if (sample_addr == 76799) begin
                next_state = CODE_FFT;
            end
        end
        CODE_FFT: begin
            if (m_axis_data_tvalid && m_axis_data_tlast) begin
                next_state = SAMPLE_FFT;
            end
        end
        SAMPLE_FFT: begin
            if (m_axis_data_tvalid && m_axis_data_tlast) begin
                next_state = PRODUCT_IFFT;
            end
        end
        PRODUCT_IFFT: begin
            if (m_axis_data_tvalid && m_axis_data_tlast) begin
                if (doppler_step != 6'd20) begin
                    next_state = PRODUCT_IFFT;
                end else begin
                    if (start_step != 5'd18) begin
                        next_state = SAMPLE_FFT;
                    end else begin
                        next_state = IDLE;
                    end
                end
            end
        end
    endcase
end

always_comb begin
    next_fft_state = fft_state;
    next_sample_addr = sample_addr;
    next_sample_fft_addr = sample_fft_addr;
    next_code_fft_addr = code_fft_addr;
    sample_fft_addr_override = sample_fft_addr;
    code_fft_addr_override = code_fft_addr;
    next_sample_i_avg = sample_i_avg;
    next_sample_q_avg = sample_q_avg;
    sample_i_avg_plus_sample = sample_i_avg + (sample_i_out ? 6'd1 : -6'd1);
    sample_q_avg_plus_sample = sample_q_avg + (sample_q_out ? 6'd1 : -6'd1);
    sample_i_in = signal_in ^ (LO_SIN[lo_phase[17:16]]);
    sample_q_in = signal_in ^ (LO_COS[lo_phase[17:16]]);
    sample_wen = 1'b0;
    sample_ren = 1'b0;
    sample_fft_wen = 1'b0;
    sample_fft_ren = 1'b0;
    code_fft_wen = 1'b0;
    code_fft_ren = 1'b0;
    sample_downsample_i = ~sample_i_avg_plus_sample[5];
    sample_downsample_q = ~sample_q_avg_plus_sample[5];
    next_doppler_step = doppler_step;
    next_acc_out = acc_out;
    next_code_index = code_index;
    next_start_index = start_index;
    next_dop_index = dop_index;
    next_start_step = start_step;
    acc_magnitude = {{16{m_axis_data_tdata[31]}}, m_axis_data_tdata[31:16]}*{{16{m_axis_data_tdata[31]}}, m_axis_data_tdata[31:16]} + 
                    {{16{m_axis_data_tdata[15]}}, m_axis_data_tdata[15:0]}*{{16{m_axis_data_tdata[15]}}, m_axis_data_tdata[15:0]};

    next_code_phase = {1'b0, code_phase};
    next_lo_phase = lo_phase;
    next_channel_out = channel_out;
    next_sv_out = sv_out;
    next_start_out = 1'b0;

    busy = 1'b1;

    code_strobe = 1'b0;
    code_clear = 1'b0;

    s_axis_config_tdata = '0;
    s_axis_config_tvalid = 1'b0;
    s_axis_data_tdata = '0;
    s_axis_data_tvalid = 1'b0;
    s_axis_data_tlast = 1'b0;
    m_axis_data_tready = 1'b0;

    si_ci_prod = {{16{sample_fft_i_out[15]}}, sample_fft_i_out} * {{16{code_fft_i_out[15]}}, code_fft_i_out};
    sq_cq_prod = {{16{sample_fft_q_out[15]}}, sample_fft_q_out} * {{16{code_fft_q_out[15]}}, code_fft_q_out};
    si_cq_prod = {{16{sample_fft_i_out[15]}}, sample_fft_i_out} * {{16{code_fft_q_out[15]}}, code_fft_q_out};
    sq_ci_prod = {{16{sample_fft_q_out[15]}}, sample_fft_q_out} * {{16{code_fft_i_out[15]}}, code_fft_i_out};

    case (state)
        IDLE: begin
            busy = 1'b0;
            if (start) begin
                next_sample_addr = '0;
                next_sample_fft_addr = '0;
                next_code_fft_addr = '0;
                next_sample_i_avg = '0;
                next_sample_q_avg = '0;
                next_code_phase = '0;
                code_clear = 1'b1;
                next_doppler_step = -6'd20;
                next_start_step = '0;
                next_acc_out = '0;
                next_code_index = '0;
                next_start_index = '0;
                next_dop_index = '0;
                next_channel_out = channel_in;
                next_sv_out = sv;
            end
        end
        SAMPLE: begin
            next_lo_phase = lo_phase + LO_RATE;

            // Write the sample
            next_sample_addr = sample_addr + 12'd1;
            sample_wen = 1'b1;

            if (sample_addr == 76799) begin
                next_fft_state = FFT_CONF;
                next_sample_addr = '0;
            end
        end
        CODE_FFT: begin
            if (fft_state == FFT_CONF) begin
                s_axis_config_tdata = {2'd0, 12'b10_10_10_10_10_10, 1'b1}; // Forward FFT, conservative scaling
                s_axis_config_tvalid = 1'b1;
                if (s_axis_config_tready) begin
                    next_fft_state = FFT_LOAD;
                    code_fft_ren = 1'b1; // Start reading code
                    next_code_fft_addr = code_fft_addr + 12'd1;
                end
            end
            if (fft_state == FFT_LOAD) begin
                s_axis_data_tdata = {16'd0, code ? 16'h7FFF : 16'h8001};
                s_axis_data_tvalid = 1'b1;
                if (s_axis_data_tready) begin
                    code_strobe = 1'b1;
                    next_code_fft_addr = code_fft_addr + 12'd1;
                    if (code_fft_addr == '0) begin
                        next_code_fft_addr = '0;
                        s_axis_data_tlast = 1'b1;
                        next_fft_state = FFT_WAIT;
                    end
                end
            end
            if (fft_state == FFT_WAIT) begin
                m_axis_data_tready = 1'b1;
                if (m_axis_data_tvalid) begin
                    code_fft_wen = 1'b1;
                    code_fft_addr_override = m_axis_data_tuser[11:0];
                    if (m_axis_data_tlast) begin
                        // Initial indices for sample and code FFTs
                        next_sample_fft_addr = '0;
                        next_fft_state = FFT_CONF;
                    end
                end
            end
        end
        SAMPLE_FFT: begin
            if (fft_state == FFT_CONF) begin
                s_axis_config_tdata = {2'd0, 12'b10_10_10_10_10_10, 1'b1}; // Forward FFT, conservative scaling
                s_axis_config_tvalid = 1'b1;
                if (s_axis_config_tready) begin
                    next_fft_state = FFT_LOAD;
                    sample_ren = 1'b1; // Start reading samples
                    next_sample_addr = sample_addr + 17'd1;
                    next_sample_fft_addr = '0;
                    next_code_phase = '0;
                end
            end
            if (fft_state == FFT_LOAD) begin
                sample_ren = 1'b1;
                next_code_phase = code_phase + CODE_RATE;
                s_axis_data_tdata = {sample_downsample_q ? 16'h7FFF : 16'h8001, sample_downsample_i ? 16'h7FFF : 16'h8001};
                next_sample_addr = (sample_addr + 17'd1) % 76800;
                next_sample_i_avg = sample_i_avg + (sample_i_out ? 6'd1 : -6'd1);
                next_sample_q_avg = sample_q_avg + (sample_q_out ? 6'd1 : -6'd1);
                
                if (next_code_phase[32]) begin
                    // Finish this averaging period
                    s_axis_data_tvalid = 1'b1;

                    // Hold til ready
                    next_sample_i_avg = sample_i_avg;
                    next_sample_q_avg = sample_q_avg;
                    next_sample_addr = sample_addr;
                
                    if (s_axis_data_tready) begin
                        next_sample_i_avg = '0;
                        next_sample_q_avg = '0;
                        next_sample_addr = (sample_addr + 17'd1) % 76800;
                        next_sample_fft_addr = sample_fft_addr + 12'd1;

                        if (sample_fft_addr == 12'd4095) begin
                            next_sample_addr = '0;
                            s_axis_data_tlast = 1'b1;
                            next_fft_state = FFT_WAIT;
                        end
                    end else begin
                        next_code_phase = code_phase;
                    end
                end
            end
            if (fft_state == FFT_WAIT) begin
                m_axis_data_tready = 1'b1;
                if (m_axis_data_tvalid) begin
                    sample_fft_wen = 1'b1;
                    sample_fft_addr_override = m_axis_data_tuser[11:0];
                    if (m_axis_data_tlast) begin
                        // Initial indices for sample and code FFTs
                        next_sample_fft_addr = '0;
                        next_code_fft_addr = 12'd0 - {{6{next_doppler_step[5]}}, next_doppler_step};
                        next_fft_state = FFT_CONF;
                    end
                end
            end
        end
        PRODUCT_IFFT: begin
            if (fft_state == FFT_CONF) begin
                s_axis_config_tdata = {2'd0, 12'b10_10_10_10_10_10, 1'b0}; // Inverse FFT, conservative scaling
                s_axis_config_tvalid = 1'b1;
                if (s_axis_config_tready) begin
                    next_fft_state = FFT_LOAD;
                    sample_fft_ren = 1'b1; // Start reading sample FFT
                    code_fft_ren = 1'b1; // Start reading code FFT
                    next_sample_fft_addr = sample_fft_addr + 12'd1;
                    next_code_fft_addr = code_fft_addr + 12'd1;
                end
            end
            if (fft_state == FFT_LOAD) begin
                sample_fft_ren = 1'b1;
                code_fft_ren = 1'b1;

                s_axis_data_tdata = {sq_ci_prod[23:8] - si_cq_prod[23:8], si_ci_prod[23:8] + sq_cq_prod[23:8]};
                s_axis_data_tvalid = 1'b1;
                if (s_axis_data_tready) begin
                    next_sample_fft_addr = sample_fft_addr + 12'd1;
                    next_code_fft_addr = code_fft_addr + 12'd1;
                    if (sample_fft_addr == '0) begin
                        next_sample_fft_addr = '0;
                        next_code_fft_addr = '0;
                        s_axis_data_tlast = 1'b1;
                        next_fft_state = FFT_WAIT;
                    end
                end
            end
            if (fft_state == FFT_WAIT) begin
                m_axis_data_tready = 1'b1;
                if (m_axis_data_tvalid) begin
                    // Look through the results
                    if (m_axis_data_tuser[11:0] < 1023) begin
                        // Check for maximum
                        if (acc_magnitude > acc_out) begin
                            next_acc_out = acc_magnitude;
                            next_code_index = m_axis_data_tuser[11:0];
                            next_start_index = start_step;
                            next_dop_index = doppler_step;
                        end
                    end

                    if (m_axis_data_tlast) begin
                        // Initial indices for sample and code FFTs
                        next_doppler_step = doppler_step + 6'd1;
                        next_sample_fft_addr = '0;
                        next_code_fft_addr = 12'd0 - {{6{next_doppler_step[5]}}, next_doppler_step};
                        next_fft_state = FFT_CONF;

                        if (doppler_step == 6'd20) begin
                            // We have finished all Doppler steps, now we need to increment the start step
                            next_doppler_step = -6'd20;
                            next_start_step = start_step + 5'd2;    // 0->18 in steps of 2 equals 10 points within code chip (plenty)
                            next_sample_addr = {12'd0, next_start_step};

                            if (start_step == 5'd18) begin
                                next_start_out = 1'b1; // Signal channel to start
                            end
                        end
                    end
                end
            end
        end
    endcase
end

endmodule
