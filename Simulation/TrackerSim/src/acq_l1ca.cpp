#include "acq_l1ca.h"

#include "fftw3.h"
#include <stdio.h>
#include <string.h>
#include "tools.h"

#define FS 69.984e6
#define FC 9.334875e6
#define CHIP_RATE 1.023e6
#define FREQ 1.57542e9

// Perform a fast fourier transform on the CA code
fftw_complex *transform_code(int tap_idx, int len)
{
    // Allocate space for the code sequence
    fftw_complex *code = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * len);

    // NCO and code generator
    double code_phase = 0.0;
    double code_rate = CHIP_RATE / FS;
    CACodeGenerator ca_code(l1_taps[tap_idx][0], l1_taps[tap_idx][1]);

    // Generate the code
    for (int i = 0; i < len; i++)
    {
        code[i][0] = ca_code.get_chip() ? 1.0 : -1.0;
        code[i][1] = 0.0;

        code_phase += code_rate;
        if (code_phase >= 1)
        {
            ca_code.clock_chip();
            code_phase -= 1.0;
        }
    }

    // Perform the FFT
    fftw_plan plan = fftw_plan_dft_1d(len, code, code, FFTW_FORWARD, FFTW_ESTIMATE);
    fftw_execute(plan);
    fftw_destroy_plan(plan);

    return code;
}

// Perform a fast fourier transform on the signal data
fftw_complex *transform_signal(uint8_t *signal_in, int len)
{
    // Allocate space for the code sequence
    fftw_complex *signal = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * len);

    // NCO for carrier wipeoff
    double carrier_phase = 0.0;
    double carrier_rate = FC * 4.0 / FS;

    // 1-bit sin/cos LUTs
    const uint8_t carrier_sin[] = {1, 1, 0, 0};
    const uint8_t carrier_cos[] = {1, 0, 0, 1};

    // Prepare the signal
    for (int i = 0; i < len; i++)
    {
        signal[i][0] = (signal_in[i] ^ carrier_sin[int(carrier_phase)]) ? -1.0 : 1.0;
        signal[i][1] = (signal_in[i] ^ carrier_cos[int(carrier_phase)]) ? -1.0 : 1.0;

        carrier_phase += carrier_rate;
        if (carrier_phase >= 4)
        {
            carrier_phase -= 4.0;
        }
    }

    // Perform the FFT
    fftw_plan plan = fftw_plan_dft_1d(len, signal, signal, FFTW_FORWARD, FFTW_ESTIMATE);
    fftw_execute(plan);
    fftw_destroy_plan(plan);

    return signal;
}

// Search for the maximum correlation
void correlate(fftw_complex *code, fftw_complex *signal, int len, double doppler_range, double *code_phase, double *doppler, double *snr)
{
    // Now that we have a frequency domain representation of the signal
    // we can easily find the correct code phase and doppler. The doppler
    // shift is performed by a simple translation of the FFT and the code
    // phase will be revealed by the point of maximum power in the time
    // domain after inversely transforming the signal.

    // First create a buffer for the output data and a
    // plan for the inverse transform
    fftw_complex *correlation = (fftw_complex *)fftw_malloc(sizeof(fftw_complex) * len);
    fftw_plan plan = fftw_plan_dft_1d(len, correlation, correlation, FFTW_BACKWARD, FFTW_ESTIMATE);

    int max_snr_idx = 0;
    int max_snr_dop = 0;
    double max_snr = 0.0;

    // Search for doppler shifts from -doppler_range to +doppler_range
    // Each bin is len/FS Hz wide
    for (int dop_shift = int(-1.0 * doppler_range * len / FS); dop_shift <= int(doppler_range * len / FS); dop_shift++)
    {
        int max_corr_idx = 0;
        double max_corr = 0.0;
        double total_corr = 0.0;

        for (int i = 0; i < len; i++)
        {
            // Create index accounting for roll-over
            int idx = (i - dop_shift + len) % len;
            correlation[i][0] = code[idx][0] * signal[i][0] + code[idx][1] * signal[i][1];
            correlation[i][1] = code[idx][1] * signal[i][0] - code[idx][0] * signal[i][1];
        }

        // Perform the inverse FFT
        fftw_execute(plan);

        // Look through the result for the maximum power point (only 1ms)
        int i;
        for (i = 0; i < FS / 1000; i++)
        {
            double power = correlation[i][0] * correlation[i][0] + correlation[i][1] * correlation[i][1];
            if (power > max_corr)
            {
                max_corr = power;
                max_corr_idx = i;
            }
            total_corr += power;
        }

        // Calculate the SNR
        double snr = max_corr / (total_corr / i);
        if (snr > max_snr)
        {
            max_snr = snr;
            max_snr_idx = max_corr_idx;
            max_snr_dop = dop_shift;
        }
    }

    // Return the results
    *code_phase = (max_snr_idx * 1000.0 / FS) * 1023.0;
    *doppler = (double)max_snr_dop * FS / len;
    *snr = max_snr;

    // Clean up
    fftw_destroy_plan(plan);
    fftw_free(correlation);
}

int acquire_l1ca(int sv, uint8_t *signal_in, int len_ms, int start_ms)
{
    if (sv < 1 || sv > 32)
    {
        printf("Invalid SV number\n");
        return 1;
    }

    if (signal_in == nullptr)
    {
        printf("Invalid signal\n");
        return 1;
    }

    // Generate FFTs for the code and signal
    int len = int(len_ms * FS / 1000);
    long long start = (long long)((long long)start_ms * FS / 1000);
    fftw_complex *code = transform_code(sv - 1, len);
    fftw_complex *signal = transform_signal(signal_in + start, len);

    double code_phase = 0.0;
    double doppler = 0.0;
    double snr = 0.0;

    // Perform the correlation
    correlate(code, signal, len, 5000.0, &code_phase, &doppler, &snr);

    // Print results
    printf("PRN %3d, Code phase: %8.1f, Doppler: %8.1f, SNR: %8.1f ", sv, code_phase, doppler, snr);
    for (int i = 0; i < (int)snr / 10; i++)
    {
        printf("*");
    }
    printf("\n");

    // Clean up
    fftw_free(code);
    fftw_free(signal);

    return 0;
}