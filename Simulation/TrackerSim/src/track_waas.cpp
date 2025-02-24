#include "track_waas.h"
#include "math.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define CHIP_RATE 1.023e6
#define CODE_LENGTH 1023
#define FREQ_L1CA 1.57542e9
#define PI 3.14159265358979323846
#define BIT_SYNC_THRESHOLD 35.0
#define BIT_SYNC_MS 1000

// 1-bit carrier LUTs
const uint8_t carrier_sin[] = {1, 1, 0, 0};
const uint8_t carrier_cos[] = {1, 0, 0, 1};

uint8_t check_parity(uint8_t *bits, uint8_t *p, uint8_t D29, uint8_t D30);

SBASWAASTracker::SBASWAASTracker(int sv, double fs, double fc, double doppler, double code_off)
{
    this->sv = sv;
    this->fs = fs;
    this->fc = fc;
    this->doppler = doppler;
    this->code_off = code_off;

    // NCOs
    double code_fractional_off = code_off - floor(code_off);
    code_phase = 0 + code_fractional_off + 0.5;
    code_rate = (CHIP_RATE + (doppler * CHIP_RATE / FREQ_L1CA)) / fs;
    carrier_phase = 0;
    carrier_rate = (fc + doppler) * 4 / fs;

    // Code generator (generates the early first)
    start_chip = (int)code_off;
    if (code_phase >= 1.0)
    {
        start_chip++;
        code_phase -= 1.0;
    }

    // Find the g2_delay for the WAAS code
    int g2_delay = 0;
    int i;
    for (i = 0; i < sizeof(waas_code_params); i++)
    {
        if (waas_code_params[i][0] == sv)
        {
            g2_delay = waas_code_params[i][1];
            break;
        }
    }

    if (i >= sizeof(waas_code_params))
    {
        fprintf(stderr, "WAAS PRN %d not found in waas_code_params\n", sv);
        exit(1);
    }

    code_gen = new WAASCodeGenerator(g2_delay, start_chip);

    // Code generator outputs
    // Figure out the start chip and increment it if we overflowed
    // the code_phase when we defined it.
    code_early = 0;
    code_prompt = 0;
    code_late = 0;

    // Accumulators
    ie = 0;
    qe = 0;
    ip = 0;
    qp = 0;
    il = 0;
    ql = 0;

    // Variable to detect if this epoch has been processed
    epoch_processed = false;

    // DLL filter
    dll = new SecondOrderPLL(5.0, doppler * CHIP_RATE / FREQ_L1CA);

    // PLL filter
    pll = new ThirdOrderPLL(50.0, doppler);

    // Time
    ms_elapsed = 0;

    // Prompt buffers
    prompt_len = 0;
    prompt_idx = 0;

    // Bit sync
    last_ip = 0;
    bit_sync_count = 0;
    memset(bit_hist, 0, sizeof(bit_hist));
    bit_synced = false;
    bit_ms = 0;
    bit_sum = 0;

    // Bit buffer
    nav_count = 0;

    // Nav
    nav_valid = false;
    last_z_count = -2; // So we don't think we incremented on the first one

    // SNR
    cn0 = 0;
}

SBASWAASTracker::~SBASWAASTracker()
{
    delete code_gen;
    delete dll;
    delete pll;
}

// Update the tracker with a new sample
void SBASWAASTracker::update_sample(uint8_t signal_sample)
{
    // Get the local oscillator signals
    uint8_t lo_i = carrier_sin[int(carrier_phase)];
    uint8_t lo_q = carrier_cos[int(carrier_phase)];

    // Update the carrier NCO
    carrier_phase += carrier_rate;
    if (carrier_phase >= 4)
    {
        carrier_phase -= 4;
    }

    // Get the code chip

    // Early code chip (first to change)
    // Late code chip (clocked at the same time, but 1 chip behind)
    if (code_phase >= 1)
    {
        // Update late code from last prompt chip
        code_late = code_prompt;

        // Get new early code chip
        code_gen->clock_chip();
        code_early = code_gen->get_chip();

        code_phase -= 1.0;
    }

    // Prompt code chip (half chip after early chip)
    if (code_phase >= 0.5)
    {
        code_prompt = code_early;
    }

    // Update the code NCO
    code_phase += code_rate;

    // Update the accumulators
    ie += (signal_sample ^ lo_i ^ code_early) ? 1 : -1;
    qe += (signal_sample ^ lo_q ^ code_early) ? 1 : -1;
    ip += (signal_sample ^ lo_i ^ code_prompt) ? 1 : -1;
    qp += (signal_sample ^ lo_q ^ code_prompt) ? 1 : -1;
    il += (signal_sample ^ lo_i ^ code_late) ? 1 : -1;
    ql += (signal_sample ^ lo_q ^ code_late) ? 1 : -1;
}

// Update the tracker with a new epoch
void SBASWAASTracker::update_epoch()
{
    // SNR
    ip_buffer[prompt_idx] = ip;
    qp_buffer[prompt_idx] = qp;
    prompt_len += (prompt_len >= 100) ? 0 : 1;
    prompt_idx = (prompt_idx + 1) % 100;

    cn0 = cn0_svn_estimator(ip_buffer, qp_buffer, 100, 0.001);

    // Compute the Costas loop discriminator
    double carrier_discriminator = 0;
    if (ip != 0)
    {
        carrier_discriminator = atan((double)qp / ip) / (2.0 * PI);
    }

    // Filter the carrier discriminator
    double carrier_error = pll->update(carrier_discriminator, 0.001); // Hz

    // Update the carrier NCO
    carrier_rate = (fc + carrier_error) * 4 / fs;

    // Compute the normalized early-minus late power discriminator
    double power_early = sqrt(ie * ie + qe * qe);
    double power_late = sqrt(il * il + ql * ql);
    double code_discriminator = 0.5 * ((power_early - power_late) / (power_early + power_late));

    // Filter the code discriminator
    double code_error = dll->update(code_discriminator, 0.001); // chips/s

    // Update the code NCO
    code_rate = (CHIP_RATE + code_error) / fs;

    // Bit sync and bit recovery
    if (bit_synced)
    {
        // Recover the last bit
        if (bit_ms == 0)
        {
            nav_buf[nav_count] = (bit_sum > 0) ? 1 : 0;
            nav_count++;
            bit_sum = 0;
        }
        bit_sum += ip;
    }
    // Start bit sync when above threshold
    else if (cn0 > BIT_SYNC_THRESHOLD || bit_sync_count != 0)
    {
        if (bit_sync_count >= BIT_SYNC_MS)
        {
            // Find maximum likelihood edge position
            int bit_off = 0;
            int bit_max = 0;
            for (int i = 0; i < 2; i++)
            {
                if (bit_hist[i] > bit_max)
                {
                    bit_max = bit_hist[i];
                    bit_off = i;
                }
            }

            // Synchronize
            bit_synced = true;
            bit_ms -= bit_off + 1;
            if (bit_off < 0)
                bit_ms += 2;
        }
        // Update the bit sync histogram
        bit_hist[bit_ms % 2] += ((ip > 0) != (last_ip > 0)) ? 1 : 0;
        bit_sync_count++;
    }

    // Increment bit counter
    bit_ms = (bit_ms + 1) % 2;

    // Process the bits
    if (nav_count >= sizeof(nav_buf))
    {
        update_nav();
    }

    // Reset the accumulators
    last_ip = ip;
    ie = 0;
    qe = 0;
    ip = 0;
    qp = 0;
    il = 0;
    ql = 0;

    // Set the flag to indicate that this epoch has been processed
    epoch_processed = true;
    ms_elapsed++;
}

void SBASWAASTracker::update_nav()
{
    uint8_t decoded_bits[250];
    int result = viterbi_decode(nav_buf, decoded_bits, sizeof(nav_buf));
    memmove(nav_buf, nav_buf + 1, 499);
    nav_count--;

    // Preamble
    uint8_t pre1[8] = {0, 1, 0, 1, 0, 0, 1, 1};
    uint8_t pre2[8] = {1, 0, 0, 1, 1, 0, 1, 0};
    uint8_t pre3[8] = {1, 1, 0, 0, 0, 1, 1, 0};

    if ((memcmp(decoded_bits, pre1, 8) == 0) || (memcmp(decoded_bits, pre2, 8) == 0) || (memcmp(decoded_bits, pre3, 8) == 0))
    {
        for (size_t i = 0; i < 8; i++)
        {
            printf("%d", decoded_bits[i]);
        }

        printf(", %d\n", result);
    }
}

void SBASWAASTracker::track(uint8_t *signal, long long size)
{

    // Per sample loop
    for (int i = 0; i < size; i++)
    {
        update_sample(signal[i]);

        // After accumulating and a new code epoch starts, we can process
        // the accumulated values to update the tracking lock and bit recovery
        if (code_gen->chip == 0)
        {
            if (!epoch_processed)
            {
                // Update the epoch
                update_epoch();
            }
        }
        else
        {
            // This resets the flag on chips other than 0
            epoch_processed = false;
        }
    }
}