#include "track_e1.h"
#include "math.h"

#include <stdio.h>
#include <string.h>

#define CHIP_RATE 1.023e6
#define CODE_LENGTH 4092
#define FREQ_E1 1.57542e9
#define PI 3.14159265358979323846

// 1-bit carrier LUTs
const uint8_t carrier_sin[] = {1, 1, 0, 0};
const uint8_t carrier_cos[] = {1, 0, 0, 1};

GalileoE1Tracker::GalileoE1Tracker(int sv, double fs, double fc, double doppler, double code_off)
{
    this->sv = sv;
    this->fs = fs;
    this->fc = fc;
    this->doppler = doppler;
    this->code_off = code_off;

    // NCOs
    double code_fractional_off = code_off - floor(code_off);
    code_phase = 0 + code_fractional_off + 0.5;
    code_rate = (CHIP_RATE + (doppler * CHIP_RATE / FREQ_E1)) / fs;
    carrier_phase = 0;
    carrier_rate = (fc + doppler) * 4 / fs;

    // Code generator (generates the early first)
    start_chip = (int)code_off;
    if (code_phase >= 1.0)
    {
        start_chip++;
        code_phase -= 1.0;
    }
    ca_code_very_early = new GalileoE1CodeGenerator(sv - 1, start_chip);

    // Code generator outputs
    // Figure out the start chip and increment it if we overflowed
    // the code_phase when we defined it.
    code_very_early = 0;
    code_early = 0;
    code_prompt = 0;
    code_late = 0;
    code_very_late = 0;

    // BOC generator
    boc1 = 0;

    // Accumulators
    ive = 0;
    qve = 0;
    ie = 0;
    qe = 0;
    ip = 0;
    qp = 0;
    il = 0;
    ql = 0;
    ivl = 0;
    qvl = 0;

    // Variable to detect if this epoch has been processed
    epoch_processed = false;

    // DLL filter
    dll = new SecondOrderPLL(1.0, doppler * CHIP_RATE / FREQ_E1);

    // PLL filter
    pll = new ThirdOrderPLL(18.0, doppler);

    ms_elapsed = 0;

    // Prompt buffers
    memset(ip_buffer, 0, sizeof(ip_buffer));
    memset(qp_buffer, 0, sizeof(qp_buffer));
    prompt_len = 0;
    prompt_idx = 0;

    // VE, VL buffers
    vel_p_squared_len = 0;
    vel_p_squared_idx = 0;
    ve_p_squared_sum = 0;
    p_p_squared_sum = 0;
    vl_p_squared_sum = 0;
    memset(ve_p_squared_buffer, 0, sizeof(ve_p_squared_buffer));
    memset(p_p_squared_buffer, 0, sizeof(p_p_squared_buffer));
    memset(vl_p_squared_buffer, 0, sizeof(vl_p_squared_buffer));

    // Code tracking adjustments
    half_el_spacing = 0.25;
    discriminator_factor = 1.0;

    // Pilot tracking
    pilot_state = E1_PILOT_NO_LOCK;
    memset(pilot_secondary_acc, 0, sizeof(pilot_secondary_acc));
    pilot_secondary_idx = 0;
    pilot_secondary_chip = 0;
    pilot_secondary_pol = 0;
}

GalileoE1Tracker::~GalileoE1Tracker()
{
    delete ca_code_very_early;
    delete dll;
    delete pll;
}

// Update the tracker with a new sample
void GalileoE1Tracker::update_sample(uint8_t signal_sample)
{
    // Update the carrier NCO
    carrier_phase += carrier_rate;
    if (carrier_phase >= 4)
    {
        carrier_phase -= 4;
    }

    // Get the code chip

    // Early code chip (1/4 chip before P chip)
    if (code_phase >= 0.5 - half_el_spacing)
    {
        code_early = code_very_early;
    }

    // Prompt code chip (0.5 chip after VE chip)
    if (code_phase >= 0.5)
    {
        code_prompt = code_early;

        // Falling edge of BOC1
        boc1 = 0;
    }

    // Late code chip (1/4 chip after P chip)
    if (code_phase >= 0.5 + half_el_spacing)
    {
        code_late = code_prompt;
    }

    // Very early code chip (first to change)
    // Late code chip (clocked at the same time, but 1 chip behind)
    if (code_phase >= 1)
    {
        // Update very late code from last late chip
        code_very_late = code_late;

        // Get new early code chip
        ca_code_very_early->clock_chip();
        code_very_early = ca_code_very_early->get_chip();

        code_phase -= 1.0;

        // Rising edge of BOC1
        boc1 = 1;
    }

    // Update the code NCO
    code_phase += code_rate;

    // Get the local oscillator signals
    uint8_t lo_i = carrier_sin[int(carrier_phase)];
    uint8_t lo_q = carrier_cos[int(carrier_phase)];

    // Update the accumulators
    ive += (signal_sample ^ lo_i ^ code_very_early ^ boc1 ^ (pilot_state == E1_PILOT_LOCK_SEC ? e1_secondary[pilot_secondary_chip] ^ pilot_secondary_pol : 0)) ? 1 : -1;
    qve += (signal_sample ^ lo_q ^ code_very_early ^ boc1 ^ (pilot_state == E1_PILOT_LOCK_SEC ? e1_secondary[pilot_secondary_chip] ^ pilot_secondary_pol : 0)) ? 1 : -1;
    ie += (signal_sample ^ lo_i ^ code_early ^ boc1 ^ (pilot_state == E1_PILOT_LOCK_SEC ? e1_secondary[pilot_secondary_chip] ^ pilot_secondary_pol : 0)) ? 1 : -1;
    qe += (signal_sample ^ lo_q ^ code_early ^ boc1 ^ (pilot_state == E1_PILOT_LOCK_SEC ? e1_secondary[pilot_secondary_chip] ^ pilot_secondary_pol : 0)) ? 1 : -1;
    ip += (signal_sample ^ lo_i ^ code_prompt ^ boc1 ^ (pilot_state == E1_PILOT_LOCK_SEC ? e1_secondary[pilot_secondary_chip] ^ pilot_secondary_pol : 0)) ? 1 : -1;
    qp += (signal_sample ^ lo_q ^ code_prompt ^ boc1 ^ (pilot_state == E1_PILOT_LOCK_SEC ? e1_secondary[pilot_secondary_chip] ^ pilot_secondary_pol : 0)) ? 1 : -1;
    il += (signal_sample ^ lo_i ^ code_late ^ boc1 ^ (pilot_state == E1_PILOT_LOCK_SEC ? e1_secondary[pilot_secondary_chip] ^ pilot_secondary_pol : 0)) ? 1 : -1;
    ql += (signal_sample ^ lo_q ^ code_late ^ boc1 ^ (pilot_state == E1_PILOT_LOCK_SEC ? e1_secondary[pilot_secondary_chip] ^ pilot_secondary_pol : 0)) ? 1 : -1;
    ivl += (signal_sample ^ lo_i ^ code_very_late ^ boc1 ^ (pilot_state == E1_PILOT_LOCK_SEC ? e1_secondary[pilot_secondary_chip] ^ pilot_secondary_pol : 0)) ? 1 : -1;
    qvl += (signal_sample ^ lo_q ^ code_very_late ^ boc1 ^ (pilot_state == E1_PILOT_LOCK_SEC ? e1_secondary[pilot_secondary_chip] ^ pilot_secondary_pol : 0)) ? 1 : -1;
}

// Update the tracker with a new epoch
void GalileoE1Tracker::update_epoch()
{
    // SNR
    ip_buffer[prompt_idx] = ip;
    qp_buffer[prompt_idx] = qp;
    prompt_len += (prompt_len >= PROMPT_LEN) ? 0 : 1;
    prompt_idx = (prompt_idx + 1) % PROMPT_LEN;

    double snr = 0;
    if (prompt_len >= PROMPT_LEN)
    {
        snr = cn0_svn_estimator(ip_buffer, qp_buffer, PROMPT_LEN, (double)CODE_LENGTH / CHIP_RATE);
    }

    // Promotion to fine tracking
    if (snr <= 40.0)
    {
        half_el_spacing = 0.25;
        discriminator_factor = 1.0;
    }
    else if (snr >= 40.0)
    {
        half_el_spacing = 1.0 / 12.0;
        discriminator_factor = 0.5;
    }

    // Pilot tracking state
    if (snr >= 40.0)
    {
        if (pilot_state == E1_PILOT_NO_LOCK)
        {
            pilot_state = E1_PILOT_LOCK_NO_SEC;
        }
    }

    if (pilot_state == E1_PILOT_LOCK_NO_SEC)
    {
        if (pilot_secondary_idx == 25)
        {
            if (memcmp(pilot_secondary_acc, e1_secondary, 25) == 0)
            {
                pilot_state = E1_PILOT_LOCK_SEC;
                pilot_secondary_chip = 0;
                pilot_secondary_pol = 1;
            }
            else if (memcmp(pilot_secondary_acc, e1_secondary_inv, 25) == 0)
            {
                pilot_state = E1_PILOT_LOCK_SEC;
                pilot_secondary_chip = 0;
                pilot_secondary_pol = 0;
            }
            else
            {
                memmove(pilot_secondary_acc, pilot_secondary_acc + 1, 24);
                pilot_secondary_acc[24] = ip > 0 ? 1 : 0;
            }
        }
        else
        {
            pilot_secondary_acc[pilot_secondary_idx] = ip > 0 ? 1 : 0;
            pilot_secondary_idx++;
        }
    }

    // VEL bump jump detection
    double ve_p_squared = sqrt(ive * ive + qve * qve);
    double p_p_squared = sqrt(ip * ip + qp * qp);
    double vl_p_squared = sqrt(ivl * ivl + qvl * qvl);

    // Update the sum by adding this power and subtracting the oldest power
    ve_p_squared_sum += ve_p_squared - ve_p_squared_buffer[vel_p_squared_idx];
    p_p_squared_sum += p_p_squared - p_p_squared_buffer[vel_p_squared_idx];
    vl_p_squared_sum += vl_p_squared - vl_p_squared_buffer[vel_p_squared_idx];

    // Update the buffers
    ve_p_squared_buffer[vel_p_squared_idx] = ve_p_squared;
    p_p_squared_buffer[vel_p_squared_idx] = p_p_squared;
    vl_p_squared_buffer[vel_p_squared_idx] = vl_p_squared;

    // Update the length and index
    vel_p_squared_len += (vel_p_squared_len >= VEL_LEN) ? 0 : 1;
    vel_p_squared_idx = (vel_p_squared_idx + 1) % VEL_LEN;

    // Compute the Costas loop discriminator or pure phase discriminator
    double carrier_discriminator = 0;
    if (ip != 0)
    {
        if (pilot_state == E1_PILOT_LOCK_SEC)
        {
            carrier_discriminator = atan2((double)qp, (double)ip) / (2.0 * PI);
        }
        else
        {
            carrier_discriminator = atan((double)qp / ip) / (2.0 * PI);
        }
    }

    // // Compute the frequency error
    // double carrier_discriminator_fll = 0;
    // if (ip != 0 && ip_buffer[(prompt_idx - 2 + PROMPT_LEN) % PROMPT_LEN] != 0)
    // {
    //     carrier_discriminator_fll = (atan((double)qp / ip) - atan((double)qp_buffer[(prompt_idx - 2 + PROMPT_LEN) % PROMPT_LEN] / ip_buffer[(prompt_idx - 2 + PROMPT_LEN) % PROMPT_LEN]));
    // }
    // carrier_discriminator_fll = PHASE_UNWRAP(carrier_discriminator_fll) / (2.0 * PI * (double)CODE_LENGTH / CHIP_RATE);

    // Filter the carrier discriminator
    double carrier_error = pll->update(carrier_discriminator, (double)CODE_LENGTH / CHIP_RATE); // Hz

    // Update the carrier NCO
    carrier_rate = (fc + carrier_error) * 4 / fs;

    // Compute the normalized very-early-minus-late power discriminator
    // double power_early = sqrt(ie * ie + qe * qe /* + ive * ive + qve * qve*/);
    // double power_late = sqrt(il * il + ql * ql /* + ivl * ivl + qvl * qvl*/);
    // double code_discriminator = 0.0;
    // if (power_early + power_late != 0)
    // {
    //     code_discriminator = ((power_early - power_late) / (power_early + power_late)); /* code */
    // }
    double code_discriminator = discriminator_factor * ((ie - il) * ip + (qe - ql) * qp);
    double code_normalization = (ie + il) * ip + (qe + ql) * qp;
    if (code_normalization != 0)
    {
        code_discriminator /= code_normalization;
    }
    else
    {
        code_discriminator = 0;
    }

    // Filter the code discriminator
    double code_error = dll->update(code_discriminator, (double)CODE_LENGTH / CHIP_RATE); // chips/s

    // Update the code NCO
    code_rate = (CHIP_RATE + code_error) / fs;

    // Carrier aiding
    code_rate += carrier_error * CHIP_RATE / FREQ_E1 / fs; // + 0.002;

    if (vel_p_squared_len >= VEL_LEN)
    {
        if (ve_p_squared_sum > 1.5 * p_p_squared_sum)
        {
            // Too early, increase code phase
            printf("Early bump jump\n");
            // code_rate += 0.5 * CHIP_RATE / CODE_LENGTH / fs;
            code_phase += 0.5;
            vel_p_squared_len = 0;
        }
        else if (vl_p_squared_sum > 1.5 * p_p_squared_sum)
        {
            // Too late, decrease code phase
            printf("Late bump jump\n");
            // code_rate -= 0.5 * CHIP_RATE / CODE_LENGTH / fs;
            code_phase -= 0.5;
            vel_p_squared_len = 0;
        }
    }

    printf("%f,%f,%f,%f,%f,", sqrt(ive * ive + qve * qve), sqrt(ie * ie + qe * qe), sqrt(ip * ip + qp * qp), sqrt(il * il + ql * ql), sqrt(ivl * ivl + qvl * qvl));
    printf("%f,%f,", code_error, carrier_error);
    printf("%f,%f,", code_rate * fs, carrier_rate * fs / 4);
    printf("%d,%d,%f\n", ip, qp, snr);

    // Reset the accumulators
    ive = 0;
    qve = 0;
    ie = 0;
    qe = 0;
    ip = 0;
    qp = 0;
    il = 0;
    ql = 0;
    ivl = 0;
    qvl = 0;

    // Set the flag to indicate that this epoch has been processed
    epoch_processed = true;
    ms_elapsed++;

    // Update the pilot secondary chip
    pilot_secondary_chip = (pilot_secondary_chip + 1) % 25;
}

void GalileoE1Tracker::track(uint8_t *signal, long long size)
{

    // Per sample loop
    for (int i = 0; i < size; i++)
    {
        update_sample(signal[i]);

        // After accumulating and a new code epoch starts, we can process
        // the accumulated values to update the tracking lock and bit recovery
        if (ca_code_very_early->chip == 0)
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