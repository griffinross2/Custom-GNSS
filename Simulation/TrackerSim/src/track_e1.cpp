#include "track_e1.h"
#include "math.h"

#include <stdio.h>
#include <string.h>

#define CHIP_RATE 1.023e6
#define CODE_LENGTH 4092
#define FREQ_E1 1.57542e9

// 1-bit carrier LUTs
const uint8_t carrier_sin[] = {1, 1, 0, 0};
const uint8_t carrier_cos[] = {1, 0, 0, 1};

uint8_t parity(uint8_t val);
uint8_t get_conv_out(uint8_t state, uint8_t input);
void viterbi_decode(uint8_t *data, uint8_t *result);

GalileoE1Tracker::GalileoE1Tracker(int sv, double fs, double fc, double doppler, double code_off, double dll_bw, double pll_bw, double fll_bw)
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
    code_gen = new GalileoE1CodeGenerator(sv - 1, start_chip);

    // Code generator outputs
    // Figure out the start chip and increment it if we overflowed
    // the code_phase when we defined it.
    code_very_early = 0;
    code_early = 0;
    code_prompt = 0;
    code_late = 0;
    code_very_late = 0;

    code_prompt_data = 0;

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

    // Data accumulators
    ip_data = 0;
    qp_data = 0;

    // Variable to detect if this epoch has been processed
    epoch_processed = false;

    // DLL filter
    dll = new SecondOrderPLL(dll_bw, doppler * CHIP_RATE / FREQ_E1);

    // PLL filter
    pll = new ThirdOrderFLLAssistedPLL(fll_bw, pll_bw, doppler);

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

    // Nav data
    nav_count = 0;
    last_page = -1;
    last_page_half = 0;

    // SNR
    cn0 = 0;
}

GalileoE1Tracker::~GalileoE1Tracker()
{
    delete code_gen;
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
        code_prompt_data = code_gen->get_data_chip();

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
        code_gen->clock_chip();
        code_very_early = code_gen->get_chip();

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

    ip_data += (signal_sample ^ lo_i ^ code_prompt_data ^ boc1) ? 1 : -1;
    qp_data += (signal_sample ^ lo_q ^ code_prompt_data ^ boc1) ? 1 : -1;
}

// Update the tracker with a new epoch
void GalileoE1Tracker::update_epoch()
{
    // SNR
    ip_buffer[prompt_idx] = ip;
    qp_buffer[prompt_idx] = qp;
    prompt_len += (prompt_len >= PROMPT_LEN) ? 0 : 1;
    prompt_idx = (prompt_idx + 1) % PROMPT_LEN;

    cn0 = cn0_svn_estimator(ip_buffer, qp_buffer, prompt_len, (double)CODE_LENGTH / CHIP_RATE);

    // Promotion to fine tracking
    if (cn0 <= 35.0)
    {
        half_el_spacing = 0.25;
        discriminator_factor = 1.0;
    }
    else if (cn0 >= 35.0)
    {
        half_el_spacing = 1.0 / 12.0;
        discriminator_factor = 1.0;
    }

    // Reduce pll bandwidth when achieving lock
    // Helps keep lock for weak signals
    if (cn0 >= 27.0)
    {
        pll->set_bandwidth(25.0, 25.0);
    }
    else
    {
        pll->set_bandwidth(35.0, 35.0);
    }

    // Pilot tracking state
    if (cn0 >= 35.0)
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

    // Compute the frequency error
    double carrier_discriminator_fll = 0;
    if (cn0 < 30.0 && ip != 0 && ip_buffer[(prompt_idx - 2 + PROMPT_LEN) % PROMPT_LEN] != 0)
    {
        carrier_discriminator_fll = (atan((double)qp / ip) - atan((double)qp_buffer[(prompt_idx - 2 + PROMPT_LEN) % PROMPT_LEN] / ip_buffer[(prompt_idx - 2 + PROMPT_LEN) % PROMPT_LEN]));
    }
    carrier_discriminator_fll = PHASE_UNWRAP(carrier_discriminator_fll) / (2.0 * PI * (double)CODE_LENGTH / CHIP_RATE);

    // Filter the carrier discriminator
    double carrier_error = pll->update(carrier_discriminator_fll, carrier_discriminator, (double)CODE_LENGTH / CHIP_RATE); // Hz

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
    if (cn0 > 30.0)
    {
        code_rate += carrier_error * CHIP_RATE / FREQ_E1 / fs; // + 0.002;
    }

    // Bump-jump
    if (vel_p_squared_len >= VEL_LEN)
    {
        if (ve_p_squared_sum > 1.5 * p_p_squared_sum)
        {
            // Too early, increase code phase
            // printf("Early bump jump\n");
            // code_rate += 0.5 * CHIP_RATE / CODE_LENGTH / fs;
            code_phase += 0.5;
            vel_p_squared_len = 0;
        }
        else if (vl_p_squared_sum > 1.5 * p_p_squared_sum)
        {
            // Too late, decrease code phase
            // printf("Late bump jump\n");
            // code_rate -= 0.5 * CHIP_RATE / CODE_LENGTH / fs;
            code_phase -= 0.5;
            vel_p_squared_len = 0;
        }
    }

    // Recover bit
    nav_buf[nav_count] = ip_data > 0 ? 1 : 0;
    nav_count++;

    // printf("%.0f,%.0f,%.0f,%.0f,%.0f,", sqrt(ive * ive + qve * qve), sqrt(ie * ie + qe * qe), sqrt(ip * ip + qp * qp), sqrt(il * il + ql * ql), sqrt(ivl * ivl + qvl * qvl));
    // printf("%.8f,%.8f,", code_error, carrier_error);
    // printf("%.8f,%.8f,", code_rate * fs, carrier_rate * fs / 4);
    // printf("%d,%d,%d,%d,%0.1f\n", ip, qp, ip_data, qp_data, cn0);

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

    ip_data = 0;
    qp_data = 0;

    // Set the flag to indicate that this epoch has been processed
    epoch_processed = true;
    ms_elapsed += 4;

    // Update the pilot secondary chip
    pilot_secondary_chip = (pilot_secondary_chip + 1) % 25;
}

void GalileoE1Tracker::update_nav()
{
    static const uint8_t sync_word[] = {0, 1, 0, 1, 1, 0, 0, 0, 0, 0};
    static const uint8_t inv_sync_word[] = {1, 0, 1, 0, 0, 1, 1, 1, 1, 1};
    static const uint8_t tail[] = {0, 0, 0, 0, 0, 0};

    uint8_t polarity;
    uint8_t *data = nav_buf;

    // Check sync word
    if (memcmp(data, sync_word, sizeof(sync_word)) == 0)
    {
        polarity = 0;
    }
    else if (memcmp(data, inv_sync_word, sizeof(inv_sync_word)) == 0)
    {
        polarity = 1;
    }
    else
    {
        memmove(nav_buf, nav_buf + 1, 249);
        nav_count--;
        return;
    }

    // Deinterleave
    data += sizeof(sync_word);
    uint8_t block[240];
    for (int i = 0; i < 8; i++)
    {
        for (int j = 0; j < 30; j++)
        {
            block[i + j * 8] = data[i * 30 + j] ^ polarity;
        }
    }

    // Decode convolutional code
    uint8_t decoded_bits[sizeof(block) / 2];
    viterbi_decode(block, decoded_bits);

    // Check tail
    if (memcmp(decoded_bits + (sizeof(decoded_bits) - sizeof(tail)), tail,
               sizeof(tail)))
    {
        memmove(nav_buf, nav_buf + 1, 249);
        nav_count--;
        return;
    }

    // Increment tGST
    ephm.inc_time();

    // Get page
    last_page_half = decoded_bits[0];
    if (last_page_half == 0)
    {
        // First half
        last_page = 0;
        memcpy(page_data, decoded_bits + 2, 112); // Copy data
        for (int i = 0; i < 6; i++)
        {
            last_page |= decoded_bits[i + 2] << (5 - i);
        }
    }
    else
    {
        // Second half
        if (last_page >= 0)
        {
            memcpy(page_data + 112, decoded_bits + 2, 16); // Copy data
            memcpy(page_crc, decoded_bits + 82, 24);       // Copy CRC
            memcpy(page_extras, decoded_bits + 18, 64);    // Copy extras

            // Check CRC
            uint32_t crc = gal_e1_crc(page_data, page_extras);
            bool crc_ok = true;

            for (int i = 0; i < 24; i++)
            {
                if (page_crc[i] != ((crc >> (23 - i)) & 0x1))
                {
                    // CRC failure
                    crc_ok = false;
                }
            }

            // Save navigation information
            if (crc_ok)
            {
                ephm.process_message(page_data, last_page);
            }
        }
    }

    // Message found
    printf("Galileo E1 PRN %d found %s half of page %d at %lld ms\n", sv, last_page_half ? "2nd" : "1st", last_page, ms_elapsed);
    nav_count = 0;
}

void GalileoE1Tracker::track(uint8_t *signal, long long size)
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
                if (nav_count >= 250)
                {
                    update_nav();
                }
            }
        }
        else
        {
            // This resets the flag on chips other than 0
            epoch_processed = false;
        }
    }
}

double GalileoE1Tracker::get_tx_time()
{
    uint32_t chips = code_gen->chip;

    double t = (double)(ephm.get_time() % 604800) +
               ((double)nav_count / 250.0) +
               ((double)chips / 1023000.0) +
               (code_phase / 1023000.0);

    return t;
}

void GalileoE1Tracker::get_satellite_ecef(double t, double *x, double *y, double *z)
{
    ephm.get_satellite_ecef(t, x, y, z);
}

double GalileoE1Tracker::get_clock_correction(double t)
{
    return ephm.get_clock_correction(t);
}

bool GalileoE1Tracker::ready_to_solve()
{
    return ephm.ephm_valid();
}

uint32_t hamming_dist2(uint32_t val1, uint32_t val2)
{
    uint32_t dist = 0;
    for (int i = 0; i < 2; i++)
    {
        dist += (val1 & 0x1) ^ (val2 & 0x1);
        val1 >>= 1;
        val2 >>= 1;
    }
    return dist;
}

uint8_t parity(uint8_t val)
{
    uint8_t parity = 0;
    for (int i = 0; i < 8; i++)
    {
        parity ^= (val >> i) & 0x1;
    }
    return parity;
}

uint8_t get_conv_out(uint8_t state, uint8_t input)
{
    static uint8_t G1 = 0b1111001;
    static uint8_t G2 = 0b1011011;

    uint8_t conv_in = state | (input << 6);

    // [7:2] res, [1:0] output bits
    uint8_t result = 0;
    result |= (parity(conv_in & G1) << 1);
    result |= (parity(conv_in & G2) == 0);

    return result;
}

void viterbi_decode(uint8_t *data, uint8_t *result)
{
    static const uint8_t nstates = 64;
    static const uint8_t ninput = 240;
    static const uint8_t rate = 2;

    uint32_t path_metric[nstates][ninput / rate + 1];

    // Setup initial state
    memset(path_metric, 0xFF, sizeof(path_metric));
    path_metric[0][0] = 0;

    // Create trellis
    for (int j = 0; j < ninput / rate; j++)
    {
        for (uint8_t i = 0; i < nstates; i++)
        {
            // skip unaccessible states
            if (path_metric[i][j] == 0xFFFFFFFF)
                continue;

            // input bit 0
            uint8_t next_state = (i >> 1);
            uint32_t conv_out = get_conv_out(i, 0);
            uint8_t rcvd = (data[j * rate] << 1) | data[j * rate + 1];
            uint32_t dist = hamming_dist2(rcvd, conv_out);
            if (path_metric[next_state][j + 1] > path_metric[i][j] + dist)
            {
                path_metric[next_state][j + 1] = path_metric[i][j] + dist;
            }
            // input bit 1
            next_state |= 0b100000;
            conv_out = get_conv_out(i, 1);
            dist = hamming_dist2(rcvd, conv_out);
            if (path_metric[next_state][j + 1] > path_metric[i][j] + dist)
            {
                path_metric[next_state][j + 1] = path_metric[i][j] + dist;
            }
        }
    }

    // Find best ending
    uint8_t best_state = 0;
    uint32_t best_metric = 0xFFFFFFFF;
    for (int i = 0; i < nstates; i++)
    {
        if (path_metric[i][ninput / rate] < best_metric)
        {
            best_metric = path_metric[i][ninput / rate];
            best_state = i;
        }
    }

    // Traceback through best path
    for (int i = ninput / rate - 1; i >= 0; i--)
    {
        result[i] = (best_state >> 5) & 1;

        // Zero path
        uint8_t previous_state = (best_state << 1) & 0b111111;
        best_metric = path_metric[previous_state][i];
        best_state = previous_state;

        // One path
        previous_state = (best_state | 1);
        if (path_metric[previous_state][i] < best_metric)
        {
            best_metric = path_metric[previous_state][i];
            best_state = previous_state;
        }
    }
}