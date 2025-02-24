#include "tools.h"

#include <string.h>
#include <math.h>
#include <stdlib.h>

CACodeGenerator::CACodeGenerator(uint8_t tap1, uint8_t tap2, int chip_start)
{
    this->tap1 = tap1;
    this->tap2 = tap2;
    this->chip = 0;

    // Initial conditions
    for (int i = 0; i < 11; i++)
    {
        g1[i] = 1;
        g2[i] = 1;
    }

    // Wind to the initial chip
    for (int i = 0; i < chip_start; i++)
    {
        // Update chip
        clock_chip();
    }
}

void CACodeGenerator::clock_chip()
{
    // Update G1
    g1[0] = g1[3] ^ g1[10];

    // Update G2
    g2[0] = g2[2] ^ g2[3] ^ g2[6] ^ g2[8] ^ g2[9] ^ g2[10];

    // Shift registers
    memmove(g1 + 1, g1, 10);
    memmove(g2 + 1, g2, 10);

    // Update chip
    chip = (chip + 1) % 1023;
}

uint8_t CACodeGenerator::get_chip()
{
    return g1[10] ^ g2[tap1] ^ g2[tap2];
}

WAASCodeGenerator::WAASCodeGenerator(int g2_delay, int chip_start)
{
    this->chip = 0;

    // Initial conditions
    g1[0] = 1;
    g2[0] = 1;
    for (int i = 0; i < 10; i++)
    {
        g1[i + 1] = 1;
        g2[i + 1] = (g2_delay >> i) & 1;
    }

    // Wind to the initial chip
    for (int i = 0; i < chip_start; i++)
    {
        // Update chip
        clock_chip();
    }
}

void WAASCodeGenerator::clock_chip()
{
    // Update G1
    g1[0] = g1[3] ^ g1[10];

    // Update G2
    g2[0] = g2[2] ^ g2[3] ^ g2[6] ^ g2[8] ^ g2[9] ^ g2[10];

    // Shift registers
    memmove(g1 + 1, g1, 10);
    memmove(g2 + 1, g2, 10);

    // Update chip
    chip = (chip + 1) % 1023;
}

uint8_t WAASCodeGenerator::get_chip()
{
    return g1[10] ^ g2[10];
}

GalileoE1CodeGenerator::GalileoE1CodeGenerator(int code_idx, int chip_start)
{
    this->code_idx = code_idx;
    this->chip = chip_start;
}

void GalileoE1CodeGenerator::clock_chip()
{
    // Update chip
    chip = (chip + 1) % 4092;
}

uint8_t GalileoE1CodeGenerator::get_chip()
{
    return (gal_e1c_code[code_idx][chip / 8] >> (7 - (chip % 8))) & 0x1;
}

uint8_t GalileoE1CodeGenerator::get_data_chip()
{
    return (gal_e1b_code[code_idx][chip / 8] >> (7 - (chip % 8))) & 0x1;
}

double cn0_svn_estimator(const double *ip, const double *qp, int length, double coh_integration_time_s)
{
    double SNR = 0.0;
    double SNR_dB_Hz = 0.0;
    double Psig = 0.0;
    double Ptot = 0.0;
    for (int i = 0; i < length; i++)
    {
        Psig += fabs(ip[i]);
        Ptot += qp[i] * qp[i] + ip[i] * ip[i];
    }
    Psig /= static_cast<double>(length);
    Psig = Psig * Psig;
    Ptot /= static_cast<double>(length);
    SNR = Psig / (Ptot - Psig);
    SNR_dB_Hz = 10.0 * log10(SNR) - 10.0 * log10(coh_integration_time_s);
    return SNR_dB_Hz;
}

void bytes_to_number(void *dest, uint8_t *buf, int dest_size, int src_size, uint8_t sign)
{
    if ((dest_size < src_size) || (dest_size % 8 != 0))
    {
        return;
    }
    uint64_t result = 0;

    // Pack bytes
    for (int i = 0; i < src_size; i++)
    {
        result |= (buf[src_size - i - 1] << i);
    }

    // Preserve sign if signed
    if (sign)
    {
        uint8_t sign_bit = (result >> (src_size - 1)) & 0x1;
        result |= sign_bit ? (0xFFFFFFFFFFFFFFFF << src_size) : 0;
    }

    // Copy result to dest
    memcpy(dest, &result, dest_size / 8);
}

uint32_t gal_e1_crc(const uint8_t *data, const uint8_t *extras)
{
    // Polynomial for the 24-bit CRC (P(X) = X^23 + X^17 + X^13 + X^12 + X^11 + X^9 + X^8 + X^7 + X^5 + X^3 + 1)
    const uint32_t POLYNOMIAL = 0x864CFB; // Binary: 1000 0010 0011 1011 1010 1001

    // CRC register initialized to 0
    uint32_t crc = 0;

    int data_index = 0;
    int extras_index = 0;

    for (int i = 0; i < 196; ++i)
    {
        // Bring in the next byte to the CRC calculation
        uint32_t byte;
        if (i < 2)
        {
            // Even bit, nominal page
            byte = 0;
        }
        else if (i < 114)
        {
            // Even page data
            byte = data[data_index++];
        }
        else if (i < 115)
        {
            // Odd bit
            byte = 1;
        }
        else if (i < 116)
        {
            // Nominal page
            byte = 0;
        }
        else if (i < 132)
        {
            // Odd page data
            byte = data[data_index++];
        }
        else
        {
            // Extras
            byte = extras[extras_index++];
        }

        crc ^= ((uint32_t)byte << 23);

        // Process the bit
        if (crc & 0x800000)
        {
            crc = (crc << 1) ^ POLYNOMIAL;
        }
        else
        {
            crc <<= 1;
        }
        crc &= 0xFFFFFF; // Ensure the CRC remains 24 bits
    }

    return crc;
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

int viterbi_decode(uint8_t *data, uint8_t *result, int ninput)
{
    static const uint8_t nstates = 64;
    static const uint8_t rate = 2;

    // Allocate path metric array
    uint32_t *path_metric[nstates];
    for (int i = 0; i < nstates; i++)
    {
        path_metric[i] = (uint32_t *)malloc((ninput / rate + 1) * sizeof(uint32_t));
    }

    // Setup initial state
    for (int i = 0; i < nstates; i++)
    {
        memset(path_metric[i], 0xFF, sizeof((ninput / rate + 1) * sizeof(uint32_t)));
    }
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
    uint32_t best_ending = 0xFFFFFFFF;
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

    // Save best ending
    best_ending = best_state;

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

    // Cleanup
    for (int i = 0; i < nstates; i++)
    {
        free(path_metric[i]);
    }

    return best_ending;
}