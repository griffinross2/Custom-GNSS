#include "tools.h"

#include <string.h>
#include <math.h>

CACodeGenerator::CACodeGenerator(uint8_t tap1, uint8_t tap2, int chip)
{
    this->tap1 = tap1;
    this->tap2 = tap2;
    this->chip = chip;

    // Initial conditions
    for (int i = 0; i < 11; i++)
    {
        g1[i] = 1;
        g2[i] = 1;
    }

    // Wind to the initial chip
    for (int i = 0; i < chip; i++)
    {
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

GalileoE1CodeGenerator::GalileoE1CodeGenerator(int code_idx, int chip)
{
    this->code_idx = code_idx;
    this->chip = chip;
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