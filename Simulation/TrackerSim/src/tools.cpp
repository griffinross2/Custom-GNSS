#include "tools.h"

#include <string.h>

CACodeGenerator::CACodeGenerator(uint8_t tap1, uint8_t tap2)
{
    this->tap1 = tap1;
    this->tap2 = tap2;

    // Initial conditions
    this->chip = 0;
    for (int i = 0; i < 11; i++)
    {
        g1[i] = 1;
        g2[i] = 1;
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