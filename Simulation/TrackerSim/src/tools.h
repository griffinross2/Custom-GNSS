#ifndef TOOLS_H
#define TOOLS_H

#include <stdint.h>

class CACodeGenerator
{
public:
    CACodeGenerator(uint8_t tap1, uint8_t tap2);

    void clock_chip();
    uint8_t get_chip();

private:
    int chip;
    uint8_t tap1;
    uint8_t tap2;
    uint8_t g1[11]; // First bit is space for new bit
    uint8_t g2[11]; // First bit is space for new bit
};

#endif // TOOLS_H
