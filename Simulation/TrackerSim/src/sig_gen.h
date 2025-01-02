#ifndef SIG_GEN_H
#define SIG_GEN_H

#include <stdint.h>
#include <random>

class GalileoSigGen
{
public:
    GalileoSigGen(
        double fs = 69.984e6,
        double fc = 9.334875e6,
        double noise_power_db = 26.25,
        double doppler_start = 0,
        double doppler_end = 0);

    void generate(uint8_t *signal, long long size);

private:
    double fs;
    double fc;
    double noise_power_db;
    double doppler_start;
    double doppler_end;
    std::normal_distribution<double> noise;
};

#endif // SIG_GEN_H
