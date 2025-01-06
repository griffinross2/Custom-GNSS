#ifndef SIG_GEN_H
#define SIG_GEN_H

#include <stdint.h>
#include <random>

class SignalFromFile
{
public:
    SignalFromFile();

    bool open(const char *filename);
    void close();
    void generate(double *signal, long long size);

private:
    FILE *file;
};

class NoiseGen
{
public:
    NoiseGen(
        double fs = 69.984e6,
        double fc = 9.334875e6,
        double bandwidth = 18e6); // MAX-2769 wideband setting

    void generate(double *signal, long long size);

private:
    double fs;
    double fc;
    double bandwidth;
    std::normal_distribution<double> noise;
};

class GPSL1CASigGen
{
public:
    GPSL1CASigGen(
        double fs = 69.984e6,
        double fc = 9.334875e6,
        double power_dbm = -128.5, // Minimum signal power from IS-GPS-200M = -158.5dBW = -128.5dbm
        double doppler_start = 0,
        double doppler_end = 0);

    void generate(double *signal, long long size);

private:
    double fs;
    double fc;
    double power_dbm;
    double doppler_start;
    double doppler_end;
};

class GalileoE1SigGen
{
public:
    GalileoE1SigGen(
        double fs = 69.984e6,
        double fc = 9.334875e6,
        double power_dbm = -127.25, // Minimum signal power from SIS ICD = -157.25dBW = -127.25dbm
        double doppler_start = 0,
        double doppler_end = 0);

    void generate(double *signal, long long size);

private:
    double fs;
    double fc;
    double power_dbm;
    double doppler_start;
    double doppler_end;
};

#endif // SIG_GEN_H
