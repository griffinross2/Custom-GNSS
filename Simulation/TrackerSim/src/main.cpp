#include "stdio.h"
#include "sig_gen.h"
#include "stdlib.h"
#include "acq_l1ca.h"
#include "acq_e1c.h"
#include "track_l1ca.h"
#include "track_e1.h"
#include <windows.h>

#define FS 69.984e6
#define FC 9.334875e6

typedef struct
{
    GalileoE1Tracker *track;
    uint8_t *signal;
    long long size;
} PARAMS;

DWORD WINAPI ThreadFunction(LPVOID lpParam)
{
    ((PARAMS *)lpParam)->track->track(((PARAMS *)lpParam)->signal, ((PARAMS *)lpParam)->size);
    return 0;
}

void save_signal_data(uint8_t *signal, long long size);

int main(int argc, char *argv[])
{
    // GalileoE1SigGen sig_gen(FS, FC, -127.25, 0, 10);
    // GPSL1CASigGen sig_gen2(FS, FC, -128.5 + 30, 0, 10);
    // NoiseGen noise_gen(FS, FC, 18e6);
    SignalFromFile sig_gen;
    const long long size = (long long)(FS * (long long)35);

    // printf("Generating signals...\n");

    if (!sig_gen.open("gnss-20170427-L1.1bit.I.bin"))
    {
        printf("Error opening file\n");
        return 1;
    }

    double dll_bw = 5.0;
    double pll_bw = 35.0;
    double fll_bw = 35.0;
    if (argc >= 4)
    {
        dll_bw = atof(argv[1]);
        pll_bw = atof(argv[2]);
        fll_bw = atof(argv[3]);
    }

    printf("Tracking GPS...\n");

    // Track GPS
    GPSL1CATracker gps0(2, FS, FC, 1600.0, 7.0);
    GPSL1CATracker gps1(21, FS, FC, -2400.0, 817.4);

    printf("Tracking Galileo...\n");

    // Track Galileo
    GalileoE1Tracker gal1(24, FS, FC, -250.0, 2838.0, dll_bw, pll_bw, fll_bw);
    GalileoE1Tracker gal2(14, FS, FC, -3250.0, 3770.6, dll_bw, pll_bw, fll_bw);
    GalileoE1Tracker gal3(26, FS, FC, 1000.0, 1001.1, dll_bw, pll_bw, fll_bw);

    // Combine signals
    for (long long i = 0; i < size; i++)
    {
        if (i % (long long)FS == 0)
        {
            printf("Time elapsed: %lld s\n", i / (long long)FS);
        }

        // Generate combine and hard-limit
        double signal = 0;
        sig_gen.generate(&signal, 1);
        uint8_t total_signal = (signal /*+ signal2[i] + noise[i]*/) > 0 ? 1 : 0;
        // gal1.track(&total_signal, 1);
        // gal2.track(&total_signal, 1);
        // gal3.track(&total_signal, 1);
        gps0.track(&total_signal, 1);
    }

    // printf("Acquiring GPS...\n");

    // Acquire GPS
    // for (int prn = 2; prn <= 2; prn++)
    // {
    //     acquire_l1ca(prn, total_signal, 10, 0);
    // }

    // printf("Acquiring Galileo...\n");

    // // Acquire Galileo
    // for (int prn = 0; prn <= 36; prn++)
    // {
    //     acquire_e1c(prn, total_signal, 8, 0);
    // }

    // PARAMS params1 = {&track1, total_signal, size};
    // PARAMS params2 = {&track2, total_signal, size};
    // PARAMS params3 = {&track3, total_signal, size};

    // HANDLE hThreadArray[3];

    // hThreadArray[0] = CreateThread(NULL, 0, ThreadFunction, (LPVOID)&params1, 0, NULL);
    // hThreadArray[1] = CreateThread(NULL, 0, ThreadFunction, (LPVOID)&params2, 0, NULL);
    // hThreadArray[2] = CreateThread(NULL, 0, ThreadFunction, (LPVOID)&params3, 0, NULL);

    // WaitForMultipleObjects(3, hThreadArray, TRUE, INFINITE);

    sig_gen.close();

    return 0;
}

void save_signal_data(uint8_t *signal, long long size)
{
    FILE *file = fopen("out.bin", "wb");
    if (file == NULL)
    {
        printf("Error opening file\n");
        return;
    }
    char byte = 0;
    for (long long i = 0; i < size; i++)
    {
        if (i > 0 && i % 8 == 0)
        {
            fwrite(&byte, sizeof(char), 1, file);
            byte = 0;
        }
        byte |= (signal[i] > 0) << (i % 8);
    }

    fclose(file);
}