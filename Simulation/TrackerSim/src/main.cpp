#include "stdio.h"
#include "sig_gen.h"
#include "stdlib.h"
#include "acq_l1ca.h"
#include "acq_e1c.h"
#include "track_l1ca.h"
#include "track_e1.h"
#include <windows.h>
#include "solve.h"

#define FS 69.984e6
#define FC 9.334875e6

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
    GPSL1CATracker gps2(26, FS, FC, -3400, 446.3);
    GPSL1CATracker gps3(5, FS, FC, 1400.0, 969.4);

    printf("Tracking Galileo...\n");

    // Track Galileo
    GalileoE1Tracker gal0(24, FS, FC, -250.0, 2838.0, dll_bw, pll_bw, fll_bw);
    GalileoE1Tracker gal1(14, FS, FC, -3250.0, 3770.6, dll_bw, pll_bw, fll_bw);
    GalileoE1Tracker gal2(26, FS, FC, 1000.0, 1001.1, dll_bw, pll_bw, fll_bw);

    // Solver
    Solution solution;
    Solver solver;
    solver.register_l1ca_channel(&gps0);
    solver.register_l1ca_channel(&gps1);
    solver.register_l1ca_channel(&gps2);
    solver.register_l1ca_channel(&gps3);
    solver.register_e1_channel(&gal0);
    solver.register_e1_channel(&gal1);
    solver.register_e1_channel(&gal2);

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
        gal0.track(&total_signal, 1);
        gal1.track(&total_signal, 1);
        gal2.track(&total_signal, 1);
        gps0.track(&total_signal, 1);
        gps1.track(&total_signal, 1);
        gps2.track(&total_signal, 1);
        gps3.track(&total_signal, 1);
        // if (gal0.ready_to_solve())
        // {
        //     double x, y, z;
        //     double t;
        //     t = gal0.get_tx_time();
        //     t -= gal0.get_clock_correction(t);
        //     gal0.get_satellite_ecef(t, &x, &y, &z);
        //     printf("%.12f,%.12f,%.12f\n", x, y, z);
        // }
        if (i % (long long)FS == 0)
        {
            if (solver.solve(&solution))
            {
                printf("Solution: lat,lon,alt,tbias: %.7f,%.7f,%.2f,%.7f\n", solution.lat, solution.lon, solution.alt, solution.t_bias);
            }
        }
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