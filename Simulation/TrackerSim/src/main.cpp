#include "stdio.h"
#include "sig_gen.h"
#include "stdlib.h"
#include "acq_l1ca.h"
#include "acq_e1c.h"

#define FS 69.984e6
#define FC 9.334875e6

int main()
{
    GalileoE1SigGen sig_gen(FS, FC, -127.25, 0, 1000);
    GPSL1CASigGen sig_gen2(FS, FC, -128.5 + 13, 0, 1000);
    NoiseGen noise_gen(FS, FC, 18e6);
    const long long size = int(FS * 1);
    double *signal = (double *)malloc(size * sizeof(double));
    double *signal2 = (double *)malloc(size * sizeof(double));
    double *noise = (double *)malloc(size * sizeof(double));
    uint8_t *total_signal = (uint8_t *)malloc(size * sizeof(uint8_t));

    printf("Generating signals...\n");

    sig_gen.generate(signal, size);
    sig_gen2.generate(signal2, size);
    noise_gen.generate(noise, size);

    // Combine signals
    for (long long i = 0; i < size; i++)
    {
        // Combine and hard-limit
        total_signal[i] = (signal[i] + signal2[i] + noise[i]) > 0 ? 1 : 0; //(signal[i] + signal2[i]) > 0 ? 1 : 0;
    }

    printf("Saving signal data...\n");

    FILE *file = fopen("out.bin", "wb");
    if (file == NULL)
    {
        printf("Error opening file\n");
        free(signal);
        free(signal2);
        free(noise);
        free(total_signal);
        return 1;
    }
    char byte = 0;
    for (long long i = 0; i < size; i++)
    {
        if (i > 0 && i % 8 == 0)
        {
            fwrite(&byte, sizeof(char), 1, file);
            byte = 0;
        }
        byte |= (total_signal[i] > 0) << (i % 8);
    }

    fclose(file);

    printf("Acquiring GPS...\n");

    // Acquire GPS
    for (int prn = 1; prn <= 1; prn++)
    {
        acquire_l1ca(prn, total_signal, 10, 0);
    }

    printf("Acquiring Galileo...\n");

    // Acquire Galileo
    for (int prn = 1; prn <= 1; prn++)
    {
        acquire_e1c(prn, total_signal, 4, 0);
    }

    free(signal);
    free(signal2);
    free(noise);
    free(total_signal);

    return 0;
}