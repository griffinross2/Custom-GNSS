#include "stdio.h"
#include "sig_gen.h"
#include "stdlib.h"
#include "acq_e1c.h"

#define FS 69.984e6
#define FC 9.334875e6

int main()
{
    GalileoSigGen sig_gen(FS, FC, 26.25 - 2, 0, 1000);
    const long long size = FS * 0.1;
    uint8_t *signal = (uint8_t *)malloc(size * sizeof(uint8_t));
    sig_gen.generate(signal, size);

    FILE *file = fopen("out.bin", "wb");
    if (file == NULL)
    {
        printf("Error opening file\n");
        free(signal);
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
        byte |= (signal[i] > 0) << (i % 8);
    }

    fclose(file);
    free(signal);

    return 0;
}