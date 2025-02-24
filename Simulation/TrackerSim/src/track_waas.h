#ifndef TRACK_WAAS_H
#define TRACK_WAAS_H

#include <stdint.h>
#include "tools.h"
#include "filters.h"
// #include "ephm_waas.h"

class SBASWAASTracker
{
public:
    SBASWAASTracker(
        int sv,
        double fs = 69.984e6,
        double fc = 9.334875e6,
        double doppler = 0,
        double code_phase = 0);

    ~SBASWAASTracker();

    void track(uint8_t *signal, long long size);

    double get_tx_time();
    void get_satellite_ecef(double t, double *x, double *y, double *z);
    double get_clock_correction(double t);
    bool ready_to_solve();
    double get_cn0() { return cn0; }
    int get_sv() { return sv; }

private:
    int sv;
    double fs;
    double fc;
    double doppler;
    double code_off;

    // NCOs
    double code_phase;
    double code_rate;
    double carrier_phase;
    double carrier_rate;

    // Code generator
    int start_chip;
    WAASCodeGenerator *code_gen;

    // Code generator outputs
    uint8_t code_early;
    uint8_t code_prompt;
    uint8_t code_late;

    // Accumulators (early, prompt, late for I and Q)
    int ie;
    int qe;
    int ip;
    int qp;
    int il;
    int ql;

    // Variable to detect if this epoch has been processed
    bool epoch_processed;

    // DLL filter
    PLL *dll;

    // PLL filter
    PLL *pll;

    // Time
    long long ms_elapsed;

    // Prompt buffers
    double ip_buffer[100];
    double qp_buffer[100];
    int prompt_len;
    int prompt_idx;

    // Bit sync
    int last_ip;
    int bit_sync_count;
    int bit_hist[2];
    bool bit_synced;
    int bit_ms;
    int bit_sum;

    // Bit buffer
    uint8_t nav_buf[500];
    int nav_count;

    // Nav
    bool nav_valid;
    int last_z_count;

    // Ephemeris
    // EphemerisWAAS ephm;

    // SNR
    double cn0;

    // Private functions
    void
    update_sample(uint8_t signal_sample);
    void update_epoch();
    void update_nav();
};

#endif // TRACK_WAAS_H
