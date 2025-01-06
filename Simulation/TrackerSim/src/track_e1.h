#ifndef TRACK_E1_H
#define TRACK_E1_H

#include <stdint.h>
#include "tools.h"
#include "filters.h"

#define PROMPT_LEN 100
#define VEL_LEN 10

typedef enum
{
    E1_PILOT_NO_LOCK = 0,
    E1_PILOT_LOCK_NO_SEC = 1,
    E1_PILOT_LOCK_SEC = 2,
} e1_pilot_tracking_state_t;

class GalileoE1Tracker
{
public:
    GalileoE1Tracker(
        int sv,
        double fs = 69.984e6,
        double fc = 9.334875e6,
        double doppler = 0,
        double code_phase = 0);

    ~GalileoE1Tracker();

    void track(uint8_t *signal, long long size);

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
    GalileoE1CodeGenerator *ca_code_very_early;

    // Code generator outputs
    uint8_t code_very_early;
    uint8_t code_early;
    uint8_t code_prompt;
    uint8_t code_late;
    uint8_t code_very_late;

    // BOC generator
    uint8_t boc1;

    // Accumulators (very early, early, prompt, late, very late for I and Q)
    int ive;
    int qve;
    int ie;
    int qe;
    int ip;
    int qp;
    int il;
    int ql;
    int ivl;
    int qvl;

    // Variable to detect if this epoch has been processed
    bool epoch_processed;

    // DLL filter
    SecondOrderPLL *dll;

    // PLL filter
    ThirdOrderPLL *pll;

    long long ms_elapsed;

    // Prompt buffers
    double ip_buffer[PROMPT_LEN];
    double qp_buffer[PROMPT_LEN];
    int prompt_len;
    int prompt_idx;

    // VE, VL buffers
    double ve_p_squared_buffer[VEL_LEN];
    double p_p_squared_buffer[VEL_LEN];
    double vl_p_squared_buffer[VEL_LEN];
    double ve_p_squared_sum;
    double p_p_squared_sum;
    double vl_p_squared_sum;
    int vel_p_squared_len;
    int vel_p_squared_idx;

    // Code tracking adjustments
    double half_el_spacing;
    double discriminator_factor;

    // Pilot tracking
    e1_pilot_tracking_state_t pilot_state;
    uint8_t pilot_secondary_acc[25];
    int pilot_secondary_idx;
    int pilot_secondary_chip;
    int pilot_secondary_pol;

    // Private functions
    void update_sample(uint8_t signal_sample);
    void update_epoch();
};

#endif // TRACK_E1_H
