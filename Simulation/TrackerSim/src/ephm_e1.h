#ifndef EPHM_E1_H
#define EPHM_E1_H

#include <stdint.h>

class EphemerisE1B
{
public:
    EphemerisE1B();

    void process_message(uint8_t *message, int page_type);

    void get_satellite_ecef(double t, double *x, double *y, double *z);
    double get_clock_correction(double t);
    double time_from_epoch(double t, double t_epoch);
    double eccentric_anomaly(double t_k);

    void inc_time();
    uint32_t get_time() { return tGST; }
    bool ephm_valid() { return pages_received.page_1 && pages_received.page_2 &&
                               pages_received.page_3 && pages_received.page_4 &&
                               pages_received.page_5 && pages_received.page_10 &&
                               time_received; }

private:
    // Flags
    struct
    {
        bool page_1 : 1;
        bool page_2 : 1;
        bool page_3 : 1;
        bool page_4 : 1;
        bool page_5 : 1;
        bool page_10 : 1;
    } pages_received;

    // Time
    bool time_received;
    uint32_t tGST;

    // Page 0 - WN and TOW
    uint16_t wn;
    uint32_t tow;

    // Page 1 - Ephmeris 1/4
    uint16_t t_oe;
    int32_t M_0;
    uint32_t e;
    uint32_t root_A;

    // Page 2 - Ephemeris 2/4
    int32_t Omega_0;
    int32_t i_0;
    int32_t omega;
    int16_t IDOT;

    // Page 3 - Ephemeris 3/4, SISA
    int32_t omega_dot;
    int16_t delta_n;
    int16_t C_uc;
    int16_t C_us;
    int16_t C_rc;
    int16_t C_rs;

    // Page 4 - SVID, Ephemeris 4/4, clock correction
    int16_t C_ic;
    int16_t C_is;
    uint16_t t_oc;
    int32_t a_f0;
    int32_t a_f1;
    int8_t a_f2;

    // Page 5 - Ionospheric correction, BGD, health and validity, GST
    uint16_t a_i0;
    int16_t a_i1;
    int16_t a_i2;
    int16_t BGD;
    uint8_t signal_health;
    uint8_t data_validity;

    // Page 10 - GST-GPS conversion parameters
    int16_t A_0G;
    int16_t A_1G;
    uint8_t t_0G;
    uint8_t WN_0G;
};

#endif // EPHM_E1_H
