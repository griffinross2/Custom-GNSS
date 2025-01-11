#ifndef EPHM_L1CA_H
#define EPHM_L1CA_H

#include <stdint.h>

class EphemerisL1CA
{
public:
    EphemerisL1CA();

    void process_message(uint8_t *message);

    void get_satellite_ecef(double t, double *x, double *y, double *z);
    double get_clock_correction(double t);
    double time_from_epoch(double t, double t_epoch);
    double eccentric_anomaly(double t_k);

    bool ephm_valid() { return frames_received.subframe_1 && frames_received.subframe_2 &&
                               frames_received.subframe_3; }

private:
    // Flags
    struct
    {
        bool subframe_1 : 1;
        bool subframe_2 : 1;
        bool subframe_3 : 1;
        bool subframe_4 : 1;
    } frames_received;

    // Subframe 1
    uint8_t ura;
    uint8_t sv_health;
    int8_t T_GD;
    uint16_t t_oc;
    int8_t a_f2;
    int16_t a_f1;
    int32_t a_f0;

    // Subframes 2 and 3
    int16_t C_rs;
    int16_t delta_n;
    int32_t M_0;
    int16_t C_uc;
    uint32_t e;
    int16_t C_us;
    uint32_t root_A;
    uint16_t t_oe;
    int16_t C_ic;
    int32_t Omega_0;
    int16_t C_is;
    int32_t i_0;
    int16_t C_rc;
    int32_t omega;
    int32_t omega_dot;
    int16_t IDOT;

    // Subframe 4 page 18
    int8_t alpha_0;
    int8_t alpha_1;
    int8_t alpha_2;
    int8_t alpha_3;
    int8_t beta_0;
    int8_t beta_1;
    int8_t beta_2;
    int8_t beta_3;
};

#endif // EPHM_L1CA_H
