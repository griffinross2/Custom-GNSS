#include "ephm_l1ca.h"
#include "tools.h"
#include "math.h"
#include <stdio.h>

EphemerisL1CA::EphemerisL1CA()
{
    frames_received.subframe_1 = false;
    frames_received.subframe_2 = false;
    frames_received.subframe_3 = false;
    frames_received.subframe_4 = false;
}

void EphemerisL1CA::process_message(uint8_t *message)
{
    uint8_t subframe_id;
    bytes_to_number(&subframe_id, message + 49, 8, 3, 0);

    if (subframe_id == 0x1)
    {
        bytes_to_number(&ura, message + 72, 8, 4, 0);
        bytes_to_number(&sv_health, message + 76, 8, 6, 0);
        bytes_to_number(&T_GD, message + 196, 8, 8, 1);
        bytes_to_number(&t_oc, message + 218, 16, 16, 0);
        bytes_to_number(&a_f2, message + 240, 8, 8, 1);
        bytes_to_number(&a_f1, message + 248, 16, 16, 1);
        bytes_to_number(&a_f0, message + 270, 32, 22, 1);
        frames_received.subframe_1 = true;
        printf("sf1\n");
    }
    else if (subframe_id == 0x2)
    {
        bytes_to_number(&C_rs, message + 68, 16, 16, 1);
        bytes_to_number(&delta_n, message + 90, 16, 16, 1);
        int32_t M_0_up;
        int32_t M_0_dn;
        bytes_to_number(&M_0_up, message + 106, 32, 8, 0);
        bytes_to_number(&M_0_dn, message + 120, 32, 24, 0);
        M_0 = (M_0_up << 24) | M_0_dn;
        bytes_to_number(&C_uc, message + 150, 16, 16, 1);
        int32_t e_up;
        int32_t e_dn;
        bytes_to_number(&e_up, message + 166, 32, 8, 0);
        bytes_to_number(&e_dn, message + 180, 32, 24, 0);
        e = (e_up << 24) | e_dn;
        bytes_to_number(&C_us, message + 210, 16, 16, 1);
        uint32_t root_A_up;
        uint32_t root_A_dn;
        bytes_to_number(&root_A_up, message + 226, 32, 8, 0);
        bytes_to_number(&root_A_dn, message + 240, 32, 24, 0);
        root_A = (root_A_up << 24) | root_A_dn;
        bytes_to_number(&t_oe, message + 270, 16, 16, 0);
        frames_received.subframe_2 = true;
        printf("sf2\n");
    }
    else if (subframe_id == 0x3)
    {
        bytes_to_number(&C_ic, message + 60, 16, 16, 1);
        int32_t Omega_0_up;
        int32_t Omega_0_dn;
        bytes_to_number(&Omega_0_up, message + 76, 32, 8, 0);
        bytes_to_number(&Omega_0_dn, message + 90, 32, 24, 0);
        Omega_0 = (Omega_0_up << 24) | Omega_0_dn;
        bytes_to_number(&C_is, message + 120, 16, 16, 1);
        int32_t i_0_up;
        int32_t i_0_dn;
        bytes_to_number(&i_0_up, message + 136, 32, 8, 0);
        bytes_to_number(&i_0_dn, message + 150, 32, 24, 0);
        i_0 = (i_0_up << 24) | i_0_dn;
        bytes_to_number(&C_rc, message + 180, 16, 16, 1);
        int32_t omega_up;
        int32_t omega_dn;
        bytes_to_number(&omega_up, message + 196, 32, 8, 0);
        bytes_to_number(&omega_dn, message + 210, 32, 24, 0);
        omega = (omega_up << 24) | omega_dn;
        bytes_to_number(&omega_dot, message + 240, 32, 24, 1);
        bytes_to_number(&IDOT, message + 278, 16, 14, 1);
        frames_received.subframe_3 = true;
        printf("sf3\n");
    }
    else if (subframe_id == 0x4)
    {
        uint8_t sv_id;
        bytes_to_number(&sv_id, message + 62, 8, 6, 0);
        if (sv_id == 56)
        {
            bytes_to_number(&alpha_0, message + 68, 8, 8, 1);
            bytes_to_number(&alpha_1, message + 76, 8, 8, 1);
            bytes_to_number(&alpha_2, message + 90, 8, 8, 1);
            bytes_to_number(&alpha_3, message + 98, 8, 8, 1);
            bytes_to_number(&beta_0, message + 106, 8, 8, 1);
            bytes_to_number(&beta_1, message + 120, 8, 8, 1);
            bytes_to_number(&beta_2, message + 128, 8, 8, 1);
            bytes_to_number(&beta_3, message + 136, 8, 8, 1);
            frames_received.subframe_4 = true;
            printf("sf4\n");
        }
    }
}

void EphemerisL1CA::get_satellite_ecef(double t, double *x, double *y, double *z)
{
    double _root_A = root_A * pow(2, -19);
    double _e = e * pow(2, -33);
    double _omega = omega * pow(2, -31) * PI;
    double _C_uc = C_uc * pow(2, -29);
    double _C_us = C_us * pow(2, -29);
    double _C_rc = C_rc * pow(2, -5);
    double _C_rs = C_rs * pow(2, -5);
    double _C_ic = C_ic * pow(2, -29);
    double _C_is = C_is * pow(2, -29);
    double _i_0 = i_0 * pow(2, -31) * PI;
    double _IDOT = IDOT * pow(2, -43) * PI;
    double _Omega_0 = Omega_0 * pow(2, -31) * PI;
    double _omega_dot = omega_dot * pow(2, -43) * PI;
    double _t_oe = t_oe * pow(2, 4);

    double A = _root_A * _root_A;
    double t_k = time_from_epoch(t, _t_oe);
    double E_k = eccentric_anomaly(t_k);
    double v_k = atan2(sqrt(1 - _e * _e) * sin(E_k), cos(E_k) - _e);
    double phi_k = v_k + _omega;
    double delta_u_k = _C_us * sin(2 * phi_k) + _C_uc * cos(2 * phi_k);
    double delta_r_k = _C_rs * sin(2 * phi_k) + _C_rc * cos(2 * phi_k);
    double delta_i_k = _C_is * sin(2 * phi_k) + _C_ic * cos(2 * phi_k);
    double u_k = phi_k + delta_u_k;
    double r_k = A * (1 - _e * cos(E_k)) + delta_r_k;
    double i_k = _i_0 + delta_i_k + _IDOT * t_k;
    double Omega_k = _Omega_0 + (_omega_dot - omega_e) * t_k - omega_e * _t_oe;
    double x_k_prime = r_k * cos(u_k);
    double y_k_prime = r_k * sin(u_k);

    *x = x_k_prime * cos(Omega_k) - y_k_prime * cos(i_k) * sin(Omega_k);
    *y = x_k_prime * sin(Omega_k) + y_k_prime * cos(i_k) * cos(Omega_k);
    *z = y_k_prime * sin(i_k);
}

double EphemerisL1CA::get_clock_correction(double t)
{
    double _a_f0 = a_f0 * pow(2, -31);
    double _a_f1 = a_f1 * pow(2, -43);
    double _a_f2 = a_f2 * pow(2, -55);
    double _root_A = root_A * pow(2, -19);
    double _e = e * pow(2, -33);
    double _t_oe = t_oe * pow(2, 4);
    double _t_oc = t_oc * pow(2, 4);
    double _T_GD = T_GD * pow(2, -31);

    double t_k = time_from_epoch(t, _t_oe);
    double E_k = eccentric_anomaly(t_k);
    double t_R = F * _e * _root_A * sin(E_k);
    t = time_from_epoch(t, _t_oc);

    return _a_f0 +
           _a_f1 * t +
           _a_f2 * (t * t) +
           t_R - _T_GD;
}

double EphemerisL1CA::time_from_epoch(double t, double t_epoch)
{
    t -= t_epoch;
    if (t > 302400)
    {
        t -= 604800;
    }
    else if (t < -302400)
    {
        t += 604800;
    }
    return t;
}

double EphemerisL1CA::eccentric_anomaly(double t_k)
{
    double _root_A = root_A * pow(2, -19);
    double _delta_n = delta_n * pow(2, -43) * PI;
    double _M_0 = M_0 * pow(2, -31) * PI;
    double _e = e * pow(2, -33);

    double A = _root_A * _root_A;
    double n_0 = sqrt(mu / (A * A * A));
    double n = n_0 + _delta_n;
    double M_k = _M_0 + n * t_k;
    double E_k = M_k;

    int iter = 0;
    while (iter++ < E_K_ITER)
    {
        double E_k_new = M_k + _e * sin(E_k);
        if (fabs(E_k_new - E_k) < 1e-10)
        {
            E_k = E_k_new;
            break;
        }
        E_k = E_k_new;
    }
    return E_k;
}