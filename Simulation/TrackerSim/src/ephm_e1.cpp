#include "ephm_e1.h"
#include "tools.h"
#include "math.h"

EphemerisE1B::EphemerisE1B()
{
    pages_received.page_1 = false;
    pages_received.page_2 = false;
    pages_received.page_3 = false;
    pages_received.page_4 = false;
    pages_received.page_5 = false;
    pages_received.page_10 = false;
}

void EphemerisE1B::process_message(uint8_t *message, int page_type)
{
    // Spare page
    if (page_type == 0 && message[6] && !message[7])
    {
        bytes_to_number(&wn, message + 96, 16, 12, 0);
        bytes_to_number(&tow, message + 108, 32, 20, 0);
        // Resynchronize tGST
        tGST = ((wn * 604800) + tow + 2) % ((uint32_t)604800 * 4096);
        time_received = true;
    }

    // Ephemeris 1/4
    if (page_type == 1)
    {
        bytes_to_number(&t_oe, message + 16, 16, 14, 0);
        bytes_to_number(&M_0, message + 30, 32, 32, 1);
        bytes_to_number(&e, message + 62, 32, 32, 0);
        bytes_to_number(&root_A, message + 94, 32, 32, 0);
        pages_received.page_1 = true;
    }

    // Ephemeris 2/4
    if (page_type == 2)
    {
        bytes_to_number(&Omega_0, message + 16, 32, 32, 1);
        bytes_to_number(&i_0, message + 48, 32, 32, 1);
        bytes_to_number(&omega, message + 80, 32, 32, 1);
        bytes_to_number(&IDOT, message + 112, 16, 14, 1);
        pages_received.page_2 = true;
    }

    // Ephemeris 3/4, SISA
    if (page_type == 3)
    {
        bytes_to_number(&omega_dot, message + 16, 32, 24, 1);
        bytes_to_number(&delta_n, message + 40, 16, 16, 1);
        bytes_to_number(&C_uc, message + 56, 16, 16, 1);
        bytes_to_number(&C_us, message + 72, 16, 16, 1);
        bytes_to_number(&C_rc, message + 88, 16, 16, 1);
        bytes_to_number(&C_rs, message + 104, 16, 16, 1);
        pages_received.page_3 = true;
    }

    // SVID, Ephemeris 4/4, clock correction
    if (page_type == 4)
    {
        bytes_to_number(&C_ic, message + 22, 16, 16, 1);
        bytes_to_number(&C_is, message + 38, 16, 16, 1);
        bytes_to_number(&t_oc, message + 54, 16, 14, 0);
        bytes_to_number(&a_f0, message + 68, 32, 31, 1);
        bytes_to_number(&a_f1, message + 99, 32, 21, 1);
        bytes_to_number(&a_f2, message + 120, 8, 6, 1);
        pages_received.page_4 = true;
    }

    // Ionospheric correction, BGD, health and validity, GST
    if (page_type == 5)
    {
        bytes_to_number(&a_i0, message + 6, 16, 11, 0);
        bytes_to_number(&a_i1, message + 17, 16, 11, 1);
        bytes_to_number(&a_i2, message + 28, 16, 14, 1);
        bytes_to_number(&BGD, message + 57, 16, 10, 1);
        bytes_to_number(&signal_health, message + 69, 8, 2, 0);
        bytes_to_number(&data_validity, message + 72, 8, 1, 0);
        bytes_to_number(&wn, message + 73, 16, 12, 0);
        bytes_to_number(&tow, message + 85, 32, 20, 0);
        // Resynchronize tGST
        tGST = ((wn * 604800) + tow + 2) % ((uint32_t)604800 * 4096);
        time_received = true;
        pages_received.page_5 = true;
    }

    // GST-GPS conversion parameters
    if (page_type == 10)
    {
        bytes_to_number(&A_0G, message + 86, 16, 16, 1);
        bytes_to_number(&A_1G, message + 102, 16, 12, 1);
        bytes_to_number(&t_0G, message + 114, 8, 8, 0);
        bytes_to_number(&WN_0G, message + 122, 8, 6, 0);
        pages_received.page_10 = true;
    }
}

void EphemerisE1B::get_satellite_ecef(double t, double *x, double *y, double *z)
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
    double _t_oe = t_oe * 60.0;

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

double EphemerisE1B::get_clock_correction(double t)
{
    double _a_f0 = a_f0 * pow(2, -34);
    double _a_f1 = a_f1 * pow(2, -46);
    double _a_f2 = a_f2 * pow(2, -59);
    double _root_A = root_A * pow(2, -19);
    double _e = e * pow(2, -33);
    double _t_oe = t_oe * 60;
    double _t_oc = t_oc * 60;
    double _BGD = BGD * pow(2, -32);

    double t_k = time_from_epoch(t, _t_oe);
    double E_k = eccentric_anomaly(t_k);
    double t_R = F * _e * _root_A * sin(E_k);
    t = time_from_epoch(t, _t_oc);

    double _A_0G = A_0G * pow(2, -35);
    double _A_1G = A_1G * pow(2, -51);
    double _t_0G = t_0G * 3600.0;
    double dt_systems = _A_0G + _A_1G * ((double)tow - _t_0G + ((wn - WN_0G) % 64) * 604800.0);

    return _a_f0 +
           _a_f1 * t +
           _a_f2 * (t * t) +
           t_R - _BGD + dt_systems;
}

double EphemerisE1B::time_from_epoch(double t, double t_epoch)
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

double EphemerisE1B::eccentric_anomaly(double t_k)
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