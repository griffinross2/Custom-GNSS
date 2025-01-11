#include "solve.h"
#include "stdio.h"

#define WGS84_A 6378137.0
#define WGS84_F_INV 298.257223563
#define WGS84_B 6356752.31424518
#define WGS84_E2 0.00669437999014132
#define C 299792458.0

#define MAX_ITER 20

void to_coords(double x, double y, double z, double *lat, double *lon, double *alt);

Solver::Solver()
{
}

bool Solver::solve(Solution *solution)
{
    double x = 0;
    double y = 0;
    double z = 0;
    double t_bias = 0;

    std::vector<GPSL1CATracker *> gps_l1ca_in_solution;
    std::vector<GalileoE1Tracker *> gal_e1_in_solution;

    // Add ready GPS L1CA channels to the solution
    for (size_t i = 0; i < gps_l1ca_channels.size(); i++)
    {
        if (gps_l1ca_channels.at(i)->ready_to_solve())
        {
            gps_l1ca_in_solution.push_back(gps_l1ca_channels.at(i));
        }
    }

    // Add ready Galileo E1 channels to the solution
    for (size_t i = 0; i < gal_e1_channels.size(); i++)
    {
        if (gal_e1_channels.at(i)->ready_to_solve())
        {
            gal_e1_in_solution.push_back(gal_e1_channels.at(i));
        }
    }

    size_t num_l1ca = gps_l1ca_in_solution.size();
    size_t num_e1 = gal_e1_in_solution.size();
    size_t num_chans = num_l1ca + num_e1;

    // Not enough channels
    if (num_chans < 4)
    {
        return false;
    }

    // Allocate workspace
    double *t_tx = (double *)malloc(sizeof(double) * num_chans);
    double *x_sat = (double *)malloc(sizeof(double) * num_chans);
    double *y_sat = (double *)malloc(sizeof(double) * num_chans);
    double *z_sat = (double *)malloc(sizeof(double) * num_chans);
    double *weights = (double *)malloc(sizeof(double) * num_chans);
    double *dPR = (double *)malloc(sizeof(double) * num_chans);

    double **jac = (double **)malloc(sizeof(double *) * num_chans);
    for (size_t i = 0; i < num_chans; i++)
    {
        jac[i] = (double *)malloc(sizeof(double) * 4);
    }

    double *mc[4];
    for (size_t i = 0; i < 4; i++)
    {
        mc[i] = (double *)malloc(sizeof(double) * num_chans);
    }

    double ma[4][4], mb[4][4], md[4];
    double t_rx;
    double t_pc = 0;

    // Get satellite positions and pseudoranges
    for (size_t i = 0; i < num_l1ca; i++)
    {
        t_tx[i] = gps_l1ca_in_solution.at(i)->get_tx_time();
        t_tx[i] -= gps_l1ca_in_solution.at(i)->get_clock_correction(t_tx[i]);
        printf("gps %lld, t_tx: %.8f\n", i, t_tx[i]);
        gps_l1ca_in_solution.at(i)->get_satellite_ecef(t_tx[i], x_sat + i, y_sat + i, z_sat + i);
        t_pc += t_tx[i];
        weights[i] = 1;
    }

    for (size_t i = num_l1ca; i < num_chans; i++)
    {
        t_tx[i] = gal_e1_in_solution.at(i - num_l1ca)->get_tx_time();
        t_tx[i] -= gal_e1_in_solution.at(i - num_l1ca)->get_clock_correction(t_tx[i]);
        printf("gal %lld, t_tx: %.8f\n", i - num_l1ca, t_tx[i]);
        gal_e1_in_solution.at(i - num_l1ca)->get_satellite_ecef(t_tx[i], x_sat + i, y_sat + i, z_sat + i);
        t_pc += t_tx[i];
        weights[i] = 1;
    }

    // Starting value for receiver time
    t_pc = t_pc / num_chans + 75e-3;

    // Taylor series solution
    int i;
    for (i = 0; i < MAX_ITER; i++)
    {
        t_rx = t_pc - t_bias;

        for (size_t j = 0; j < num_chans; j++)
        {
            // SV position in ECI
            double theta = (t_tx[j] - t_rx) * omega_e;
            double x_sat_eci = x_sat[j] * cos(theta) - y_sat[j] * sin(theta);
            double y_sat_eci = x_sat[j] * sin(theta) + y_sat[j] * cos(theta);
            double z_sat_eci = z_sat[j];

            // Geometric range
            double gr = sqrt(
                (x - x_sat_eci) * (x - x_sat_eci) +
                (y - y_sat_eci) * (y - y_sat_eci) +
                (z - z_sat_eci) * (z - z_sat_eci));

            // Pseudorange error
            dPR[j] = C * (t_rx - t_tx[j]) - gr;

            // Jacobian
            jac[j][0] = (x - x_sat_eci) / gr;
            jac[j][1] = (y - y_sat_eci) / gr;
            jac[j][2] = (z - z_sat_eci) / gr;
            jac[j][3] = C;
        }

        // ma = H^T * W * H
        for (int r = 0; r < 4; r++)
        {
            for (int c = 0; c < 4; c++)
            {
                ma[r][c] = 0;
                for (int j = 0; j < num_chans; j++)
                {
                    ma[r][c] += jac[j][r] * weights[j] * jac[j][c];
                }
            }
        }

        // Determinant
        double determinant =
            ma[0][3] * ma[1][2] * ma[2][1] * ma[3][0] - ma[0][2] * ma[1][3] * ma[2][1] * ma[3][0] - ma[0][3] * ma[1][1] * ma[2][2] * ma[3][0] + ma[0][1] * ma[1][3] * ma[2][2] * ma[3][0] +
            ma[0][2] * ma[1][1] * ma[2][3] * ma[3][0] - ma[0][1] * ma[1][2] * ma[2][3] * ma[3][0] - ma[0][3] * ma[1][2] * ma[2][0] * ma[3][1] + ma[0][2] * ma[1][3] * ma[2][0] * ma[3][1] +
            ma[0][3] * ma[1][0] * ma[2][2] * ma[3][1] - ma[0][0] * ma[1][3] * ma[2][2] * ma[3][1] - ma[0][2] * ma[1][0] * ma[2][3] * ma[3][1] + ma[0][0] * ma[1][2] * ma[2][3] * ma[3][1] +
            ma[0][3] * ma[1][1] * ma[2][0] * ma[3][2] - ma[0][1] * ma[1][3] * ma[2][0] * ma[3][2] - ma[0][3] * ma[1][0] * ma[2][1] * ma[3][2] + ma[0][0] * ma[1][3] * ma[2][1] * ma[3][2] +
            ma[0][1] * ma[1][0] * ma[2][3] * ma[3][2] - ma[0][0] * ma[1][1] * ma[2][3] * ma[3][2] - ma[0][2] * ma[1][1] * ma[2][0] * ma[3][3] + ma[0][1] * ma[1][2] * ma[2][0] * ma[3][3] +
            ma[0][2] * ma[1][0] * ma[2][1] * ma[3][3] - ma[0][0] * ma[1][2] * ma[2][1] * ma[3][3] - ma[0][1] * ma[1][0] * ma[2][2] * ma[3][3] + ma[0][0] * ma[1][1] * ma[2][2] * ma[3][3];

        // mb = inverse(ma)
        mb[0][0] = (ma[1][2] * ma[2][3] * ma[3][1] - ma[1][3] * ma[2][2] * ma[3][1] + ma[1][3] * ma[2][1] * ma[3][2] - ma[1][1] * ma[2][3] * ma[3][2] - ma[1][2] * ma[2][1] * ma[3][3] + ma[1][1] * ma[2][2] * ma[3][3]) / determinant;
        mb[0][1] = (ma[0][3] * ma[2][2] * ma[3][1] - ma[0][2] * ma[2][3] * ma[3][1] - ma[0][3] * ma[2][1] * ma[3][2] + ma[0][1] * ma[2][3] * ma[3][2] + ma[0][2] * ma[2][1] * ma[3][3] - ma[0][1] * ma[2][2] * ma[3][3]) / determinant;
        mb[0][2] = (ma[0][2] * ma[1][3] * ma[3][1] - ma[0][3] * ma[1][2] * ma[3][1] + ma[0][3] * ma[1][1] * ma[3][2] - ma[0][1] * ma[1][3] * ma[3][2] - ma[0][2] * ma[1][1] * ma[3][3] + ma[0][1] * ma[1][2] * ma[3][3]) / determinant;
        mb[0][3] = (ma[0][3] * ma[1][2] * ma[2][1] - ma[0][2] * ma[1][3] * ma[2][1] - ma[0][3] * ma[1][1] * ma[2][2] + ma[0][1] * ma[1][3] * ma[2][2] + ma[0][2] * ma[1][1] * ma[2][3] - ma[0][1] * ma[1][2] * ma[2][3]) / determinant;
        mb[1][0] = (ma[1][3] * ma[2][2] * ma[3][0] - ma[1][2] * ma[2][3] * ma[3][0] - ma[1][3] * ma[2][0] * ma[3][2] + ma[1][0] * ma[2][3] * ma[3][2] + ma[1][2] * ma[2][0] * ma[3][3] - ma[1][0] * ma[2][2] * ma[3][3]) / determinant;
        mb[1][1] = (ma[0][2] * ma[2][3] * ma[3][0] - ma[0][3] * ma[2][2] * ma[3][0] + ma[0][3] * ma[2][0] * ma[3][2] - ma[0][0] * ma[2][3] * ma[3][2] - ma[0][2] * ma[2][0] * ma[3][3] + ma[0][0] * ma[2][2] * ma[3][3]) / determinant;
        mb[1][2] = (ma[0][3] * ma[1][2] * ma[3][0] - ma[0][2] * ma[1][3] * ma[3][0] - ma[0][3] * ma[1][0] * ma[3][2] + ma[0][0] * ma[1][3] * ma[3][2] + ma[0][2] * ma[1][0] * ma[3][3] - ma[0][0] * ma[1][2] * ma[3][3]) / determinant;
        mb[1][3] = (ma[0][2] * ma[1][3] * ma[2][0] - ma[0][3] * ma[1][2] * ma[2][0] + ma[0][3] * ma[1][0] * ma[2][2] - ma[0][0] * ma[1][3] * ma[2][2] - ma[0][2] * ma[1][0] * ma[2][3] + ma[0][0] * ma[1][2] * ma[2][3]) / determinant;
        mb[2][0] = (ma[1][1] * ma[2][3] * ma[3][0] - ma[1][3] * ma[2][1] * ma[3][0] + ma[1][3] * ma[2][0] * ma[3][1] - ma[1][0] * ma[2][3] * ma[3][1] - ma[1][1] * ma[2][0] * ma[3][3] + ma[1][0] * ma[2][1] * ma[3][3]) / determinant;
        mb[2][1] = (ma[0][3] * ma[2][1] * ma[3][0] - ma[0][1] * ma[2][3] * ma[3][0] - ma[0][3] * ma[2][0] * ma[3][1] + ma[0][0] * ma[2][3] * ma[3][1] + ma[0][1] * ma[2][0] * ma[3][3] - ma[0][0] * ma[2][1] * ma[3][3]) / determinant;
        mb[2][2] = (ma[0][1] * ma[1][3] * ma[3][0] - ma[0][3] * ma[1][1] * ma[3][0] + ma[0][3] * ma[1][0] * ma[3][1] - ma[0][0] * ma[1][3] * ma[3][1] - ma[0][1] * ma[1][0] * ma[3][3] + ma[0][0] * ma[1][1] * ma[3][3]) / determinant;
        mb[2][3] = (ma[0][3] * ma[1][1] * ma[2][0] - ma[0][1] * ma[1][3] * ma[2][0] - ma[0][3] * ma[1][0] * ma[2][1] + ma[0][0] * ma[1][3] * ma[2][1] + ma[0][1] * ma[1][0] * ma[2][3] - ma[0][0] * ma[1][1] * ma[2][3]) / determinant;
        mb[3][0] = (ma[1][2] * ma[2][1] * ma[3][0] - ma[1][1] * ma[2][2] * ma[3][0] - ma[1][2] * ma[2][0] * ma[3][1] + ma[1][0] * ma[2][2] * ma[3][1] + ma[1][1] * ma[2][0] * ma[3][2] - ma[1][0] * ma[2][1] * ma[3][2]) / determinant;
        mb[3][1] = (ma[0][1] * ma[2][2] * ma[3][0] - ma[0][2] * ma[2][1] * ma[3][0] + ma[0][2] * ma[2][0] * ma[3][1] - ma[0][0] * ma[2][2] * ma[3][1] - ma[0][1] * ma[2][0] * ma[3][2] + ma[0][0] * ma[2][1] * ma[3][2]) / determinant;
        mb[3][2] = (ma[0][2] * ma[1][1] * ma[3][0] - ma[0][1] * ma[1][2] * ma[3][0] - ma[0][2] * ma[1][0] * ma[3][1] + ma[0][0] * ma[1][2] * ma[3][1] + ma[0][1] * ma[1][0] * ma[3][2] - ma[0][0] * ma[1][1] * ma[3][2]) / determinant;
        mb[3][3] = (ma[0][1] * ma[1][2] * ma[2][0] - ma[0][2] * ma[1][1] * ma[2][0] + ma[0][2] * ma[1][0] * ma[2][1] - ma[0][0] * ma[1][2] * ma[2][1] - ma[0][1] * ma[1][0] * ma[2][2] + ma[0][0] * ma[1][1] * ma[2][2]) / determinant;

        // mc = inverse(H^T * W * H) * H^T
        for (size_t r = 0; r < 4; r++)
        {
            for (size_t c = 0; c < num_chans; c++)
            {
                mc[r][c] = 0;
                for (int j = 0; j < 4; j++)
                {
                    mc[r][c] += mb[r][j] * jac[c][j];
                }
            }
        }

        // md = inverse(H^T * W * H) * H^T * W * dPR
        for (size_t r = 0; r < 4; r++)
        {
            md[r] = 0;
            for (size_t j = 0; j < num_chans; j++)
            {
                md[r] += mc[r][j] * weights[j] * dPR[j];
            }
        }

        double dx = md[0];
        double dy = md[1];
        double dz = md[2];
        double dt = md[3];

        double error = sqrt(dx * dx + dy * dy + dz * dz);
        if (error < 1.0)
        {
            break;
        }

        x += dx;
        y += dy;
        z += dz;
        t_bias += dt;
    }

    // Get result
    to_coords(x, y, z, &solution->lat, &solution->lon, &solution->alt);
    solution->t_bias = t_bias;

    // Free workspace
    free(t_tx);
    free(x_sat);
    free(y_sat);
    free(z_sat);
    free(weights);
    free(dPR);
    for (size_t i = 0; i < num_chans; i++)
    {
        free(jac[i]);
    }
    free(jac);
    for (size_t i = 0; i < 4; i++)
    {
        free(mc[i]);
    }

    return (i != MAX_ITER);
}

void to_coords(double x, double y, double z, double *lat, double *lon, double *alt)
{
    const double a = WGS84_A;
    const double e2 = WGS84_E2;
    const double p = sqrt(x * x + y * y);

    *lon = 2.0 * atan2(y, x + p);
    *lat = atan(z / (p * (1.0 - e2)));
    *alt = 0.0;

    for (int i = 0; i < 100; i++)
    {
        double N = a / sqrt(1.0 - e2 * sin(*lat) * sin(*lat));
        double alt_new = p / cos(*lat) - N;
        *lat = atan(z / (p * (1.0 - e2 * N / (N + alt_new))));
        if (fabs(alt_new - *alt) < 1e-3)
        {
            *alt = alt_new;
            break;
        }
        *alt = alt_new;
    }

    *lat *= 180.0 / PI;
    *lon *= 180.0 / PI;
}

void Solver::register_l1ca_channel(GPSL1CATracker *channel)
{
    gps_l1ca_channels.push_back(channel);
}

void Solver::register_e1_channel(GalileoE1Tracker *channel)
{
    gal_e1_channels.push_back(channel);
}