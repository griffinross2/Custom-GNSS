#ifndef SOLVE_H
#define SOLVE_H

#include <vector>
#include "track_l1ca.h"
#include "track_e1.h"

typedef struct
{
    double lat;
    double lon;
    double alt;
    double t_bias;
} Solution;

class Solver
{
public:
    Solver();

    bool solve(Solution *solution);

    void register_l1ca_channel(GPSL1CATracker *channel);
    void register_e1_channel(GalileoE1Tracker *channel);

private:
    std::vector<GPSL1CATracker *> gps_l1ca_channels;
    std::vector<GalileoE1Tracker *> gal_e1_channels;
};

#endif // SOLVE_H
