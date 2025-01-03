#ifndef ACQ_L1CA_H
#define ACQ_L1CA_H

#include "stdint.h"

int acquire_l1ca(int sv = 1, uint8_t *signal_in = nullptr, int len_ms = 4, int start_ms = 0);

#endif // ACQ_L1CA_H
