#ifndef ACQ_WAAS_H
#define ACQ_WAAS_H

#include "stdint.h"

int acquire_waas(int sv_idx, uint8_t *signal_in = nullptr, int len_ms = 4, int start_ms = 0);

#endif // ACQ_WAAS_H
