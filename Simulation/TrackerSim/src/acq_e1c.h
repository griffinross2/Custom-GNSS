#ifndef ACQ_E1C_H
#define ACQ_E1C_H

#include "stdint.h"

int acquire_e1c(int sv = 1, uint8_t *signal_in = nullptr, int len_ms = 4, int start_ms = 0);

#endif // ACQ_E1C_H
