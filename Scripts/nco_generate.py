import numpy as np

DOP = [x for x in range(-5000, 5001, 250)]

for dop in DOP:
    f_code = 1023000 + (dop * 1023000.0 / 1575.42e6)
    f_lo = 4020000 + dop

    fcw_code = int(np.round(f_code * (2**32) / 19200000.0))
    fcw_lo = int(np.round(f_lo * (2**32) / 19200000.0))

    print(f"{dop:5d} {fcw_code:10d} {fcw_lo:10d}")