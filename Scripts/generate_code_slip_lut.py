import numpy as np

DOPS = [dop for dop in range(-5000, 5001, 250)]
# NUM_SLIP = 16 # Number of slip values to use

# slip_times = [1.0/(10.23e6) * (i << (24-int(np.log2(NUM_SLIP)))) for i in range(NUM_SLIP)]


# # Decimal code offsets
# print("      | ", end='')
# for slip_time in slip_times:
#     print(f"{slip_time:06.5f}", end=' | ')
# print()

# for dop in DOPS:
#     print(f"{dop:5d} | ", end='')
#     for slip_time in slip_times:
#         # determine all possible code slippage amount due to doppler
#         shift = (dop * 1023000.0 / 1575.42e6) * slip_time
#         print(f"{shift:7.2f}", end=' | ')
#     print()

# print()

# # Integer tenth code offsets
# print("      | ", end='')
# for slip_time in slip_times:
#     print(f"{slip_time:06.5f}", end=' | ')
# print()

# for dop in DOPS:
#     print(f"{dop:5d} | ", end='')
#     for slip_time in slip_times:
#         # determine all possible code slippage amount due to doppler
#         shift = int(np.round((dop * 1023000.0 / 1575.42e6) * slip_time * 10))
#         print(f"{shift:7d}", end=' | ')
#     print()

# print(f"Total LUT entries: {len(DOPS) * len(slip_times)}")

SEARCH_TIME = 0.224068832 # seconds

for dop in DOPS:
    # determine all possible code slippage amount in 10ths of a code chip
    shift = int(np.round(((dop * 1023000.0 / 1575.42e6) + 1023000.0) * SEARCH_TIME * 10)) % 10230
    print(f"{dop:5d} {int(np.floor(shift/10)):7d} {int(np.round((shift % 10)*(18.77/10))):3d}")