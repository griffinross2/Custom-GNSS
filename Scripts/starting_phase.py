import numpy as np

start_indexes = [x for x in range(0, 19)]

for start_index in start_indexes:
    phase = (start_index / 18.77) * (2**32)
    phase =int(np.round(phase))

    print(f"{start_index:2d} {phase:10d}")