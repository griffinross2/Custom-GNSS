# Script to check magnitude bit density of a sample
# Should be ~33% for 1-bit sign, 1-bit magnitude

import numpy as np

def check_byte(by):
    ones = 0
    for i in range(8):
        if by & (1 << i) != 0:
            ones += 1

    return ones/8.0

def check_bit_density(fname):
    with open(fname, 'rb') as f:
        data = f.read()

    byte_densities = np.array(list(map(check_byte, data)))
    density = np.mean(byte_densities)
    std_dev = np.std(byte_densities)
    min_density = np.min(byte_densities)
    max_density = np.max(byte_densities)

    print(f"Number of bytes: {len(data)}")
    print(f"Bit density: {density*100.0:.2f}%")
    print(f"Standard deviation: {std_dev*100.0:.2f}%")
    print(f"Minimum density: {min_density*100.0:.2f}%")
    print(f"Maximum density: {max_density*100.0:.2f}%")

if __name__ == "__main__":
    check_bit_density("capture-5-4-25-1.bin")