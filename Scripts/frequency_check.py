# Check the frequency of some periodic signal sample

import numpy as np
import matplotlib.pyplot as plt

def gen_sample(fname):
    with open(fname, 'rb') as f:
        data = f.read()

    # Unpack bits into bytes
    sample = np.frombuffer(data, dtype=np.uint8)
    sample = np.unpackbits(sample, bitorder='little')

    # Convert from 0s and 1s to -1s and 1s
    sample = np.where(sample == 0, -1, 1)

    return sample

def check_frequency(sample):
    # Calculate the frequency of the signal
    # Sampling rate is 19.2 MHz
    freq = np.fft.fftfreq(len(sample), d=1/19200000.0)
    spectrum = np.fft.fft(sample)

    # Get the magnitude of the spectrum
    magnitude = np.abs(spectrum)
    magnitude[0] = 0  # Remove DC component

    plt.figure()
    plt.plot(freq, magnitude)
    plt.show(block=False)

    # Find the peak frequency
    max_idx = np.argmax(magnitude)
    peak_freq = freq[np.argmax(magnitude)]
    print(f"Max: {peak_freq} at {max_idx}")

    return peak_freq

if __name__ == "__main__":
    sample = gen_sample("capture_preset.bin")
    peak_freq = check_frequency(sample)

    sample = gen_sample("capture_spi.bin")
    peak_freq = check_frequency(sample)

    plt.show()