import numpy as np

N = 32768 * 4

def init_sin_lut_q15():
    sin_lut_q15 = np.zeros((N//4,), dtype='int16')
    for k in range(N//4):
        angle = 2.0 * np.pi * k / N
        s = np.sin(angle)
        q15 = s * 32768.0
        if (q15 == 32768):
            q15 = 32767
        sin_lut_q15[k] = q15
    return sin_lut_q15

if __name__ == "__main__":
    sin_lut_q15 = init_sin_lut_q15()
    with open("sin_lut_q15.bin", "w") as f:
        f.write("{\n")
        for i in range(len(sin_lut_q15)):
            f.write(f"{sin_lut_q15[i]:d},\n")
        f.write("}\n")
    print("sin_lut_q15.bin generated")