import numpy as np
import scipy.signal as signal
from matplotlib import pyplot as plt

def gen_sine(fs, f, periods):
    n = int(fs*(periods/f))
    t = np.linspace(start=0, stop=periods/f, num=n)
    return t, np.sin(t*f*2*np.pi)

def upsample(t, x, ratio):
    return np.linspace(start=0, stop=np.max(t), num=len(t) * ratio), signal.resample_poly(x, up=ratio, down=1)


def modulate(input_signal):
    """Simple 1st-order Delta-Sigma Modulator"""
    sigma = 0
    output = []
    sigma_history = []
    for x in input_signal:
        # Delta: Difference between input and previous output
        # Sigma: Accumulate the error
        sigma += (x - (1 if sigma >= 0 else -1))
        sigma_history.append(sigma)
        # Quantizer (1-bit)
        bit = 1 if sigma >= 0 else -1
        output.append(bit)
    return np.array(output), sigma_history

def modulate_int(input_signal, bits):
    """Simple 1st-order Delta-Sigma Modulator"""
    sigma = 0
    output = []
    sigma_history = []
    pos = 2**(bits)
    neg = -2**(bits)
    int_input = np.int_(input_signal * (2**bits))
    for x in int_input:
        # Delta: Difference between input and previous output
        # Sigma: Accumulate the error
        sigma += x - (pos if sigma >= 0 else neg)
        sigma_history.append(sigma)
        # Quantizer (1-bit)
        bit = pos if sigma >= 0 else neg
        output.append(bit)
    return np.array(output), sigma_history


fs = 48000
t, in_signal = gen_sine(fs, 1000, 100)
#in_signal = np.sign(in_signal)

# Upsample and filter
upsample_ratio = 100
t, upsampled = upsample(t, in_signal, upsample_ratio)

# Sigma-delta modulate
modulated, sigma = modulate_int(upsampled*0.9, 16)
print(f"Max sigma: {np.max(sigma)}")

# Regeneration filter
cutoff = fs * 2
order = 6
lpf = signal.butter(order, cutoff, fs=fs*upsample_ratio, btype="low", output="sos")
filtered = signal.sosfilt(lpf, modulated)

print(f"original={len(in_signal)}, upsampled={len(upsampled)}, modulated={len(modulated)}")
#plt.plot(t, modulated)
#plt.plot(t, sigma)
plt.plot(t, filtered)
plt.show()

fft = np.fft.fft(filtered)
fft_mag = np.abs(fft)[0:len(fft)//2]

fft_mag /= np.max(fft_mag)
plt.semilogx(range(len(fft_mag)), np.log(fft_mag)*20)
plt.grid(True)
# plt.plot(fft_mag)
plt.show()
