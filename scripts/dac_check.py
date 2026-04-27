import numpy as np
import matplotlib.pyplot as plt

# --- load + parse header --------------------------------------------------
hdr = {}
with open('../fpga/toy-dac/toy-dac.sim/sim_1/behav/xsim/dout.bits.txt') as f:
    for line in f:
        if not line.startswith('#'): break
        if '=' in line:
            k, v = line.lstrip('# ').strip().split('=', 1)
            hdr[k] = v
mclk_hz   = float(hdr['mclk_hz'])
tone_hz   = float(hdr['tone_hz'])
k_cycles  = int(hdr['k_cycles'])
N_bits    = int(hdr['capture_cycles'])

bits = np.loadtxt('../fpga/toy-dac/toy-dac.sim/sim_1/behav/xsim/dout.bits.txt', dtype=np.int8, comments='#')
x = bits.astype(np.float64) * 2.0 - 1.0     # ±1

# --- decimate to a sane audio rate ---------------------------------------
# Two-stage box-average so we land at ~96 kHz.
D1 = 12      # 100 MHz / 12  ≈ 8.33 MHz
D2 = 87      # 8.33 MHz / 87 ≈ 95.8 kHz   (D1*D2 = 1044, close to 1024)
# Trim to a multiple of D1*D2 and to a power of 2 in samples for clean FFT
total_dec = D1 * D2
M = (len(x) // total_dec) * total_dec
y = x[:M].reshape(-1, D1).mean(1)
y = y.reshape(-1, D2).mean(1)

fs_dec = mclk_hz / total_dec
print(f"Decimated to {fs_dec/1e3:.2f} kHz, {len(y)} samples "
      f"({len(y)/fs_dec*1e3:.2f} ms)")

# --- coherent FFT ---------------------------------------------------------
# K_CYCLES tone cycles fit in CAPTURE_CYCLES mclk samples, which after
# decimation is len(y) samples *only if* M == N_bits. If we trimmed,
# coherence is slightly off — fine, just use a window.
y -= y.mean()
N = len(y)
w = np.hanning(N)
# Hann coherent gain = 0.5
Y = np.fft.rfft(y * w) / (N * 0.5)
mag_db = 20 * np.log10(np.abs(Y) + 1e-30)
freqs  = np.fft.rfftfreq(N, 1/fs_dec)

# --- find tone + harmonics -----------------------------------------------
def bin_db(f):
    b = int(round(f / fs_dec * N))
    # take peak of ±2 bins to handle Hann skirt
    return mag_db[max(0,b-2):b+3].max(), freqs[b]

f1 = tone_hz
h1_db, _ = bin_db(f1)
print(f"Fundamental {f1:7.2f} Hz : {h1_db:7.2f} dB")
for h in (2, 3, 4, 5, 6, 7):
    hd, hf = bin_db(h * f1)
    print(f"  H{h} ({hf:7.1f} Hz) : {hd:7.2f} dB  ({hd-h1_db:+6.2f} dBc)")

# --- plot, zoomed to audio band ------------------------------------------
plt.figure(figsize=(11,5))
plt.semilogx(freqs[1:], mag_db[1:])
plt.axvline(f1, color='g', alpha=0.4, label=f'F1 = {f1:.1f} Hz')
for h in (3,5,7): plt.axvline(h*f1, color='r', alpha=0.3, ls='--')
plt.xlim(20, fs_dec/2)
plt.ylim(mag_db.min()-5, h1_db+10)
plt.grid(which='both', alpha=0.4)
plt.xlabel('Hz'); plt.ylabel('dBFS'); plt.legend()
plt.title('interp+dac, dout FFT')
plt.tight_layout(); plt.show()