"""Diagnostic: why did in-band SNR drop ~2 dB when the fabric clock (and
therefore the fractional-ASRC output rate) was halved?

Bench facts established on hardware:
  * direct DDS tone (fed straight to the DSM)        : ~91 dB
  * same DDS resampled through the ASRC/halfbands     : ~89 dB
  * I2S audio through the ASRC/halfbands              : ~89 dB
  * adding a 3rd halfband (3.375 -> 6.75 MHz DSM in)  : no change
  * disabling dither / disabling servo                : no change

The additive noise of the resampler itself is tiny (the coefficient
file reports a ~-104 dBFS quant floor and ~-130 dBFS phase-interp
spurs), so simple "resampler noise" cannot explain a 2 dB hit to a
-91 dB floor. This script reproduces the signal chain bit-true through
the *real* deployed DSM (dsm_coeffs.vh: order-4, OSR 338, Hinf 1.2)
and measures the in-band SINAD for each path so we can see which stage
actually costs the 2 dB and why.

Chain modelled (matches root.v / asrc.v / halfband_up2.v):

  44.1 kHz sine
     -> fractional_asrc  (256-phase x 64-tap polyphase, real coeffs)
        to Fs_asrc                       (1.6875 MHz "old" | 843.75 kHz "new")
     -> halfband x2 cascade up to 6.75 MHz DSM-input rate
     -> zero-order hold x2 to the 13.5 MHz modulator rate
     -> bit-true CIFB DSM
     -> in-band FFT SINAD

Run:
  python scripts/asrc_rate_snr_experiment.py
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from dsm_model import FixedPointConfig, ModulatorCoeffs, simulate


# --------------------------------------------------------------------- #
# Fixed rates (Hz)
# --------------------------------------------------------------------- #
FS_IN      = 44_100
FS_MOD     = 13_500_000            # modulator / bit rate (same both builds)
FS_DSM_IN  = 6_750_000             # DSM din update rate (same both builds)
FS_ASRC_OLD = 1_687_500            # fractional ASRC output @ 108 MHz build
FS_ASRC_NEW =   843_750            # fractional ASRC output @ 54 MHz build

AMP_FS   = 0.5                     # -6 dBFS
AUDIO_BW = 20_000.0

N_DSM    = 1 << 18                 # modulator samples analysed (FFT window)
N_WARM   = 8192                    # extra samples dropped before analysis
DITHER_PEAK_FS = 0.5               # matches dac.v TPDF sum peak

# Coherent tone: exactly TONE_BINS cycles in the N_DSM-sample FFT window
# so it lands on one FFT bin with zero leakage (~1 kHz).
TONE_BINS = round(1000.0 * N_DSM / FS_MOD)
F_TONE    = TONE_BINS * FS_MOD / N_DSM


# --------------------------------------------------------------------- #
# Deployed DSM coefficients (from src/rtl/dsm_coeffs.vh, Q3.20)
# --------------------------------------------------------------------- #
def deployed_coeffs() -> tuple[ModulatorCoeffs, FixedPointConfig]:
    fp = FixedPointConfig(coeff_w=24, coeff_frac=20, state_w=36, state_frac=31)
    coeffs = ModulatorCoeffs(
        a=[331, 6877, 67934, 381874],
        g=[0, 65],
        b1=165,
        c=[1_048_576, 1_048_576, 1_048_576, 1_048_576],
        order=4,
    )
    return coeffs, fp


# --------------------------------------------------------------------- #
# Fractional polyphase ASRC (float model of fractional_asrc.v)
# --------------------------------------------------------------------- #
def load_polyphase(mem_path: Path) -> np.ndarray:
    """Return the 256 polyphase rows (phase, tap) as float, DC gain 1."""
    vals = []
    for line in mem_path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("//"):
            continue
        v = int(line, 16)
        if v >= (1 << 17):          # 18-bit two's complement
            v -= (1 << 18)
        vals.append(v)
    arr = np.array(vals, dtype=np.float64)
    rows = arr.reshape(-1, 64)      # 257 rows (256 + sentinel)
    banks = rows[:256] / float(1 << 17)   # coeffs sum to 2^17 per phase
    return banks


def fractional_resample(x: np.ndarray, banks: np.ndarray,
                        fs_in: float, fs_out: float,
                        n_out: int) -> np.ndarray:
    """Phase-accumulator polyphase resampler, matching fractional_asrc.v.

    step (Q0.32) = fs_in / fs_out input-samples per output sample.
    """
    taps = banks.shape[1]
    step = int(round((fs_in / fs_out) * (1 << 32)))
    acc = 0
    origin = taps                  # leave history room
    out = np.empty(n_out, dtype=np.float64)
    # sentinel row = phase 0 shifted left by one tap (see coeff generator)
    sent = np.zeros(taps)
    sent[:taps - 1] = banks[0, 1:taps]

    for i in range(n_out):
        phase = (acc >> 24) & 0xFF
        alpha = ((acc >> 8) & 0xFFFF) / float(1 << 16)
        ca = banks[phase]
        cb = banks[phase + 1] if phase + 1 < 256 else sent
        coeff = ca + (cb - ca) * alpha
        window = x[origin - taps + 1: origin + 1][::-1]
        out[i] = np.dot(coeff, window)
        acc += step
        if acc >> 32:
            origin += (acc >> 32)
            acc &= 0xFFFFFFFF
    return out


# --------------------------------------------------------------------- #
# 15-tap halfband x2 interpolator (halfband_up2.v)
# --------------------------------------------------------------------- #
def halfband_up2(x: np.ndarray) -> np.ndarray:
    C0, C2, C4, C6 = -960.0, 4258.0, -18003.0, 80241.0
    h = np.array([C0, 0, C2, 0, C4, 0, C6, (1 << 17),
                  C6, 0, C4, 0, C2, 0, C0], dtype=np.float64) / (1 << 17)
    up = np.zeros(len(x) * 2, dtype=np.float64)
    up[0::2] = x
    y = np.convolve(up, h)          # DC gain 2 compensates zero-insertion
    return y


# --------------------------------------------------------------------- #
# In-band SINAD (tone vs everything else in 0..20 kHz)
# --------------------------------------------------------------------- #
def analyse(v: np.ndarray, fs: float, bw: float, f_tone: float) -> dict:
    """Coherent breakdown: SINAD, harmonic THD, worst non-harmonic spur,
    and residual broadband noise (all in dB relative to the tone)."""
    v = v.astype(np.float64)
    n = len(v)
    spec = np.abs(np.fft.rfft(v)) ** 2   # rectangular: tone is bin-aligned
    freqs = np.fft.rfftfreq(n, d=1.0 / fs)
    band = freqs <= bw
    band[0] = False
    df = fs / n
    k = int(round(f_tone / df))

    def bin_power(centre_bin: int) -> float:
        s = slice(max(1, centre_bin - 1), centre_bin + 2)
        return float(np.sum(spec[s]))

    sig = bin_power(k)
    total = float(np.sum(spec[band]))
    sinad = 10 * np.log10(sig / max(total - sig, 1e-30))

    # Harmonics 2..40 that fall in band.
    harm_p = 0.0
    used = set(range(max(1, k - 1), k + 2))
    for h in range(2, 41):
        kb = k * h
        if freqs[kb] > bw:
            break
        harm_p += bin_power(kb)
        used.update(range(max(1, kb - 1), kb + 2))
    thd = 10 * np.log10(harm_p / sig) if harm_p > 0 else -np.inf

    # Worst non-harmonic in-band spur.
    masked = spec.copy()
    masked[~band] = 0.0
    for b in used:
        if b < len(masked):
            masked[b] = 0.0
    spur_bin = int(np.argmax(masked))
    spur_dbc = 10 * np.log10(masked[spur_bin] / sig) if masked[spur_bin] > 0 else -np.inf
    spur_hz = freqs[spur_bin]

    # Broadband noise = in-band total minus tone minus harmonics.
    noise_p = max(total - sig - harm_p, 1e-30)
    noise_dbc = 10 * np.log10(noise_p / sig)

    return {"sinad": sinad, "thd": thd, "spur_dbc": spur_dbc,
            "spur_hz": spur_hz, "noise_dbc": noise_dbc}


# --------------------------------------------------------------------- #
# Build DSM input for each path, run modulator, measure
# --------------------------------------------------------------------- #
def run_case(label: str, din_6p75: np.ndarray | None,
             coeffs: ModulatorCoeffs, fp: FixedPointConfig,
             direct: np.ndarray | None = None,
             dither: float = DITHER_PEAK_FS) -> None:
    if direct is not None:
        u = direct[:N_DSM + N_WARM]
    else:
        # zero-order hold 6.75 MHz -> 13.5 MHz modulator rate (repeat x2)
        u = np.repeat(din_6p75, 2)[:N_DSM + N_WARM]
    res = simulate(coeffs, fp, u, dither_peak_fs=dither)
    v = res["v"][N_WARM:N_WARM + N_DSM]     # drop warm-up, keep coherent window
    r = analyse(v, FS_MOD, AUDIO_BW, F_TONE)
    sat = " (SAT!)" if res["saturated"] else ""
    print(f"  {label:32s}: SINAD {r['sinad']:6.1f} | THD {r['thd']:6.1f} | "
          f"noise {r['noise_dbc']:6.1f} | spur {r['spur_dbc']:6.1f}dBc "
          f"@{r['spur_hz']:6.0f}Hz{sat}")


def asrc_path(banks: np.ndarray, fs_asrc: float, n_asrc: int,
              n_halfbands: int) -> np.ndarray:
    """44.1k sine -> fractional ASRC -> N halfbands -> 6.75 MHz stream."""
    # Enough 44.1k input to cover the resampler window + output span.
    n_in = int(n_asrc * FS_IN / fs_asrc) + 256
    t_in = np.arange(n_in) / FS_IN
    x = AMP_FS * np.sin(2 * np.pi * F_TONE * t_in)
    y = fractional_resample(x, banks, FS_IN, fs_asrc, n_asrc)
    for _ in range(n_halfbands):
        y = halfband_up2(y)
    # trim filter transients
    return y[64:]


def main() -> None:
    root = Path(__file__).resolve().parent.parent
    mem = root / "fpga" / "toy-dac" / "src" / "rtl" / "frac_asrc.mem"
    banks = load_polyphase(mem)
    coeffs, fp = deployed_coeffs()

    n_dsm_in = (N_DSM + N_WARM) // 2 + 4096   # 6.75 MHz samples needed (+slack)

    print(f"Tone {F_TONE:.0f} Hz @ {20*np.log10(AMP_FS):.1f} dBFS, "
          f"DSM {FS_MOD/1e6:.3f} MHz, band 0..{AUDIO_BW/1e3:.0f} kHz, "
          f"N={N_DSM}\n")

    # 1) direct: ideal sine straight to the DSM at the modulator rate
    t_mod = np.arange(N_DSM + N_WARM) / FS_MOD
    direct = AMP_FS * np.sin(2 * np.pi * F_TONE * t_mod)
    run_case("direct sine 13.5MHz (dither 0.5)", None, coeffs, fp, direct=direct)
    run_case("direct sine 13.5MHz (no dither)", None, coeffs, fp,
             direct=direct, dither=0.0)

    # 2) OLD build: fractional ASRC @ 1.6875 MHz + 2 halfbands -> 6.75 MHz
    n_asrc_old = n_dsm_in // 4 + 64
    old = asrc_path(banks, FS_ASRC_OLD, n_asrc_old, n_halfbands=2)
    run_case("ASRC 1.6875MHz +2HB (108MHz)", old[:n_dsm_in], coeffs, fp)
    run_case("ASRC 1.6875MHz +2HB (no dither)", old[:n_dsm_in], coeffs, fp,
             dither=0.0)

    # 3) NEW build: fractional ASRC @ 843.75 kHz + 3 halfbands -> 6.75 MHz
    n_asrc_new = n_dsm_in // 8 + 64
    new = asrc_path(banks, FS_ASRC_NEW, n_asrc_new, n_halfbands=3)
    run_case("ASRC 843.75kHz +3HB (54MHz)", new[:n_dsm_in], coeffs, fp)
    run_case("ASRC 843.75kHz +3HB (no dither)", new[:n_dsm_in], coeffs, fp,
             dither=0.0)


if __name__ == "__main__":
    main()
