"""
analyze_idle_tones.py
─────────────────────
Post-process the bitstream dumps from dac_idle_tone_tb.v.

For each `idle_*.bin` file:
  * read the raw 1-bit modulator output (one byte per cycle, 0x00 or 0x01)
  * map to ±1
  * decimate by an integer factor with an FIR low-pass anti-alias filter,
    bringing the rate from 108 MHz down to ~108 kHz (still well above
    the 20 kHz audio band, but small enough to FFT in seconds)
  * compute power spectrum
  * print the strongest discrete spurs in the audio band (20 Hz – 20 kHz)
    and the broadband audio-band noise level for reference
  * optionally save a PNG plot

Usage:
    python analyze_idle_tones.py                # process every idle_*.bin in cwd
    python analyze_idle_tones.py --plot         # also save PNG plots
    python analyze_idle_tones.py --dir <path>   # work in a different directory

Requires: numpy, scipy, matplotlib (matplotlib only if --plot).
"""

from __future__ import annotations

import argparse
import os
import sys
from glob import glob

import numpy as np
from scipy import signal


MCLK_HZ          = 108_000_000
DECIM            = 1024            # 108 MHz -> 105.46875 kHz
AUDIO_BAND_HI_HZ = 20_000
AUDIO_BAND_LO_HZ = 20
TOP_N_SPURS      = 8


def load_bitstream(path: str) -> np.ndarray:
    """Load a .bin file produced by dac_idle_tone_tb.v.

    Each byte is 0x00 or 0x01. Returned array is float32 in {-1, +1}.
    """
    raw = np.fromfile(path, dtype=np.uint8)
    if raw.size == 0:
        raise ValueError(f"empty file: {path}")
    return (raw.astype(np.float32) * 2.0) - 1.0


def decimate_lp(x: np.ndarray, factor: int) -> np.ndarray:
    """Decimate by `factor` with a long FIR LP, in chunks to limit memory."""
    # scipy.signal.decimate with an FIR is fine but uses default ntaps=30,
    # which has poor stop-band rejection at this decimation. Build our own.
    ntaps = 8 * factor + 1                   # ~8k taps for factor=1024
    cutoff = 0.45 / factor                   # normalised to Nyquist of input
    fir = signal.firwin(ntaps, cutoff, window="hamming")
    # Use polyphase decimation for speed.
    return signal.upfirdn(fir, x, up=1, down=factor)


def power_spectrum_db(y: np.ndarray, fs: float):
    """Single-sided power spectrum in dB relative to ±1 full-scale."""
    n = len(y)
    # Hann window for clean main lobe; correct for window energy.
    win = np.hanning(n)
    Y = np.fft.rfft(y * win)
    # window normalisation (coherent gain for a sine = sum(win)/n = 0.5)
    Y /= (np.sum(win) / 2.0)
    psd = 20.0 * np.log10(np.abs(Y) + 1e-30)
    f   = np.fft.rfftfreq(n, d=1.0 / fs)
    return f, psd


def find_audio_band_spurs(f: np.ndarray, psd_db: np.ndarray):
    """Return list of (freq_hz, level_db) for the loudest peaks in 20 Hz–20 kHz."""
    band = (f >= AUDIO_BAND_LO_HZ) & (f <= AUDIO_BAND_HI_HZ)
    f_b   = f[band]
    psd_b = psd_db[band]
    # Peak picking with a minimum local prominence so we ignore floor wiggles.
    peaks, props = signal.find_peaks(psd_b, prominence=10.0)
    if peaks.size == 0:
        return []
    top = np.argsort(props["prominences"])[::-1][:TOP_N_SPURS]
    return [(float(f_b[p]), float(psd_b[p])) for p in peaks[top]]


def audio_band_noise_db(f: np.ndarray, psd_db: np.ndarray, exclude: list):
    """Mean PSD level (dB) in the audio band, excluding ±50 Hz around any
    listed peak frequency. Crude but enough to compare runs."""
    band = (f >= AUDIO_BAND_LO_HZ) & (f <= AUDIO_BAND_HI_HZ)
    keep = band.copy()
    for f0, _ in exclude:
        near = (f >= f0 - 50.0) & (f <= f0 + 50.0)
        keep &= ~near
    if not keep.any():
        return float("nan")
    # mean of magnitude (linear), then convert back, to avoid the
    # log-mean fooling us about a few isolated bins.
    lin = 10 ** (psd_db[keep] / 10.0)
    return 10.0 * np.log10(lin.mean() + 1e-30)


def analyze_sine(f, psd_db, fs, n_samples):
    """Treat the input as a single sine. Find the fundamental as the
    largest peak in the audio band, then sum power in narrow bins around
    each integer harmonic to compute THD. Report SNR (signal vs all
    audio-band noise excluding fundamental + harmonic bins)."""
    band = (f >= AUDIO_BAND_LO_HZ) & (f <= AUDIO_BAND_HI_HZ)
    f_b   = f[band]
    psd_b = psd_db[band]
    if psd_b.size == 0:
        return None
    # Fundamental = highest peak
    i0 = int(np.argmax(psd_b))
    f0 = float(f_b[i0])
    fund_db = float(psd_b[i0])

    # Bin width and search half-width: be generous to capture window leakage.
    df = fs / n_samples
    half_bins = max(3, int(round(15.0 / df)))   # ±15 Hz minimum

    def peak_near(target_hz):
        idx = int(round(target_hz / df))
        lo = max(0, idx - half_bins)
        hi = min(len(psd_db) - 1, idx + half_bins)
        seg = psd_db[lo:hi + 1]
        if seg.size == 0:
            return None, None
        k = int(np.argmax(seg))
        return float(f[lo + k]), float(seg[k])

    harmonics = []
    for n in range(2, 11):
        fh = n * f0
        if fh > AUDIO_BAND_HI_HZ:
            break
        fh_meas, lvl = peak_near(fh)
        if fh_meas is not None:
            harmonics.append((n, fh_meas, lvl))

    # THD = sqrt(sum of harmonic powers) / fundamental power, in dB
    if harmonics:
        h_lin = sum(10 ** (h[2] / 10.0) for h in harmonics)
        thd_db = 10 * np.log10(h_lin) - fund_db
    else:
        thd_db = float("-inf")

    # SNR: sum noise power in audio band, excluding fundamental and harmonic bins.
    excl = [(f0, fund_db)] + [(h[1], h[2]) for h in harmonics]
    keep = band.copy()
    for fc, _ in excl:
        near = (f >= fc - 30.0) & (f <= fc + 30.0)
        keep &= ~near
    noise_lin = (10 ** (psd_db[keep] / 10.0)).sum()
    noise_db_total = 10 * np.log10(noise_lin + 1e-30)
    snr_db = fund_db - noise_db_total

    return {
        "f0": f0, "fund_db": fund_db,
        "harmonics": harmonics,
        "thd_db": thd_db,
        "snr_db": snr_db,
        "noise_total_db": noise_db_total,
    }


def analyze_one(path: str, do_plot: bool):
    name = os.path.basename(path)
    x = load_bitstream(path)
    print(f"\n=== {name}  ({x.size:,} samples = "
          f"{x.size / MCLK_HZ * 1e3:.1f} ms) ===")

    y_dec = decimate_lp(x, DECIM)
    fs    = MCLK_HZ / DECIM

    # Remove DC; we are looking for tones, not the signal mean.
    y_dec = y_dec - y_dec.mean()

    f, psd_db = power_spectrum_db(y_dec, fs)

    is_sine = name.startswith("sine_")
    spurs = []
    sine_info = None
    if is_sine:
        sine_info = analyze_sine(f, psd_db, fs, len(y_dec))
        if sine_info is not None:
            print(f"  fundamental : {sine_info['f0']:8.2f} Hz  {sine_info['fund_db']:7.2f} dBFS")
            for n, fh, lvl in sine_info["harmonics"]:
                rel = lvl - sine_info["fund_db"]
                print(f"  H{n:<2d}        : {fh:8.2f} Hz  {lvl:7.2f} dBFS  ({rel:+6.2f} dBc)")
            print(f"  THD         : {sine_info['thd_db']:7.2f} dBc")
            print(f"  SNR (audio) : {sine_info['snr_db']:7.2f} dB")
            spurs = [(sine_info["f0"], sine_info["fund_db"])] + \
                    [(h[1], h[2]) for h in sine_info["harmonics"]]
    else:
        spurs = find_audio_band_spurs(f, psd_db)
        if spurs:
            print(f"  top {len(spurs)} audio-band spur(s):")
            for fhz, lvl in sorted(spurs, key=lambda t: -t[1]):
                print(f"     {fhz:8.1f} Hz   {lvl:7.2f} dBFS")
        else:
            print("  no discrete spurs found above prominence threshold")

        floor_db = audio_band_noise_db(f, psd_db, spurs)
        print(f"  audio-band noise floor (mean): {floor_db:7.2f} dBFS")

    if do_plot:
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print("  (matplotlib not installed, skipping plot)")
            return
        band = (f >= 1.0) & (f <= AUDIO_BAND_HI_HZ * 2)
        plt.figure(figsize=(10, 5))
        plt.semilogx(f[band], psd_db[band], linewidth=0.6)
        plt.axvspan(AUDIO_BAND_LO_HZ, AUDIO_BAND_HI_HZ,
                    color="orange", alpha=0.1, label="audio band")
        for fhz, lvl in spurs:
            plt.plot(fhz, lvl, "rv", markersize=8)
            plt.text(fhz, lvl + 1, f"{fhz:.0f} Hz", fontsize=8,
                     ha="center", color="red")
        plt.title(name)
        plt.xlabel("Frequency (Hz)")
        plt.ylabel("PSD (dBFS)")
        plt.grid(True, which="both", alpha=0.3)
        plt.tight_layout()
        out_png = os.path.splitext(path)[0] + ".png"
        plt.savefig(out_png, dpi=120)
        plt.close()
        print(f"  plot -> {out_png}")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dir", default=".",
                    help="directory containing idle_*.bin (default: cwd)")
    ap.add_argument("--plot", action="store_true",
                    help="save a PNG of each file's spectrum")
    args = ap.parse_args()

    patterns = [os.path.join(args.dir, p) for p in ("idle_*.bin", "sine_*.bin")]
    files = sorted({p for pat in patterns for p in glob(pat)})
    if not files:
        print(f"no files matching idle_*.bin or sine_*.bin in {args.dir}",
              file=sys.stderr)
        sys.exit(1)

    print(f"analysing {len(files)} bitstream dump(s) "
          f"(decim {DECIM}x -> {MCLK_HZ / DECIM:.1f} Hz)")
    for path in files:
        try:
            analyze_one(path, args.plot)
        except Exception as exc:                             # noqa: BLE001
            print(f"  ERROR processing {path}: {exc}", file=sys.stderr)


if __name__ == "__main__":
    main()
