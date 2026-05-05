"""Compare integrated out-of-band noise of two modulator designs.

Useful for diagnosing why a higher-order modulator can show worse THD
on hardware despite better in-band SNR — the increased out-of-band
noise intermodulates in any non-linear analog stage downstream and
folds back as apparent in-band distortion.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from dsm_model import (FixedPointConfig, ModulatorCoeffs, simulate)


def integrated_power(v, fs, f_lo, f_hi):
    """Integrated power of bit stream `v` between f_lo and f_hi (Hz)."""
    n = len(v)
    spec = np.fft.rfft(v.astype(np.float64))
    psd = (np.abs(spec) ** 2) / (n * fs)  # one-sided PSD, V^2/Hz
    f = np.fft.rfftfreq(n, d=1.0 / fs)
    band = (f >= f_lo) & (f <= f_hi)
    return float(np.trapezoid(psd[band], f[band]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mclk-hz', type=float, default=108e6)
    ap.add_argument('--n-samples', type=int, default=1 << 18)
    ap.add_argument('--dc', type=float, default=0.0)
    ap.add_argument('--dither-peak-fs', type=float, default=0.5)
    ap.add_argument('coeffs', nargs='+', type=Path)
    args = ap.parse_args()

    fp = FixedPointConfig()
    u = np.full(args.n_samples, args.dc, dtype=np.float64)

    print(f'{"design":35s} {"P(<20k)":>10s} {"P(20k-1M)":>11s} '
          f'{"P(1M-Nyq)":>11s} {"toggle":>7s}  {"max-run":>7s}')
    print('-' * 100)

    for path in args.coeffs:
        c = ModulatorCoeffs.from_json(path, fp)
        r = simulate(c, fp, u, dither_peak_fs=args.dither_peak_fs)
        v = r['v'].astype(np.float64)
        p_audio = integrated_power(v, args.mclk_hz, 20.0, 20e3)
        p_mid   = integrated_power(v, args.mclk_hz, 20e3, 1e6)
        p_high  = integrated_power(v, args.mclk_hz, 1e6, args.mclk_hz / 2)
        toggle = float(np.mean(np.diff(v) != 0))
        # Longest run of identical bits
        b = (v > 0).astype(np.int8)
        d = np.diff(np.where(np.concatenate(([1], b[:-1] != b[1:], [1])))[0])
        max_run = int(d.max()) if d.size else 0
        print(f'{path.name:35s} {p_audio:10.3e} {p_mid:11.3e} '
              f'{p_high:11.3e} {toggle:7.4f}  {max_run:7d}')


if __name__ == '__main__':
    main()
