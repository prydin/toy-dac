"""Bit-true behavioural model of the parametric CIFB ΔΣ modulator.

Consumes the coefficient JSON written by `synthesize_dsm.py` and runs the
modulator in fixed-point arithmetic that mirrors what the future
generalised dac.v will do. Reports per-integrator state extrema and the
empirically-measured maximum stable DC input (umax) for the synthesised
NTF — the two quantities `python-deltasigma`'s `scaleABCD` would give
us if it weren't unusably slow on Python ≥ 3.13.

CIFB topology implemented (matches the "what dac.v already does" shape,
generalised to N integrators with optional resonator feedback `g`):

    σ₀[n+1] = σ₀[n] + b₁·u[n]      − a₁·v[n]
    σ_k[n+1] = σ_k[n] + c_k·σ_{k-1}[n]
                       − (g_{k/2}·σ_{k+1}[n] if k even and g pair active)
                       − a_{k+1}·v[n]
    v[n]    = sign(σ_{N-1}[n] + dither)

Fixed-point convention
----------------------
- Coefficients are signed Q(W-F-1).F (taken straight from the .json
  raw floats; we requantise here so the script can sweep different
  fractional-bit settings without rerunning synthesis).
- Integrator state, signal, and product use a wider Q(SI-SF-1).SF
  format with `state_int_bits` integer guard bits above the
  comparator-trip level.
- Quantizer output is ±1·2^SF (so it sits at the same fractional
  scale as the integrators and can be subtracted cleanly).
- All adds/subs are integer; multiplies are integer × integer with
  result right-shifted by F to renormalise back to SF fractional
  bits. This is precisely what the RTL will do (a DSP48 multiply
  followed by a >>F shift, or CSD shift-and-add).

Outputs
-------
- Per-integrator min/max state values across the run.
- A "saturated?" flag (state hit the [-2^(SI-1), 2^(SI-1)-1] limits).
- The measured umax: largest DC input fraction we tried for which
  no integrator saturated and the bit stream remained bounded.
- DC offset and SNR estimate of the bit-stream after a brick-wall
  decimation filter to the audio band.

Run examples
------------
  python scripts/dsm_model.py                                # use defaults
  python scripts/dsm_model.py --coeffs data/dsm_coeffs.json
  python scripts/dsm_model.py --sweep-umax                   # find max stable DC
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np


# --------------------------------------------------------------------- #
# Coefficient quantisation
# --------------------------------------------------------------------- #

@dataclass
class FixedPointConfig:
    """Width parameters for the bit-true model.

    coeff_w / coeff_frac match what `synthesize_dsm.py --q-frac-bits`
    emitted into the .vh. state_w / state_frac is wider — typically
    coeff_frac plus a few integer guard bits above ±1 so the
    integrators have headroom before they wrap.
    """
    coeff_w: int = 24
    coeff_frac: int = 20
    state_w: int = 32
    state_frac: int = 20

    @property
    def state_max(self) -> int:
        return (1 << (self.state_w - 1)) - 1

    @property
    def state_min(self) -> int:
        return -(1 << (self.state_w - 1))

    @property
    def one_q(self) -> int:
        """Integer representation of +1.0 in state Q-format."""
        return 1 << self.state_frac


def quantise(x: float, frac_bits: int) -> int:
    return int(round(x * (1 << frac_bits)))


def quantise_vec(vals: list[float], frac_bits: int) -> list[int]:
    return [quantise(v, frac_bits) for v in vals]


# --------------------------------------------------------------------- #
# Modulator core
# --------------------------------------------------------------------- #

@dataclass
class ModulatorCoeffs:
    """Quantised CIFB coefficients ready for the bit-true loop."""
    a: list[int]
    g: list[int]
    b1: int
    c: list[int]
    order: int

    @classmethod
    def from_json(cls, path: Path, fp: FixedPointConfig) -> 'ModulatorCoeffs':
        data = json.loads(path.read_text(encoding='utf-8'))
        a = quantise_vec(data['a'], fp.coeff_frac)
        g = quantise_vec(data['g'], fp.coeff_frac)
        b = quantise_vec(data['b'], fp.coeff_frac)
        c = quantise_vec(data['c'], fp.coeff_frac)
        return cls(a=a, g=g, b1=b[0], c=c, order=data['order'])


def _mul_q(x: int, coeff: int, coeff_frac: int) -> int:
    """Signed integer multiply, then arithmetic right-shift by coeff_frac.

    Mirrors the DSP48 path: `(x * coeff) >> coeff_frac` with rounding
    truncation (toward −∞), which is what >>> in Verilog does for
    signed values.
    """
    p = x * coeff
    # Python's // rounds toward −∞, matching arithmetic shift right.
    return p >> coeff_frac


def simulate(coeffs: ModulatorCoeffs, fp: FixedPointConfig,
             u: np.ndarray, dither_peak_fs: float = 0.0,
             input_dither_peak_fs: float = 0.0,
             seed: int = 0xCAFEBABE) -> dict:
    """Run the modulator on input vector `u` (floats in [-1, 1]).

    Returns dict with the bit stream `v` (±1), per-integrator state
    history extrema, and a saturation flag.

    `dither_peak_fs` is the TPDF-sum peak in FS units (e.g. 0.5 means
    ±0.5 FS at the comparator). Matches dac.v's quantizer-input dither.

    `input_dither_peak_fs` is an optional second TPDF source injected at
    the input of integrator 0 (added to u before the b1 multiply). Uses
    an independent RNG stream so the two sources are uncorrelated.
    Default 0.0 reproduces the previous behaviour exactly.
    """
    order = coeffs.order
    sigma = [0] * order  # all-zero initial state
    state_min = [0] * order
    state_max = [0] * order
    saturated = False

    n = len(u)
    v_out = np.empty(n, dtype=np.int8)
    one_q = fp.one_q
    smin, smax = fp.state_min, fp.state_max
    cf = fp.coeff_frac

    # Quantise input to state Q-format up front.
    u_q = (u * one_q).round().astype(np.int64)

    # Pre-compute TPDF dither sequence in state Q-format. Sum of two
    # uniform sources, each spanning ±dither_peak_fs/2 → triangular PDF
    # peak ±dither_peak_fs at the comparator. Use a deterministic
    # generator so cross-checks are repeatable.
    if dither_peak_fs > 0.0:
        rng = np.random.default_rng(seed)
        half = dither_peak_fs / 2.0
        d1 = rng.uniform(-half, half, size=n)
        d2 = rng.uniform(-half, half, size=n)
        dither_q = ((d1 + d2) * one_q).round().astype(np.int64)
    else:
        dither_q = np.zeros(n, dtype=np.int64)

    # Optional second TPDF source at the integrator-0 input. Independent
    # RNG stream (seed + 1) so it is uncorrelated with the comparator
    # dither. Folded into u_q so the b1 multiply scales it the same way
    # as the signal — which is how the RTL would inject it if added.
    if input_dither_peak_fs > 0.0:
        rng_in = np.random.default_rng(seed + 1)
        half_in = input_dither_peak_fs / 2.0
        di1 = rng_in.uniform(-half_in, half_in, size=n)
        di2 = rng_in.uniform(-half_in, half_in, size=n)
        u_q = u_q + ((di1 + di2) * one_q).round().astype(np.int64)

    for i in range(n):
        # Quantizer with dither at comparator input.
        v = one_q if (sigma[order - 1] + dither_q[i]) >= 0 else -one_q
        v_out[i] = 1 if v > 0 else -1

        # Compute next state for each integrator. Use a snapshot of the
        # current state so all updates see the same σ[n], not partially
        # updated values (matches non-blocking <= in Verilog).
        sigma_now = list(sigma)

        # Stage 0: σ₀ += b₁·u − a₁·v
        sigma[0] = sigma_now[0] \
            + _mul_q(u_q[i], coeffs.b1, cf) \
            - _mul_q(v, coeffs.a[0], cf)

        # Stages 1 … N-1: σ_k += c_k·σ_{k-1} − a_{k+1}·v
        # plus optional resonator feedback −g_j·σ_{k+1} into σ_k for
        # selected even k (this is how the toolbox places NTF zeros
        # off DC). Pairing convention: g[0] couples (σ_{N-2}, σ_{N-1}),
        # g[1] couples (σ_{N-4}, σ_{N-3}), etc., matching what
        # realizeNTF returns for CIFB.
        for k in range(1, order):
            sigma[k] = sigma_now[k] \
                + _mul_q(sigma_now[k - 1], coeffs.c[k - 1], cf) \
                - _mul_q(v, coeffs.a[k], cf)

        # Apply resonator feedback. Each g[j] acts as
        #   σ_{idx_lo} -= g[j] · σ_{idx_hi}
        # with (idx_lo, idx_hi) = (order-2-2j, order-1-2j).
        for j, gj in enumerate(coeffs.g):
            idx_hi = order - 1 - 2 * j
            idx_lo = idx_hi - 1
            if idx_lo < 0:
                break
            sigma[idx_lo] = sigma[idx_lo] - _mul_q(sigma_now[idx_hi], gj, cf)

        # Track extrema and detect saturation.
        for k in range(order):
            s = sigma[k]
            if s < state_min[k]:
                state_min[k] = s
            if s > state_max[k]:
                state_max[k] = s
            if s <= smin or s >= smax:
                saturated = True
                # Clamp so the run can continue and we still see how
                # bad it got.
                sigma[k] = max(smin, min(smax, s))

    return {
        'v': v_out,
        'state_min': state_min,
        'state_max': state_max,
        'state_min_fs': [s / one_q for s in state_min],
        'state_max_fs': [s / one_q for s in state_max],
        'saturated': saturated,
    }


# --------------------------------------------------------------------- #
# Analysis helpers
# --------------------------------------------------------------------- #

def in_band_snr(v: np.ndarray, osr: int, n_skip: int = 0) -> tuple[float, float]:
    """Estimate SNR of bitstream v using a brick-wall lowpass at fs/(2·OSR).

    Returns (snr_db, dc_offset). DC offset is in units of full scale.
    """
    v = v[n_skip:].astype(np.float64)
    n = len(v)
    # Hann window to suppress spectral leakage of the DC step / sine.
    w = np.hanning(n)
    spec = np.fft.rfft(v * w) / np.sum(w)
    psd = np.abs(spec) ** 2
    # Bin 0 is DC; band edge bin index for [0, fs/(2·OSR)] is n/(2·OSR).
    band_edge = max(1, n // (2 * osr))

    dc_power = psd[0]
    # Treat any peak inside the band as signal, the rest of the band as
    # noise. For a DC input, signal *is* the DC peak.
    signal_power = float(np.max(psd[:band_edge]))
    noise_power = float(np.sum(psd[1:band_edge]) - signal_power)
    if noise_power <= 0:
        return float('inf'), float(np.sqrt(dc_power))
    snr_db = 10.0 * np.log10(signal_power / noise_power)
    dc = float(np.mean(v))
    return snr_db, dc


def sweep_dc_tones(coeffs: ModulatorCoeffs, fp: FixedPointConfig,
                   osr: int,
                   dc_levels: np.ndarray,
                   n_samples: int,
                   dither_peak_fs: float,
                   input_dither_peak_fs: float,
                   warmup: int = 4096,
                   seed: int = 0xCAFEBABE) -> list:
    """Scan DC input levels and characterise idle-tone content.

    For each DC level, run the modulator, drop a warm-up window, then
    take a Hann-windowed FFT of the bitstream and report:
      - peak in-band tone height in dBFS (excluding the DC bin),
      - integrated in-band noise level in dBFS (signal-band sum minus
        the peak-tone bin), and
      - the frequency of the peak tone as a normalised fraction of fs.

    "In-band" is bins (1 .. n/(2*OSR)) of the post-warmup FFT, matching
    the audio band of a downstream OSR decimator.

    Returns a list of dicts, one per input level.
    """
    records = []
    for dc in dc_levels:
        u = np.full(n_samples, float(dc), dtype=np.float64)
        res = simulate(coeffs, fp, u,
                       dither_peak_fs=dither_peak_fs,
                       input_dither_peak_fs=input_dither_peak_fs,
                       seed=seed)
        v = res['v'][warmup:].astype(np.float64)
        # Remove DC. The bitstream mean tracks the input level; left in,
        # Hann window leakage spreads it across the first few bins and
        # dominates the "peak tone" measurement at every non-zero DC.
        v = v - v.mean()
        n = len(v)
        w = np.hanning(n)
        spec = np.fft.rfft(v * w) / np.sum(w)
        psd = np.abs(spec) ** 2
        band_edge = max(2, n // (2 * osr))
        # Skip the DC-leakage zone at the bottom of the band. Hann main-
        # lobe half-width is ~2 bins; a conservative 4-bin skip plus the
        # mean-subtraction above keeps DC out of the tone hunt.
        DC_SKIP = 4
        lo_bin = min(DC_SKIP, band_edge - 1)
        in_band = psd[lo_bin:band_edge]
        if in_band.size == 0:
            peak_dbfs = -np.inf
            peak_bin = 0
            noise_dbfs = -np.inf
        else:
            peak_idx_local = int(np.argmax(in_band))
            peak_bin = peak_idx_local + lo_bin
            peak_power = float(in_band[peak_idx_local])
            # Noise = in-band power (above DC skip) minus the peak tone's
            # main lobe (\u00b13 bins around the peak, Hann main-lobe
            # half-width ~2).
            LOBE = 3
            lo = max(lo_bin, peak_bin - LOBE)
            hi = min(band_edge, peak_bin + LOBE + 1)
            in_band_total = float(np.sum(psd[lo_bin:band_edge]))
            tone_lobe = float(np.sum(psd[lo:hi]))
            noise_power = max(in_band_total - tone_lobe, 1e-30)
            peak_dbfs = 10.0 * np.log10(max(peak_power, 1e-30))
            noise_dbfs = 10.0 * np.log10(noise_power)
        records.append({
            'dc_fs': float(dc),
            'peak_bin': int(peak_bin),
            'peak_f_norm': float(peak_bin) / n,
            'peak_dbfs': float(peak_dbfs),
            'noise_dbfs': float(noise_dbfs),
            'saturated': bool(res['saturated']),
        })
    return records


# --------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------- #

def report_run(label: str, result: dict, fp: FixedPointConfig) -> None:
    print(f'--- {label} ---')
    print(f'  saturated: {result["saturated"]}')
    print('  per-integrator state range (in FS units, ±1 = comparator trip):')
    for k, (lo, hi) in enumerate(zip(result['state_min_fs'],
                                     result['state_max_fs'])):
        guard = (1 << (fp.state_w - 1 - fp.state_frac))
        flag = '  <-- SAT' if (lo <= -guard or hi >= guard - 1) else ''
        print(f'    σ[{k}]: [{lo:+.3f}, {hi:+.3f}]{flag}')


def cmd_dc(coeffs: ModulatorCoeffs, fp: FixedPointConfig,
           dc: float, n: int, osr: int,
           dither_peak_fs: float = 0.0,
           input_dither_peak_fs: float = 0.0) -> dict:
    u = np.full(n, dc, dtype=np.float64)
    result = simulate(coeffs, fp, u, dither_peak_fs=dither_peak_fs,
                      input_dither_peak_fs=input_dither_peak_fs)
    snr, mean = in_band_snr(result['v'], osr, n_skip=n // 8)
    result['snr_db'] = snr
    result['dc_meas'] = mean
    return result


def cmd_sine(coeffs: ModulatorCoeffs, fp: FixedPointConfig,
             amp_fs: float, f_norm: float, n: int, osr: int,
             dither_peak_fs: float = 0.0,
             input_dither_peak_fs: float = 0.0,
             n_harmonics: int = 7) -> dict:
    """Run a sine input and report fundamental/harmonic/noise levels.

    `f_norm` is the input frequency normalised to the modulator clock
    (cycles/sample). Pick something coherent with the FFT length so
    the fundamental lands on a bin.
    """
    # Snap frequency to the nearest integer-bin (coherent FFT).
    k_bin = max(1, int(round(f_norm * n)))
    f_norm_snap = k_bin / n
    t = np.arange(n)
    u = amp_fs * np.sin(2.0 * np.pi * f_norm_snap * t)
    result = simulate(coeffs, fp, u, dither_peak_fs=dither_peak_fs,
                      input_dither_peak_fs=input_dither_peak_fs)

    v = result['v'].astype(np.float64)
    n_skip = n // 8
    v = v[n_skip:]
    nf = len(v)
    # Blackman-Harris 4-term window: peak sidelobe -92 dB, sidelobes
    # fall as 1/k^7. Critical when the fundamental is in a low bin
    # (e.g. 1 kHz at 108 MHz mclk → k_fund ~ 10 even at 1M-pt FFT).
    a = (0.35875, 0.48829, 0.14128, 0.01168)
    nn = np.arange(nf)
    w = (a[0] - a[1]*np.cos(2*np.pi*nn/(nf-1))
              + a[2]*np.cos(4*np.pi*nn/(nf-1))
              - a[3]*np.cos(6*np.pi*nn/(nf-1)))
    spec = np.fft.rfft(v * w) / nf
    psd = np.abs(spec) ** 2
    # Find the fundamental as the largest bin within ±0.1% of the
    # commanded frequency (robust against the truncation shift).
    k_target = max(1, int(round(f_norm_snap * nf)))
    search = max(2, k_target // 1000)
    lo = max(1, k_target - search)
    hi = min(nf // 2, k_target + search + 1)
    k_fund = lo + int(np.argmax(psd[lo:hi]))
    band_edge = max(k_fund * (n_harmonics + 2), nf // (2 * osr))
    band_edge = min(band_edge, nf - 1)

    # Sum a Blackman-Harris main-lobe (±5 bins) for fundamental and
    # each harmonic. BH4 main-lobe half-width is ~4 bins.
    LOBE = 5
    def lobe(k: int) -> float:
        a, b = max(0, k - LOBE), min(nf // 2, k + LOBE + 1)
        return float(np.sum(psd[a:b]))

    fund_p = lobe(k_fund)
    harm_bins = []
    harm_p_total = 0.0
    for h in range(2, n_harmonics + 1):
        kh = k_fund * h
        if kh >= nf // 2:
            break
        p = lobe(kh)
        harm_bins.append((h, kh, p))
        harm_p_total += p
    # Noise = total in-band power minus fundamental and harmonic lobes.
    excluded = set()
    for kc in [k_fund] + [kh for _, kh, _ in harm_bins]:
        excluded.update(range(max(0, kc - LOBE), min(nf // 2 + 1, kc + LOBE + 1)))
    mask = np.ones(band_edge + 1, dtype=bool)
    mask[0] = False
    for b in excluded:
        if 0 <= b <= band_edge:
            mask[b] = False
    noise_p = float(np.sum(psd[:band_edge + 1][mask]))

    result['f_norm'] = f_norm_snap
    result['fund_dbfs'] = 10.0 * np.log10(fund_p) if fund_p > 0 else -np.inf
    result['snr_db'] = (10.0 * np.log10(fund_p / noise_p)
                       if noise_p > 0 else float('inf'))
    result['thd_db'] = (10.0 * np.log10(harm_p_total / fund_p)
                       if harm_p_total > 0 else -np.inf)
    result['thd_pct'] = 100.0 * np.sqrt(harm_p_total / fund_p) if fund_p > 0 else 0.0
    result['harmonics'] = [
        (h, 10.0 * np.log10(p / fund_p) if (p > 0 and fund_p > 0) else -np.inf)
        for h, _, p in harm_bins
    ]
    return result


def cmd_sweep_umax(coeffs: ModulatorCoeffs, fp: FixedPointConfig,
                    n: int, osr: int) -> float:
    """Binary-search for max DC input that keeps every integrator unsaturated."""
    lo, hi = 0.0, 1.0
    best = 0.0
    # First confirm 0 is stable; if not, the design itself is broken.
    r0 = cmd_dc(coeffs, fp, 0.0, n, osr)
    if r0['saturated']:
        print('Modulator is unstable at zero input — coefficients invalid.')
        return 0.0
    # Coarse sweep first to bracket.
    for u in np.linspace(0.05, 0.95, 19):
        r = cmd_dc(coeffs, fp, float(u), n, osr)
        if r['saturated']:
            hi = float(u)
            break
        best = float(u)
        lo = float(u)
    # Bisect.
    for _ in range(8):
        mid = 0.5 * (lo + hi)
        r = cmd_dc(coeffs, fp, mid, n, osr)
        if r['saturated']:
            hi = mid
        else:
            lo = mid
            best = mid
    return best


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--coeffs', type=Path,
                        default=Path('data/dsm_coeffs.json'))
    parser.add_argument('--state-w', type=int, default=32)
    parser.add_argument('--state-frac', type=int, default=20)
    parser.add_argument('--coeff-w', type=int, default=24)
    parser.add_argument('--coeff-frac', type=int, default=20)
    parser.add_argument('--n-samples', type=int, default=200_000,
                        help='Length of test input (samples at modulator rate).')
    parser.add_argument('--dc', type=float, default=0.5,
                        help='DC input level (fraction of FS) for the single-'
                             'point run.')
    parser.add_argument('--sweep-umax', action='store_true',
                        help='Binary-search for max stable DC input.')
    parser.add_argument('--sine-amp', type=float, default=None,
                        help='If set, run a sine input at this amplitude (FS) '
                             'and report SNR/THD instead of a DC run.')
    parser.add_argument('--sine-freq-hz', type=float, default=1000.0,
                        help='Sine frequency in Hz (used with --sine-amp).')
    parser.add_argument('--mclk-hz', type=float, default=108e6,
                        help='Modulator sample clock in Hz.')
    parser.add_argument('--dither-peak-fs', type=float, default=0.5,
                        help='TPDF dither peak in FS units at the comparator '
                             '(matches dac.v half-LSB-peak default = 0.5). '
                             'Set 0 to disable.')
    parser.add_argument('--input-dither-peak-fs', type=float, default=0.0,
                        help='Optional second TPDF dither at the integrator-0 '
                             'input. Default 0 reproduces current dac.v.')
    parser.add_argument('--sweep-tones', action='store_true',
                        help='Scan DC input levels and report peak in-band '
                             'tone height per level (idle-tone diagnostic).')
    parser.add_argument('--sweep-start', type=float, default=0.0,
                        help='Start of DC sweep, in FS units.')
    parser.add_argument('--sweep-stop', type=float, default=0.3,
                        help='End of DC sweep (inclusive), in FS units.')
    parser.add_argument('--sweep-step', type=float, default=0.001,
                        help='Step size of DC sweep, in FS units.')
    parser.add_argument('--sweep-samples', type=int, default=1 << 17,
                        help='Samples per DC level in sweep (default 131072).')
    parser.add_argument('--sweep-warmup', type=int, default=4096,
                        help='Warm-up samples dropped before FFT per level.')
    parser.add_argument('--sweep-output', type=Path, default=None,
                        help='Optional CSV path to dump full sweep records.')
    args = parser.parse_args()

    fp = FixedPointConfig(coeff_w=args.coeff_w, coeff_frac=args.coeff_frac,
                          state_w=args.state_w, state_frac=args.state_frac)
    coeffs = ModulatorCoeffs.from_json(args.coeffs, fp)
    osr = json.loads(args.coeffs.read_text(encoding='utf-8'))['osr']

    print(f'Loaded coefficients: order={coeffs.order}, OSR={osr}')
    print(f'Fixed-point: coeff Q{fp.coeff_w - fp.coeff_frac - 1}.{fp.coeff_frac}, '
          f'state Q{fp.state_w - fp.state_frac - 1}.{fp.state_frac}')
    print(f'  a = {coeffs.a}')
    print(f'  g = {coeffs.g}')
    print(f'  b₁ = {coeffs.b1}')
    print(f'  c = {coeffs.c}')
    print()

    if args.sweep_umax:
        umax = cmd_sweep_umax(coeffs, fp, args.n_samples, osr)
        print(f'\nMeasured umax (max stable DC input): {umax:.3f} FS')
    elif args.sweep_tones:
        n_steps = int(round((args.sweep_stop - args.sweep_start)
                            / args.sweep_step)) + 1
        dc_levels = args.sweep_start + args.sweep_step * np.arange(n_steps)
        print(f'Sweeping {n_steps} DC levels from {dc_levels[0]:+.4f} to '
              f'{dc_levels[-1]:+.4f} FS '
              f'(comparator dither = {args.dither_peak_fs:.2f} FS, '
              f'input dither = {args.input_dither_peak_fs:.2f} FS, '
              f'{args.sweep_samples} samples/level)')
        recs = sweep_dc_tones(coeffs, fp, osr, dc_levels,
                              n_samples=args.sweep_samples,
                              dither_peak_fs=args.dither_peak_fs,
                              input_dither_peak_fs=args.input_dither_peak_fs,
                              warmup=args.sweep_warmup)
        # Compact table.
        print()
        print(f'  {"dc_fs":>9}  {"peak_dBFS":>10}  {"f_norm":>10}  '
              f'{"noise_dBFS":>11}  sat')
        for r in recs:
            print(f'  {r["dc_fs"]:+9.4f}  {r["peak_dbfs"]:10.2f}  '
                  f'{r["peak_f_norm"]:10.6f}  {r["noise_dbfs"]:11.2f}  '
                  f'{"Y" if r["saturated"] else "."}')
        worst = max(recs, key=lambda r: r['peak_dbfs'])
        print(f'\n  worst peak tone: {worst["peak_dbfs"]:.2f} dBFS at '
              f'DC = {worst["dc_fs"]:+.4f} FS '
              f'(f_norm = {worst["peak_f_norm"]:.6f})')
        if args.sweep_output is not None:
            import csv
            with args.sweep_output.open('w', newline='', encoding='utf-8') as fh:
                w = csv.DictWriter(fh, fieldnames=list(recs[0].keys()))
                w.writeheader()
                w.writerows(recs)
            print(f'  wrote {args.sweep_output}')
    elif args.sine_amp is not None:
        f_norm = args.sine_freq_hz / args.mclk_hz
        result = cmd_sine(coeffs, fp, args.sine_amp, f_norm,
                          args.n_samples, osr,
                          dither_peak_fs=args.dither_peak_fs,
                          input_dither_peak_fs=args.input_dither_peak_fs)
        report_run(f'sine {args.sine_amp:.3f} FS @ {args.sine_freq_hz:.1f} Hz '
                   f'(dither peak = {args.dither_peak_fs:.2f} FS)',
                   result, fp)
        print(f'  fundamental : {result["fund_dbfs"]:+.2f} dBFS')
        print(f'  in-band SNR : {result["snr_db"]:.1f} dB')
        print(f'  THD         : {result["thd_db"]:+.1f} dBc  '
              f'({result["thd_pct"]:.4f} %)')   
        for h, lvl_db in result['harmonics']:
            print(f'    H{h}: {lvl_db:+6.1f} dBc')
    else:
        result = cmd_dc(coeffs, fp, args.dc, args.n_samples, osr,
                        dither_peak_fs=args.dither_peak_fs,
                        input_dither_peak_fs=args.input_dither_peak_fs)
        report_run(f'DC = {args.dc:+.3f} FS '
                   f'(dither peak = {args.dither_peak_fs:.2f} FS)',
                   result, fp)
        print(f'  in-band SNR: {result["snr_db"]:.1f} dB')
        print(f'  measured DC: {result["dc_meas"]:+.6f} FS '
              f'(target {args.dc:+.6f}, error '
              f'{(result["dc_meas"] - args.dc) * 1e6:+.0f} ppm of FS)')


if __name__ == '__main__':
    main()
  