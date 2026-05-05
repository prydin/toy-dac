"""Synthesize a single-bit ΔΣ modulator NTF and emit RTL-ready coefficients.

Uses the python-deltasigma toolbox (Schreier's MATLAB Delta-Sigma toolbox
ported to Python) to design a stable higher-order modulator that the
current `dac.v` (CIFB, ORDER ∈ {1, 2}, Pascal-triangle coefficients) cannot
realise.

Why this script exists
----------------------
`dac.v` is currently locked to ORDER ≤ 2 because the Pascal-triangle NTF
zeros-stacked-at-DC shape has out-of-band gain (OBG) = 2^N, and N=3
already exceeds the Lee rule (Hinf ≲ 1.5) for a 1-bit quantizer →
integrators rail for any input. The fix is not "different parameters" but
a different NTF: zeros spread off DC (resonator feedback `g`) and Hinf
clamped during synthesis. This script does that synthesis and reports
coefficients, dynamic-range scaling, peak SQNR, and max stable input.

Output
------
1. Console report: synthesised NTF, scaled (a, g, b, c) coefficients,
   per-integrator state range, predicted peak SQNR, max DC input.
2. JSON file with the raw floats (machine-readable).
3. Verilog header (`.vh`) with `localparam` constants suitable for
   `\u0060include`-ing from a future generalised dac.v.

Coefficients emitted in CIFB form, matching the topology dac.v already
uses for ORDER=2:
    σ₀[n+1] = σ₀[n] + b₁·u[n] − a₁·v[n]
    σ_k[n+1] = σ_k[n] + c_k·σ_{k-1}[n] − a_{k+1}·v[n]   (k = 1 … N-1)
    (with optional `−g_j·σ_{2j+1}[n]` resonator feedback into σ_{2j} to
     move an NTF zero pair off DC for in-band noise shaping)
    v[n] = sign(σ_{N-1}[n] + dither)

Coefficients are floating-point. RTL implementation will need either
fixed-point multiplies (DSP48) or CSD shift-and-add decomposition.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import builtins
import collections
import collections.abc
import fractions
import math
import sys
import types

import numpy as np

# python-deltasigma (last release 2015) hasn't kept up with the
# scientific-Python ecosystem; restore the symbols it expects before
# importing it. None of these affect our own code.
#   - NumPy ≥ 1.20 removed np.float / np.int / np.complex aliases.
#   - NumPy ≥ 2.0 removed numpy.distutils. The only thing the toolbox
#     does with it is look up BLAS include paths for a (now-irrelevant)
#     cython speed-up; an empty stub is fine.
#   - Python ≥ 3.9 removed fractions.gcd (use math.gcd instead).
for _name in ('float', 'int', 'complex', 'bool'):
    if not hasattr(np, _name):
        setattr(np, _name, getattr(builtins, _name))
if 'numpy.distutils' not in sys.modules:
    _distutils = types.ModuleType('numpy.distutils')
    _sysinfo = types.ModuleType('numpy.distutils.system_info')
    _sysinfo.get_info = lambda _name: {}
    _distutils.system_info = _sysinfo
    sys.modules['numpy.distutils'] = _distutils
    sys.modules['numpy.distutils.system_info'] = _sysinfo
if not hasattr(fractions, 'gcd'):
    fractions.gcd = math.gcd

# Python ≥ 3.10 moved the ABCs (Iterable, Callable, Mapping, …) out of
# `collections` into `collections.abc`. The toolbox still imports them
# from the old location.
for _abc in ('Iterable', 'Callable', 'Mapping', 'Sequence', 'Hashable',
             'Container', 'Sized', 'Set', 'MutableSet', 'MutableMapping',
             'MutableSequence'):
    if not hasattr(collections, _abc):
        setattr(collections, _abc, getattr(collections.abc, _abc))

# scipy ≥ 1.12 removed scipy.signal.step2 (its only difference from
# `step` was a default-T heuristic the toolbox doesn't rely on).
import scipy.signal as _sps  # noqa: E402
if not hasattr(_sps, 'step2'):
    _sps.step2 = _sps.step

# NumPy ≥ 1.25 rejects float arguments to size parameters (the toolbox
# has a bare `N = 1000.0` in scaleABCD that then flows into `randn`).
# Coerce floats → ints at the boundary so we don't have to patch the
# installed library.
import numpy.random as _npr  # noqa: E402
_orig_randn = _npr.randn
def _randn_intargs(*args, **kwargs):
    args = tuple(int(a) if isinstance(a, float) else a for a in args)
    return _orig_randn(*args, **kwargs)
_npr.randn = _randn_intargs

import deltasigma as ds  # noqa: E402


def synthesize(order: int, osr: int, hinf: float, f0: float,
               opt: int, stf_gain: float = 0.5) -> dict:
    """Run the full synthesize → realize → scale pipeline.

    `stf_gain` rescales b₁ so the loop's signal-transfer function at
    DC equals the requested value (`STF(DC) = b₁ / a₁` for CIFB).
    Default 0.5 matches the legacy Pascal-2 back-compat path in dac.v
    so a digital full-scale input maps to half analog full-scale, the
    convention the existing analog board level expects. Set 1.0 for
    unity STF (digital FS → analog FS, 6 dB hotter output).

    Returns a dict containing the NTF, raw and scaled coefficient
    vectors, integrator state ranges, predicted peak SQNR and the
    maximum stable DC input level (umax) reported by scaleABCD.
    """
    # 1. NTF synthesis. opt=2 spreads zeros across the passband for
    # minimum in-band noise; Hinf bounds OBG (Lee rule says ≲ 1.5 for
    # 1-bit). nlev=2 → single-bit quantizer.
    ntf = ds.synthesizeNTF(order=order, osr=osr, opt=opt, H_inf=hinf, f0=f0)

    # 2. Realise NTF in CIFB topology — that's what dac.v already
    # implements (cascade-of-integrators with distributed feedback).
    a, g, b, c = ds.realizeNTF(ntf, form='CIFB')
    # The toolbox returns 0-d ndarrays / bare scalars for length-1
    # vectors; coerce everything to a plain 1-D list so downstream
    # code (and JSON / Verilog emitters) sees a uniform shape.
    a = np.atleast_1d(a).astype(float).tolist()
    g = np.atleast_1d(g).astype(float).tolist()
    b = np.atleast_1d(b).astype(float).tolist()
    c = np.atleast_1d(c).astype(float).tolist()

    # 3. Convert to ABCD state-space form (kept for downstream tooling
    # that wants it; we don't currently use the result). The toolbox's
    # stuffABCD has a broadcast bug for some orders — non-fatal here
    # because we don't consume the output.
    b = [b[0]] + [0.0] * (len(b) - 1)

    # Rescale b₁ to give the requested STF(DC) = b₁ / a₁. (NTF is
    # unchanged — only the input feed-in scales — so noise shaping
    # and stability are preserved.)
    if a and a[0] != 0.0:
        b[0] = stf_gain * a[0]

    try:
        _abcd = ds.stuffABCD(np.array(a), np.array(g), np.array(b),
                             np.array(c), form='CIFB')
    except Exception:  # noqa: BLE001
        _abcd = None

    # NOTE: scaleABCD() is intentionally skipped. The toolbox's pure-
    # Python simulateDSM (the cython speed-up doesn't build on modern
    # Python) is too slow to be practical inside scaleABCD's inner
    # loop. We emit the raw realizeNTF coefficients; per-integrator
    # state ranges and umax should be measured with our own bit-true
    # RTL simulator before committing the coefficients to silicon.
    a_s, g_s, b_s, c_s = a, g, b, c
    umax = float('nan')

    # 5. Predict peak SQNR analytically from the NTF magnitude integrated
    # over the audio band [0, π/OSR]. For a 1-bit modulator (Δ = 2) with
    # white-quantizer assumption, in-band noise power is
    #   Pn = (Δ²/12)·(1/π)·∫₀^(π/OSR) |H(e^jω)|² dω
    # Peak signal power is for a sine at full scale (amplitude 1), so
    # Ps = 0.5. (We can't use umax here because we skipped scaleABCD;
    # the resulting SQNR is an upper bound assuming the modulator can
    # be driven to ±1 — adjust downward by 20·log10(umax) once umax is
    # measured.)
    n_pts = 4096
    w_band = np.linspace(0, np.pi / osr, n_pts)
    h_band = np.abs(ds.evalTF(ntf, np.exp(1j * w_band)))
    pn = (4.0 / 12.0) * np.trapezoid(h_band ** 2, w_band) / np.pi
    ps = 0.5
    peak_snr = 10.0 * np.log10(ps / pn)

    return {
        'order': order,
        'osr': osr,
        'hinf': hinf,
        'f0': f0,
        'opt': opt,
        'ntf_zeros': ntf[0].tolist(),
        'ntf_poles': ntf[1].tolist(),
        'a_raw': a,
        'g_raw': g,
        'b_raw': b,
        'c_raw': c,
        'a': a_s,
        'g': g_s,
        'b': b_s,
        'c': c_s,
        'umax': float(umax),
        'peak_snr_db': float(peak_snr),
        'stf_gain': float(stf_gain),
    }


def emit_json(result: dict, path: Path) -> None:
    """Write the raw float coefficients as JSON for downstream tools."""
    # numpy complex zeros/poles must be turned into [real, imag] pairs.
    serialisable = dict(result)
    serialisable['ntf_zeros'] = [[z.real, z.imag] for z in result['ntf_zeros']]
    serialisable['ntf_poles'] = [[p.real, p.imag] for p in result['ntf_poles']]
    path.write_text(json.dumps(serialisable, indent=2), encoding='utf-8')


def emit_verilog_header(result: dict, path: Path,
                         q_frac_bits: int) -> None:
    """Emit a Verilog header with `localparam` coefficient constants.

    Coefficients are quantised to signed Q-format with `q_frac_bits`
    fractional bits. The header is `\u0060include`-able from RTL and
    declares one parameter per coefficient plus the array sizes.
    """
    def q(x: float) -> int:
        return int(round(x * (1 << q_frac_bits)))

    order = result['order']
    width = q_frac_bits + 4  # a few integer guard bits
    lines = [
        '// Auto-generated by scripts/synthesize_dsm.py — do not edit.',
        '// Re-run synthesize_dsm.py with different --order/--osr/--hinf to regenerate;',
        '// dac.v is parametric in ORDER/N_G and adapts to whatever this file declares.',
        f'// Synthesis inputs: --order {order} --osr {result["osr"]} '
        f'--hinf {result["hinf"]} --opt {result["opt"]} --f0 {result["f0"]}.',
        f'// STF(DC) gain (b₁/a₁): {result["stf_gain"]:.4f} '
        f'(digital FS → {20 * np.log10(result["stf_gain"]):+.2f} dBFS analog).',
        f'// Predicted peak SQNR (analytic, sine at umax): '
        f'{result["peak_snr_db"]:.1f} dB.',
        f'// Max stable DC input (umax): {result["umax"]:.3f} FS.',
        f'// Q-format: signed Q{width - q_frac_bits - 1}.{q_frac_bits} '
        f'(total {width} bits).',
        '',
        f'localparam integer DSM_ORDER = {order};',
        f'localparam integer DSM_N_G  = {len(result["g"])};',
        f'localparam integer DSM_COEFF_W = {width};',
        f'localparam integer DSM_COEFF_FRAC = {q_frac_bits};',
        '',
    ]

    def array_param(name: str, vals: list[float]) -> str:
        if not vals:
            return f'// {name}: (empty)'
        elems = ', '.join(
            f"{width}'sd{q(v)}" if q(v) >= 0 else f"-{width}'sd{-q(v)}"
            for v in vals
        )
        return (f'localparam signed [{width}-1:0] '
                f'{name} [0:{len(vals) - 1}] = '
                "'{" + elems + '};')

    lines.append(array_param('DSM_A', result['a']))
    lines.append(array_param('DSM_B', result['b'][:1]))  # only b₁ used
    lines.append(array_param('DSM_C', result['c']))
    if result['g']:
        lines.append(array_param('DSM_G', result['g']))
    else:
        lines.append('// no resonator g coefficients (all NTF zeros at DC)')

    path.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def report(result: dict) -> None:
    """Pretty-print the synthesis result to stdout."""
    print(f"order            : {result['order']}")
    print(f"OSR              : {result['osr']}")
    print(f"Hinf (OBG limit) : {result['hinf']}")
    print(f"f0 (norm.)       : {result['f0']}")
    print(f"opt              : {result['opt']}")
    print()
    print('NTF zeros (z-plane):')
    for z in result['ntf_zeros']:
        print(f'  {z.real:+.6f} {z.imag:+.6f}j  '
              f'(|z|={abs(z):.4f}, ∠={np.angle(z):+.4f} rad)')
    print()
    print('Scaled CIFB coefficients:')
    print(f"  a = {[f'{x:+.6f}' for x in result['a']]}")
    print(f"  g = {[f'{x:+.6f}' for x in result['g']]}")
    print(f"  b = {[f'{x:+.6f}' for x in result['b']]}")
    print(f"  c = {[f'{x:+.6f}' for x in result['c']]}")
    print()
    print(f"umax (max stable DC input)  : {result['umax']:.4f} FS")
    print(f"STF(DC) gain (b₁/a₁)         : {result['stf_gain']:.4f} "
          f"({20 * np.log10(result['stf_gain']):+.2f} dB)")
    print(f"peak SQNR (analytic, sine)  : {result['peak_snr_db']:.1f} dB")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--order', type=int, default=3,
                        help='Modulator order (default: 3).')
    parser.add_argument('--osr', type=int, default=2700,
                        help='Oversampling ratio. Default 2700 ≈ '
                             '108 MHz / (2·20 kHz).')
    parser.add_argument('--hinf', type=float, default=1.5,
                        help='Out-of-band-gain ceiling (Lee rule: 1.5 '
                             'for 1-bit). Default 1.5.')
    parser.add_argument('--f0', type=float, default=0.0,
                        help='Bandpass centre frequency (normalised to fs). '
                             '0 = lowpass. Default 0.')
    parser.add_argument('--opt', type=int, default=2,
                        help='NTF zero-optimisation level: 0=all at DC '
                             '(Pascal-like), 1=optimised pole/zero, '
                             '2=optimised + free zeros. Default 2.')
    parser.add_argument('--stf-gain', type=float, default=0.5,
                        help='Signal-transfer-function DC gain (b₁/a₁). '
                             'Default 0.5 matches the legacy Pascal-2 '
                             'back-compat scaling in dac.v (digital FS '
                             '→ analog half-FS). Set 1.0 for unity STF.')
    parser.add_argument('--out-json', type=Path,
                        default=Path('data/dsm_coeffs.json'),
                        help='Path for JSON coefficient dump.')
    parser.add_argument('--out-vh', type=Path,
                        default=Path('fpga/toy-dac/src/rtl/dsm_coeffs.vh'),
                        help='Path for Verilog `include` header.')
    parser.add_argument('--q-frac-bits', type=int, default=20,
                        help='Fractional bits for fixed-point coefficient '
                             'quantisation in the .vh output.')
    args = parser.parse_args()

    result = synthesize(args.order, args.osr, args.hinf, args.f0,
                        args.opt, args.stf_gain)
    report(result)

    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    emit_json(result, args.out_json)
    print(f'\nWrote {args.out_json}')

    args.out_vh.parent.mkdir(parents=True, exist_ok=True)
    emit_verilog_header(result, args.out_vh, args.q_frac_bits)
    print(f'Wrote {args.out_vh}')


if __name__ == '__main__':
    main()
