"""Polyphase coefficient generator for the fractional-phase ASRC.

Designs a single Kaiser-windowed-sinc lowpass prototype of length
PHASES * TAPS_PER_PHASE samples in the "dense" time grid (PHASES taps
per input sample), decomposes it into PHASES polyphase sub-filters,
quantises to 18-bit signed, and writes a `$readmemh`-friendly .mem
file with row-major layout plus a phase-0-shifted sentinel row at
the end.

Why one bank serves both 44.1 kHz and 48 kHz inputs
---------------------------------------------------
The polyphase filter operates on the input-sample timebase: the
fractional-phase ASRC reads h(t) where t is in units of input samples.
The cutoff target is Fs_in/2 (anti-image). In the dense-domain
(PHASES*Fs_in) that's normalised cutoff = 1/PHASES — independent of
Fs_in. So the same coefficient table works for any input rate, with
the step constant carrying the rate information.

Layout in the .mem file
-----------------------
Row major, 18-bit signed two's complement, one hex value per line
(5 hex digits each, since 18 bits fit in 5 nibbles).

  row 0   : phase  0 taps[0..TAPS-1]
  row 1   : phase  1 taps[0..TAPS-1]
  ...
  row 255 : phase 255 taps[0..TAPS-1]
  row 256 : phase   0 taps[1..TAPS-1, 0]   ← sentinel (poly[0] left-
                                            shifted by one tap; new
                                            last tap is zero)

The sentinel lets the lerp engine read phase i and phase i+1 without
a wrap-around branch when i = PHASES-1. The mathematically correct
neighbour for phase 255 is "phase 0 of the *next* input sample". When
applied to the SAME sample window [origin..origin-T+1] used by the
primary phase, this is equivalent to phase-0 coefficients with tap
indices shifted by +1 — i.e. tap j of the sentinel is poly[0][j+1]
for j < T-1, and tap (T-1) is zero. Without this shift the lerp at
the wrap boundary blends two filters at the same window position and
introduces error proportional to alpha * input-slope (~−26 dBFS for
a half-FS 1 kHz tone), visible as periodic spurs in the output.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np


PHASES_DEFAULT = 256
TAPS_DEFAULT = 64
COEFF_BITS_DEFAULT = 18
ATTEN_DEFAULT_DB = 100.0
# With N=8192 (256*32) dense taps and beta for 100 dB, the Kaiser
# transition width is ~7.8e-4 of dense Nyquist, i.e. ~20% of the 1/256
# image edge. Center the transition so its high edge sits at the image:
# cutoff = 1/L - Δ/2  ≈  0.9/L. This keeps the first input image at
# Fs_in - f_pass safely inside the stopband.
CUTOFF_FRAC_DEFAULT = 0.9


# ---------------------------------------------------------------------------
# Filter design
# ---------------------------------------------------------------------------

def kaiser_beta(attenuation_db: float) -> float:
    """Kaiser window beta for desired stopband attenuation."""
    if attenuation_db > 50:
        return 0.1102 * (attenuation_db - 8.7)
    if attenuation_db >= 21:
        return 0.5842 * (attenuation_db - 21) ** 0.4 + 0.07886 * (attenuation_db - 21)
    return 0.0


def design_prototype(phases: int, taps_per_phase: int,
                     attenuation_db: float,
                     cutoff_frac: float = 1.0) -> np.ndarray:
    """Design the dense-domain lowpass prototype.

    Parameters
    ----------
    phases : int
        Number of polyphase phases (= dense-rate / input-rate).
    taps_per_phase : int
        Filter taps per output sample after polyphase decomposition.
    attenuation_db : float
        Target stopband attenuation (drives the Kaiser beta).
    cutoff_frac : float
        Cutoff as a fraction of Fs_in/2, default 1.0 (cutoff = Fs_in/2).
        Use slightly below 1.0 (e.g. 0.91 for 20 kHz / 22.05 kHz at 44.1k)
        if you want a guard band before the image edge.

    Returns
    -------
    h : np.ndarray, shape (phases * taps_per_phase,)
        Floating-point prototype, normalised so each polyphase phase
        has unity DC gain (sum-to-1 per phase).
    """
    n_total = phases * taps_per_phase
    beta = kaiser_beta(attenuation_db)
    window = np.kaiser(n_total, beta)

    # Cutoff in normalised dense-Nyquist units. Fs_in/2 maps to 1/phases.
    fc = cutoff_frac / phases
    n = np.arange(n_total)
    mid = (n_total - 1) / 2.0
    arg = fc * (n - mid)
    h = np.where(arg == 0.0, fc, np.sin(np.pi * arg) / (np.pi * arg))
    h *= window

    # Per-phase normalisation: each polyphase row should have unity DC
    # gain so that the resampled output preserves signal level.
    # Sum of all dense taps = phases * (per-phase DC gain), so:
    h *= phases / np.sum(h)
    return h


def polyphase_decompose(h: np.ndarray, phases: int) -> np.ndarray:
    """Reshape a dense prototype into shape (phases, taps_per_phase).

    Phase k's k-th tap is h[k + j*phases] for j = 0..taps_per_phase-1.

    Convention: row k is the FIR applied at phase k. Tap 0 is the
    most-recent input sample, tap (taps-1) the oldest.
    """
    taps_per_phase = len(h) // phases
    # h is laid out [phase0_tap0, phase1_tap0, ..., phaseL-1_tap0,
    #                phase0_tap1, phase1_tap1, ...]
    # Reshape (taps, phases) then transpose to (phases, taps).
    return h[: phases * taps_per_phase].reshape(taps_per_phase, phases).T


# ---------------------------------------------------------------------------
# Quantisation
# ---------------------------------------------------------------------------

def quantise(banks: np.ndarray, coeff_bits: int) -> np.ndarray:
    """Scale and round float coefficients to signed two's-complement ints.

    Scale is chosen so that each polyphase phase (which has unity DC
    gain in the float domain) sums to exactly 2^(B-1) in fixed-point.
    The DUT's right-shift by COEFF_W-1 then maps a unity DC input back
    to unity DC output. Peak coefficients land at ~0.9 * 2^(B-1), so we
    give up ~10% headroom in exchange for a hardware-exact unity DC
    gain (no per-channel multiplier needed downstream).
    """
    if banks.size == 0 or np.max(np.abs(banks)) == 0:
        return np.zeros_like(banks, dtype=np.int64)
    scale = float(1 << (coeff_bits - 1))
    q = np.round(banks * scale).astype(np.int64)
    lo, hi = -(2 ** (coeff_bits - 1)), 2 ** (coeff_bits - 1) - 1
    return np.clip(q, lo, hi)


def to_twos_complement_hex(value: int, bits: int) -> str:
    """Format a signed integer as a fixed-width unsigned hex string."""
    if value < 0:
        value = (1 << bits) + value
    nibbles = (bits + 3) // 4
    return f"{value:0{nibbles}x}"


# ---------------------------------------------------------------------------
# .mem file emission
# ---------------------------------------------------------------------------

def write_mem(path: Path, banks_q: np.ndarray, coeff_bits: int,
              header_lines: list[str]) -> None:
    """Emit a $readmemh-compatible .mem with sentinel row appended.

    banks_q has shape (phases, taps_per_phase). Output row order:
    phase 0, phase 1, ..., phase (phases-1), phase 0_shifted (sentinel).

    The sentinel row represents "phase 0 of the NEXT input sample" so
    that lerping between phase (phases-1) and the sentinel at the wrap
    boundary gives an output continuous with the next-sample
    convolution. Concretely, when convolved with the SAME sample window
    [origin..origin-T+1] used by the primary phase, the sentinel must
    behave as if poly[0] were applied to the SHIFTED window
    [origin+1..origin-T+2]. That is equivalent to taking poly[0]'s tap
    indices shifted by +1 (sentinel[j] = poly[0][j+1]), with the new
    last tap (j = T-1) zeroed (no source dense-tap exists past the
    filter's right edge).
    """
    phases, taps = banks_q.shape
    sentinel = np.zeros((1, taps), dtype=banks_q.dtype)
    sentinel[0, :taps - 1] = banks_q[0, 1:taps]      # left-shift phase 0 by one tap
    rows = np.concatenate([banks_q, sentinel], axis=0)

    with path.open("w") as f:
        for line in header_lines:
            f.write(f"// {line}\n")
        f.write(f"// Phases: {phases} (+1 sentinel), taps/phase: {taps}, "
                f"coeff width: {coeff_bits} bits signed two's complement\n")
        f.write(f"// Total entries: {rows.size}\n")
        for phase_idx, row in enumerate(rows):
            f.write(f"// --- phase {phase_idx}"
                    f"{' (sentinel = phase 0)' if phase_idx == phases else ''} ---\n")
            for v in row:
                f.write(to_twos_complement_hex(int(v), coeff_bits) + "\n")


# ---------------------------------------------------------------------------
# Verification (Python golden model)
# ---------------------------------------------------------------------------

def verify_passband_stopband(h: np.ndarray, phases: int,
                             coeff_bits: int) -> dict:
    """Measure achieved passband ripple and stopband attenuation.

    Evaluates the *dense-domain* prototype as a continuous LPF on the
    [0, dense_Nyquist] axis. Cutoff target was Fs_in/2 = dense_Nyq /
    phases.
    """
    # Compute frequency response at high resolution.
    n_fft = 1 << 15
    H = np.fft.rfft(h, n_fft)
    mag_db = 20.0 * np.log10(np.maximum(np.abs(H), 1e-30))
    freqs_norm = np.linspace(0.0, 1.0, mag_db.size)  # 0 = DC, 1 = Nyq

    # Passband: 0..0.8/phases (matches a typical 20 kHz audio band
    # for Fs_in = 44.1k where Fs_in/2 = 22.05 kHz). Stopband: starts at
    # 1.0/phases, the first image edge.
    pb_edge = 0.8 / phases
    sb_edge = 1.0 / phases
    pb_mask = freqs_norm <= pb_edge
    sb_mask = freqs_norm >= sb_edge

    # Normalise to DC gain so passband ripple / stopband attenuation
    # are meaningful (per-phase normalisation gives DC gain = phases
    # for the dense prototype).
    dc_db = mag_db[0]
    norm_db = mag_db - dc_db

    pb_ripple = float(norm_db[pb_mask].max() - norm_db[pb_mask].min())
    sb_atten = float(-norm_db[sb_mask].max())  # attenuation is positive

    # Quantisation noise floor estimate: 6.02 * (B-1) - 1.76 dB
    qn_floor_dbfs = -(6.02 * (coeff_bits - 1) + 1.76)

    return {
        "passband_edge_norm": pb_edge,
        "stopband_edge_norm": sb_edge,
        "passband_ripple_db": pb_ripple,
        "stopband_atten_db": sb_atten,
        "quant_noise_floor_dbfs": qn_floor_dbfs,
    }


def simulate_resample(banks_q: np.ndarray, coeff_bits: int,
                      step: int, n_in: int, sig_freq_norm: float) -> np.ndarray:
    """Bit-true-ish polyphase + lerp resampling of a sine input.

    sig_freq_norm: input frequency as fraction of Fs_in (0..0.5).
    step: phase-accumulator step (Q0.32, scaled so 2^32 = one input sample).
    n_in: number of input samples to feed.

    Returns the resampled output (float, normalised back to ±1).
    """
    phases, taps = banks_q.shape
    # Build input ring of size = taps + headroom.
    x = np.sin(2.0 * np.pi * sig_freq_norm * np.arange(n_in)).astype(np.float64)
    x_int = np.round(x * (2 ** 23 - 1)).astype(np.int64)  # 24-bit-ish

    # Sentinel row appended for lerp.
    rows = np.concatenate([banks_q, banks_q[0:1, :]], axis=0)

    coeff_max = (1 << (coeff_bits - 1)) - 1

    out = []
    phase_acc = np.uint64(0)
    rptr = taps - 1  # warmup: need `taps` samples of history before first MAC
    while rptr + 1 < n_in:
        idx = int(phase_acc >> np.uint64(24))            # top 8 bits
        alpha = int((phase_acc >> np.uint64(16)) & np.uint64(0xff))  # next 8 bits

        c0 = rows[idx]
        c1 = rows[idx + 1]
        # Linear blend between phase i and phase i+1.
        # c_blended = c0 + ((c1 - c0) * alpha) >> 8  (matches RTL intent)
        blended = c0 + ((c1 - c0) * alpha) // 256

        # Newest-first window: w[0] = x[rptr] (current), w[K-1] = oldest.
        # phase[k][j] is the coefficient on x[m-j], so dot products line up.
        window = x_int[rptr - taps + 1: rptr + 1][::-1]
        # acc is Q(coeff_bits-1) * Q23 → ~Q(coeff_bits+22). Normalise.
        acc = int(np.dot(window, blended))
        out.append(acc / (coeff_max * (2 ** 23 - 1)))

        # Advance phase; carry → consume one input sample.
        new_phase = int(phase_acc) + step
        carries = new_phase >> 32
        phase_acc = np.uint64(new_phase & 0xffffffff)
        rptr += carries

    return np.asarray(out)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

DEFAULT_OUT = Path(__file__).resolve().parent.parent / "fpga" / "toy-dac" / "src" / "rtl" / "frac_asrc.mem"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--phases", type=int, default=PHASES_DEFAULT)
    parser.add_argument("--taps", type=int, default=TAPS_DEFAULT)
    parser.add_argument("--bits", type=int, default=COEFF_BITS_DEFAULT,
                        help="Coefficient bit width (signed)")
    parser.add_argument("--atten", type=float, default=ATTEN_DEFAULT_DB,
                        help="Target stopband attenuation in dB")
    parser.add_argument("--cutoff-frac", type=float, default=CUTOFF_FRAC_DEFAULT,
                        help="Cutoff as fraction of Fs_in/2 (default 0.9)")
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT,
                        help="Output .mem path")
    parser.add_argument("--verify", action="store_true",
                        help="Run the resampling golden model and report metrics")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(argv)

    h = design_prototype(args.phases, args.taps, args.atten, args.cutoff_frac)
    banks = polyphase_decompose(h, args.phases)
    banks_q = quantise(banks, args.bits)

    metrics = verify_passband_stopband(h, args.phases, args.bits)

    header = [
        "Fractional-phase ASRC polyphase coefficients",
        f"Generated by scripts/frac_asrc_coeffs.py",
        f"Layout: row-major, sentinel = phase-0 left-shifted by one tap",
        f"Cutoff (frac of Fs_in/2): {args.cutoff_frac}",
        f"Designed stopband attenuation: {args.atten} dB",
        f"Achieved passband ripple: {metrics['passband_ripple_db']:.3f} dB "
        f"(0..{metrics['passband_edge_norm']:.4f} of Fs_in/2 * phases)",
        f"Achieved stopband attenuation: {metrics['stopband_atten_db']:.2f} dB "
        f"(>= {metrics['stopband_edge_norm']:.4f} of dense Nyquist)",
        f"Quantisation noise floor estimate: {metrics['quant_noise_floor_dbfs']:.2f} dBFS",
    ]

    args.out.parent.mkdir(parents=True, exist_ok=True)
    write_mem(args.out, banks_q, args.bits, header)

    if not args.quiet:
        print(f"wrote {args.out}")
        for line in header:
            print(f"  {line}")

    if args.verify:
        # Resample a 1 kHz tone at 44.1 kHz input to 1.6875 MHz output.
        # step = round((Fs_in / Fs_out) * 2**32)
        fs_in = 44_100
        fs_out = 1_687_500
        step = int(round((fs_in / fs_out) * (1 << 32)))
        sig_norm = 1_000.0 / fs_in
        out = simulate_resample(banks_q, args.bits, step, n_in=4096,
                                sig_freq_norm=sig_norm)
        # FFT spur level relative to fundamental
        spec = np.abs(np.fft.rfft(out * np.hanning(len(out))))
        spec_db = 20.0 * np.log10(np.maximum(spec, 1e-30) / spec.max())
        # Find spurs outside the bin around the fundamental.
        peak_bin = int(np.argmax(spec))
        mask = np.ones_like(spec_db, dtype=bool)
        mask[max(0, peak_bin - 4): peak_bin + 5] = False
        worst_spur = float(spec_db[mask].max())
        if not args.quiet:
            print("Resampling sanity check (1 kHz @ 44.1k → 1.6875M):")
            print(f"  worst spur outside fundamental: {worst_spur:.1f} dBc "
                  f"({len(out)} output samples)")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
