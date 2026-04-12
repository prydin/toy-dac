"""Generate FIR coefficients for a polyphase upsampling interpolator.

Designs a lowpass prototype filter via windowed-sinc (Kaiser window),
decomposes it into polyphase sub-filters, and quantises the coefficients
to 32-bit signed integers.
"""

import argparse
import numpy as np


def kaiser_beta(attenuation_db: float) -> float:
    """Compute Kaiser window beta from desired stop-band attenuation."""
    if attenuation_db > 50:
        return 0.1102 * (attenuation_db - 8.7)
    elif attenuation_db >= 21:
        return 0.5842 * (attenuation_db - 21) ** 0.4 + 0.07886 * (attenuation_db - 21)
    else:
        return 0.0


def kaiser_order(attenuation_db: float, transition_width: float) -> int:
    """Estimate minimum filter order from Kaiser's formula.

    transition_width is normalised to [0, 1] (fraction of Nyquist).
    """
    order = int(np.ceil((attenuation_db - 7.95) / (2.285 * transition_width * np.pi)))
    if order < 1:
        order = 1
    return order


def design_prototype(upsample_ratio: int, taps_per_phase: int | None,
                     attenuation_db: float) -> np.ndarray:
    """Design the prototype lowpass FIR filter.

    The cutoff is at pi/L (normalised to Nyquist of the upsampled rate).
    Returns the full set of N = upsample_ratio * taps_per_phase coefficients.
    """
    L = upsample_ratio
    # Normalised transition band width (fraction of Nyquist at the high rate).
    # A reasonable default: half the passband width.
    transition_width = 1.0 / L

    beta = kaiser_beta(attenuation_db)

    if taps_per_phase is None:
        order = kaiser_order(attenuation_db, transition_width)
        # Round up so the total length is a multiple of L
        taps_per_phase = int(np.ceil((order + 1) / L))
        if taps_per_phase < 4:
            taps_per_phase = 4

    N = L * taps_per_phase  # total prototype filter length
    window = np.kaiser(N, beta)

    # Sinc lowpass at cutoff = 1/L (normalised to Nyquist of high rate)
    fc = 1.0 / L  # cutoff in [0,1] relative to half the upsampled rate
    n = np.arange(N)
    mid = (N - 1) / 2.0
    sinc_arg = fc * (n - mid)
    h = np.where(sinc_arg == 0, fc, np.sin(np.pi * sinc_arg) / (np.pi * sinc_arg))
    h *= window

    # Normalise so each polyphase phase sums to ~1 (unity DC gain per phase)
    h *= L / np.sum(h)

    return h


def quantise_to_int32(h: np.ndarray) -> np.ndarray:
    """Scale and round floating-point coefficients to signed 32-bit integers.

    The largest coefficient maps close to 2**31 - 1.
    """
    max_val = np.max(np.abs(h))
    if max_val == 0:
        return np.zeros_like(h, dtype=np.int32)
    scale = (2**31 - 1) / max_val
    return np.clip(np.round(h * scale), -(2**31), 2**31 - 1).astype(np.int32)


def polyphase_decompose(h: np.ndarray, L: int) -> np.ndarray:
    """Decompose prototype h into L polyphase sub-filters.

    Returns an array of shape (L, taps_per_phase).
    Phase k contains h[k], h[k+L], h[k+2L], ...
    """
    taps_per_phase = len(h) // L
    return h[:L * taps_per_phase].reshape(taps_per_phase, L).T


def main():
    parser = argparse.ArgumentParser(
        description="Generate 32-bit integer FIR coefficients for a polyphase upsampling interpolator.")
    parser.add_argument("ratio", type=int, help="Upsampling ratio (L)")
    parser.add_argument("--taps-per-phase", type=int, default=None,
                        help="Number of taps per polyphase sub-filter (auto if omitted)")
    parser.add_argument("--attenuation", type=float, default=80.0,
                        help="Stop-band attenuation in dB (default: 80)")
    parser.add_argument("--proto", action="store_true",
                        help="Output the raw prototype filter instead of polyphase decomposition")
    parser.add_argument("--output", type=str, default=None,
                        help="Write coefficients to a file (default: print to stdout)")
    args = parser.parse_args()

    if args.ratio < 2:
        parser.error("Upsampling ratio must be >= 2")

    L = args.ratio
    h = design_prototype(L, args.taps_per_phase, args.attenuation)
    h_int = quantise_to_int32(h)

    lines = []

    if args.proto:
        N = len(h_int)
        lines.append(f"; Prototype FIR filter for {L}x upsampling interpolator")
        lines.append(f"; Taps: {N}")
        lines.append(f"; Stop-band attenuation: {args.attenuation} dB")
        lines.append(f"; Coefficients are 32-bit signed integers")
        lines.append("radix = 10;")
        lines.append("coefdata =")
        for i, c in enumerate(h_int):
            sep = ";" if i == N - 1 else ","
            lines.append(f"{int(c)}{sep}")
    else:
        phases = polyphase_decompose(h_int, L)
        taps_per_phase = phases.shape[1]
        flat = phases.flatten()
        total = len(flat)

        lines.append(f"; Polyphase FIR filter for {L}x upsampling interpolator")
        lines.append(f"; Phases: {L}, Taps per phase: {taps_per_phase}")
        lines.append(f"; Stop-band attenuation: {args.attenuation} dB")
        lines.append(f"; Coefficients are 32-bit signed integers")
        lines.append(f"; Layout: phase-major order — phase 0 taps, then phase 1, ...")
        lines.append("radix = 10;")
        lines.append("coefdata =")
        idx = 0
        for phase_idx in range(L):
            lines.append(f"; Phase {phase_idx}")
            for tap in range(taps_per_phase):
                sep = ";" if idx == total - 1 else ","
                lines.append(f"{int(phases[phase_idx, tap])}{sep}")
                idx += 1

    output_text = "\n".join(lines) + "\n"

    if args.output:
        with open(args.output, "w") as f:
            f.write(output_text)
        print(f"Coefficients written to {args.output}")
    else:
        print(output_text)


if __name__ == "__main__":
    main()
