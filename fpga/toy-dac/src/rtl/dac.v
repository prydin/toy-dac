`timescale 1ns / 1ps
`default_nettype none

// CIFB ΔΣ modulator with one shared loop and one comparator.
//
// Structure for ORDER N (N ∈ {1, 2}):
//
//   stage 0 :  σ₀[n+1] = σ₀[n] + din − a₁·v
//   stage 1 :  σ₁[n+1] = σ₁[n] + σ₀[n] − a₂·v       (only if ORDER=2)
//   v[n]    =  sign(σₙ₋₁[n] + dither)               (combinational)
//
// Feedback coefficients (hard-coded — Pascal-triangle weights for
// stacked-NTF-zeros-at-DC, the textbook maximally-flat shape):
//   ORDER=1 :  a = {1}      NTF = (1 − z⁻¹)¹  OBG=2  unconditionally stable
//   ORDER=2 :  a = {1, 2}   NTF = (1 − z⁻¹)²  OBG=4  stable for |din| ≲ 0.9 FS
//
// (ORDER ≥ 3 with these coefficients is unstable on a single-bit
//  quantizer — OBG=8 exceeds the Lee bound, integrators rail for any
//  input including zero. Verified in sim. If a 3rd-order NTF is ever
//  needed, the zeros must be moved off DC and/or a multi-bit
//  quantizer used; that's a future redesign, not a parameter tweak.)
//
// All aₖ are 1 or 2 → implemented as shift, no multipliers.
// Quantizer feedback magnitude is up_inc = 2^WORDLENGTH (so q_fb is
// always at full scale of the input word).

module dac #(
    parameter WORDLENGTH = 32,
    // Modulator order. Supported: 1 or 2. See header for stability bounds.
    parameter ORDER = 2
)(
    input  wire                          clk,
    input  wire                          rst,
    input  wire signed [WORDLENGTH-1:0]  din,
    input  wire                          dvalid,
    input  wire [31:0]                   dither1,
    input  wire [31:0]                   dither2,
    output reg                           dout = 0
);

// ── Sanity check on ORDER ──────────────────────────────────────────────
initial begin
    if (ORDER < 1 || ORDER > 2) begin
        $display("ERROR dac.v: ORDER=%0d not supported (must be 1 or 2)", ORDER);
        $finish;
    end
end

// ── Bit widths ─────────────────────────────────────────────────────────
// Integrators need headroom over the input word. Worst-case stage gain
// in the stable 2nd-order Pascal modulator is 2×; 10 guard bits is
// comfortable.
localparam GUARD_BITS = 10;
localparam ACCLENGTH  = WORDLENGTH + GUARD_BITS;

// Quantizer levels in ACCLENGTH-bit signed space: ±2^WORDLENGTH.
localparam signed [ACCLENGTH-1:0] up_inc   = $signed({{(GUARD_BITS-1){1'b0}}, 1'b1, {WORDLENGTH{1'b0}}});
localparam signed [ACCLENGTH-1:0] down_inc = -up_inc;

// 2·v, used as the second-stage feedback when ORDER==2. Just up_inc
// shifted left by one (so ±2^(WORDLENGTH+1)).
localparam signed [ACCLENGTH-1:0] up_inc_x2   = up_inc <<< 1;
localparam signed [ACCLENGTH-1:0] down_inc_x2 = -up_inc_x2;

// ── State ──────────────────────────────────────────────────────────────
reg  signed [ACCLENGTH-1:0]  sigma0 = {ACCLENGTH{1'b0}};
reg  signed [ACCLENGTH-1:0]  sigma1 = {ACCLENGTH{1'b0}};   // unused if ORDER==1

// ── Quantizer-input TPDF dither ────────────────────────────────────────
// Dither is summed at the COMPARATOR input rather than at the loop
// input. Two reasons:
//
//  - At the comparator, dither sees the noise transfer function
//    NTF = (1−z⁻¹)^ORDER, so the audio-band component is removed
//    exactly the same way as quantization noise. We can therefore
//    use much larger dither without raising the audio-band noise
//    floor — and large dither is what actually breaks the modulator
//    out of pattern-locked limit cycles near rational inputs (zero,
//    ±FS/2, etc.). Loop-input dither sees STF ≈ 1 and reaches the
//    audio output 1:1, so it has to be tiny and consequently fails
//    to randomize the quantizer decision.
//
//  - The integrators in front of the quantizer low-pass-filter any
//    dither injected at the loop input — by the time it reaches the
//    comparator the high-frequency content (where decorrelation
//    actually happens) is gone.
//
// TPDF = sum of two independent uniform sources → triangular PDF,
// the optimal dither shape for a quantizer (1st-order moment of the
// quantization error becomes signal-independent).
//
// IMPORTANT — magnitude: dither must be at least ±½ LSB of the
// QUANTIZER (not of the data word) to actually randomize comparator
// decisions. The quantizer step here is 2·up_inc = 2^(WORDLENGTH+1),
// so each uniform source needs to span ±2^WORDLENGTH and the TPDF
// sum spans ±2^(WORDLENGTH+1) — i.e. one full quantizer LSB peak
// per side, the textbook value. Using narrower dither leaves the
// quantizer in pattern-locked operation and (worse) the small
// signal-correlated perturbation can actually *increase* spur
// power. We use the full 32-bit LFSR words as $signed.
wire signed [WORDLENGTH:0] dither_tpdf =
        $signed(dither1[WORDLENGTH-1:0]) + $signed(dither2[WORDLENGTH-1:0]);

// Comparator: sign-extract on (last integrator + dither). Combinational
// — the Pascal-coefficient NTF derivation assumes a zero-delay
// quantizer (the only z⁻¹s in the loop come from the integrators
// themselves). Registering the comparator adds an extra z⁻¹ in the
// feedback path, which would destabilise the loop. We take the sign
// combinationally and only register the pad output `dout`.
wire signed [ACCLENGTH-1:0]  sigma_last     = (ORDER == 2) ? sigma1 : sigma0;
wire signed [ACCLENGTH-1:0]  sigma_last_dith = sigma_last + $signed(dither_tpdf);
wire q_bit = ~sigma_last_dith[ACCLENGTH-1];   // sign bit (1 = sigma+dither >= 0)
wire signed [ACCLENGTH-1:0]  v_full    = q_bit ? up_inc    : down_inc;
wire signed [ACCLENGTH-1:0]  v_full_x2 = q_bit ? up_inc_x2 : down_inc_x2;

reg  signed [WORDLENGTH-1:0] din_held = 0;

// ── Modulator ──────────────────────────────────────────────────────────
always @(posedge clk or posedge rst) begin
    if (rst) begin
        sigma0   <= {ACCLENGTH{1'b0}};
        sigma1   <= {ACCLENGTH{1'b0}};
        din_held <= {WORDLENGTH{1'b0}};
        dout     <= 1'b0;
    end else begin
        if (dvalid)
            din_held <= din;

        // Stage 0: σ₀ += din − a₁·v ; a₁ = 1 for both ORDER=1 and ORDER=2.
        sigma0 <= sigma0 + din_held - v_full;

        // Stage 1 (only meaningful when ORDER=2):
        //   σ₁ += σ₀ − 2·v
        // For ORDER=1 we still clock sigma1, but it stays at zero
        // because nothing reads it (sigma_last comes from sigma0)
        // and the synthesizer will trim it.
        if (ORDER == 2)
            sigma1 <= sigma1 + sigma0 - v_full_x2;

        // Output the quantizer bit (registered for clean IOB launch)
        dout <= q_bit;
    end
end

endmodule

`default_nettype wire

