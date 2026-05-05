`timescale 1ns / 1ps
`default_nettype none

// dac — parametric 1-bit CIFB ΔΣ modulator with one shared loop.
//
// Topology (Cascade-of-Integrators with distributed Feedback,
// optional resonator feedback to move NTF zeros off DC):
//
//   stage 0     : σ₀[n+1] = σ₀[n] + b₁·u[n]     − a₁·v[n]
//   stage k≥1   : σ_k[n+1] = σ_k[n] + c_{k-1}·σ_{k-1}[n] − a_{k+1}·v[n]
//                            − g_j·σ_{k+1}[n]   (if a g pair lands on k)
//   v[n]        = sign(σ_{N-1}[n] + dither)     (combinational)
//
// Resonator pairing (matches python-deltasigma realizeNTF/CIFB
// convention; same indexing as scripts/dsm_model.py):
//   g[0] couples (σ_{N-2}, σ_{N-1})  →  σ_{N-2} -= g[0]·σ_{N-1}
//   g[1] couples (σ_{N-4}, σ_{N-3})  →  σ_{N-4} -= g[1]·σ_{N-3}
//   …
//
// Coefficient sources
// -------------------
// For ORDER ∈ {1, 2} the module synthesises Pascal-triangle weights
// internally if the caller leaves DSM_A all-zero (the legacy default).
// Pascal NTF = (1 − z⁻¹)^ORDER, OBG = 2^ORDER:
//   ORDER=1: a={1.0},     b₁=0.5            (OBG=2, unconditionally stable)
//   ORDER=2: a={1.0, 2.0}, b₁=0.5, c={1.0,—} (OBG=4, stable for |u|≲0.9 FS)
//
// (The b₁=0.5 default reproduces the legacy module's effective scaling:
//  the old code added `din` directly while feeding back v=2^WORDLENGTH,
//  so din_FS/v_FS = 0.5. In normalised units that *is* b₁ = 0.5.)
//
// For ORDER ≥ 3, Pascal weights have OBG ≥ 8 which exceeds the Lee
// bound for a 1-bit quantizer — every integrator rails. Higher orders
// need spread NTF zeros and a tighter OBG; use scripts/synthesize_dsm.py
// to design a stable NTF and pass the resulting (a, c, b₁, g) arrays
// (or `\u0060include "dsm_coeffs.vh"` and parameter-pass them).
//
// Signal scaling
// --------------
// din is signed Q(0).(WORDLENGTH-1); ±2^(WORDLENGTH-1) represents ±1.0 FS
// in the modulator's normalised units. State uses Q(STATE_INT_BITS).
// (WORDLENGTH-1) — STATE_INT_BITS integer guard bits above ±1 FS give
// the integrators headroom before they wrap. Quantizer feedback v has
// magnitude 2^(WORDLENGTH-1) (= 1.0 FS).
//
// All multiplies (a, c, b₁, g) are computed as
//   (state · coeff) >>> COEFF_FRAC
// Vivado infers DSP48 slices automatically; for power-of-two
// coefficients (Pascal) constant-propagation reduces them to shifts.

module dac #(
    parameter integer WORDLENGTH     = 32,
    // Modulator order. ORDER ∈ {1, 2} get auto-Pascal coefficients if
    // the caller doesn't override them. ORDER ≥ 3 requires synthesised
    // coefficients (see scripts/synthesize_dsm.py).
    parameter integer ORDER          = 2,
    // Coefficient Q-format (signed Q(COEFF_W-COEFF_FRAC-1).COEFF_FRAC).
    // Match what synthesize_dsm.py emits in dsm_coeffs.vh.
    parameter integer COEFF_W        = 24,
    parameter integer COEFF_FRAC     = 20,
    // Integer guard bits above ±1 FS in the integrator state. 4 = ±16
    // FS headroom before wrap; comfortable for any well-behaved NTF.
    parameter integer STATE_INT_BITS = 4,
    // Number of resonator-feedback pairs. 0 for Pascal/no-resonator.
    parameter integer N_G            = 0,
    // Coefficient arrays (signed Q-format above). Default all-zero;
    // module synthesises Pascal weights internally when ORDER∈{1,2}
    // and DSM_A[0]==0.
    parameter signed [COEFF_W-1:0] DSM_A  [0:ORDER-1] = '{default: 0},
    parameter signed [COEFF_W-1:0] DSM_C  [0:ORDER-1] = '{default: 0},
    parameter signed [COEFF_W-1:0] DSM_B1                      = 0,
    parameter signed [COEFF_W-1:0] DSM_G  [0:(N_G > 0 ? N_G-1 : 0)] = '{default: 0}
)(
    input  wire                          clk,
    input  wire                          rst,
    input  wire signed [WORDLENGTH-1:0]  din,
    input  wire                          dvalid,
    input  wire [31:0]                   dither1,
    input  wire [31:0]                   dither2,
    output reg                           dout = 1'b0
);

// ── Sanity checks ──────────────────────────────────────────────────────
initial begin
    if (ORDER < 1) begin
        $display("ERROR dac.v: ORDER must be >= 1 (got %0d)", ORDER);
        $finish;
    end
    if (ORDER > 2 && DSM_A[0] == 0) begin
        $display("ERROR dac.v: ORDER=%0d but DSM_A is all-zero. Pass synthesised coefficients (see scripts/synthesize_dsm.py).", ORDER);
        $finish;
    end
end

// ── Effective coefficients ─────────────────────────────────────────────
// Auto-Pascal for legacy ORDER∈{1,2} when caller didn't override DSM_A.
// Detected at elaboration (DSM_A is a parameter), so the generate
// branch is selected statically.
localparam signed [COEFF_W-1:0] ONE_Q  = $signed({{(COEFF_W-COEFF_FRAC-1){1'b0}}, 1'b1, {COEFF_FRAC{1'b0}}});
localparam signed [COEFF_W-1:0] HALF_Q = ONE_Q >>> 1;

wire signed [COEFF_W-1:0] a_eff [0:ORDER-1];
wire signed [COEFF_W-1:0] c_eff [0:ORDER-1];
wire signed [COEFF_W-1:0] b1_eff;

genvar gi;
generate
    if (DSM_A[0] == 0 && ORDER == 1) begin : g_pascal1
        assign a_eff[0] = ONE_Q;
        assign c_eff[0] = '0;
        assign b1_eff   = HALF_Q;
    end else if (DSM_A[0] == 0 && ORDER == 2) begin : g_pascal2
        assign a_eff[0] = ONE_Q;
        assign a_eff[1] = ONE_Q <<< 1;
        assign c_eff[0] = ONE_Q;   // gain σ₀ → σ₁
        assign c_eff[1] = '0;      // unused (last stage has no successor)
        assign b1_eff   = HALF_Q;
    end else begin : g_user_coeffs
        for (gi = 0; gi < ORDER; gi = gi + 1) begin : g_passthru
            assign a_eff[gi] = DSM_A[gi];
            assign c_eff[gi] = DSM_C[gi];
        end
        assign b1_eff = DSM_B1;
    end
endgenerate

// ── Bit widths ─────────────────────────────────────────────────────────
localparam integer STATE_FRAC = WORDLENGTH - 1;                  // = 31
localparam integer STATE_W    = STATE_FRAC + 1 + STATE_INT_BITS; // = 36 default

// Quantizer feedback magnitude: ±1.0 FS in state Q-format = ±2^STATE_FRAC.
localparam signed [STATE_W-1:0] V_POS = $signed({{(STATE_INT_BITS){1'b0}}, 1'b1, {STATE_FRAC{1'b0}}});
localparam signed [STATE_W-1:0] V_NEG = -V_POS;

// ── State ──────────────────────────────────────────────────────────────
reg  signed [STATE_W-1:0] sigma [0:ORDER-1];
reg  signed [WORDLENGTH-1:0] din_held;

integer iv;
initial begin
    for (iv = 0; iv < ORDER; iv = iv + 1) sigma[iv] = '0;
    din_held = '0;
end

// din placed in state Q-format (sign-extend; fractional bits already align
// because STATE_FRAC = WORDLENGTH-1).
wire signed [STATE_W-1:0] din_q =
    {{(STATE_W - WORDLENGTH){din_held[WORDLENGTH-1]}}, din_held};

// ── TPDF dither at the comparator input ────────────────────────────────
// (See historical notes: dither at the comparator sees the full NTF and
// can be much larger than loop-input dither without raising audio-band
// noise. Two independent uniform sources summed give the textbook
// triangular PDF.)
//
// Magnitude rule: peak TPDF span ≈ ±½ quantizer LSB. The quantizer step
// here is 2·V_POS = 2^WORDLENGTH, so ½-LSB peak = ±V_POS/2 = ±2^(WL-2).
// Each uniform source therefore spans ±V_POS/4 = ±2^(WL-3) and the TPDF
// sum spans ±V_POS/2.
//
// Why ½-LSB rather than the textbook 1-LSB peak? σ_{N-1} of a stable
// 1-bit modulator hovers near ±1 FS = ±V_POS. A 1-LSB-peak dither (=
// ±V_POS) is comparable to σ_{N-1} itself and lets the dither — not the
// signal — pick the comparator output, randomising the loop. Half-LSB
// peak keeps the comparator signal-dominated while still randomising
// near σ_{N-1} = 0 crossings (which is where dither is needed).
//
// We take the high WORDLENGTH-2 bits of each 32-bit LFSR word and
// sign-extend to get sources of magnitude ±2^(WL-3).
wire signed [WORDLENGTH-1:0] dither_a = $signed({{2{dither1[WORDLENGTH-1]}}, dither1[WORDLENGTH-1:2]});
wire signed [WORDLENGTH-1:0] dither_b = $signed({{2{dither2[WORDLENGTH-1]}}, dither2[WORDLENGTH-1:2]});
wire signed [WORDLENGTH:0]   dither_tpdf = dither_a + dither_b;
wire signed [STATE_W-1:0]    dither_q =
    {{(STATE_W - WORDLENGTH - 1){dither_tpdf[WORDLENGTH]}}, dither_tpdf};

// Comparator: combinational sign of (last integrator + dither). Must be
// combinational — adding a register here introduces an extra z⁻¹ in the
// loop that breaks the NTF derivation.
wire signed [STATE_W-1:0] sigma_last_dith = sigma[ORDER-1] + dither_q;
wire q_bit = ~sigma_last_dith[STATE_W-1];          // 1 → +
wire signed [STATE_W-1:0] v_full = q_bit ? V_POS : V_NEG;

// ── Q-format multiply: (x · coeff) >>> COEFF_FRAC ──────────────────────
// Result truncated to STATE_W bits; coefficient magnitudes are bounded
// (< 2^(COEFF_W-COEFF_FRAC-1)) so the product stays inside the
// STATE_INT_BITS guard band.
function automatic signed [STATE_W-1:0] mul_q(
    input signed [STATE_W-1:0] x,
    input signed [COEFF_W-1:0] c
);
    reg signed [STATE_W+COEFF_W-1:0] p;
    begin
        p = x * c;
        mul_q = p >>> COEFF_FRAC;
    end
endfunction

// ── Combinational next-state ───────────────────────────────────────────
reg  signed [STATE_W-1:0] sigma_next [0:ORDER-1];
integer k, j, idx_lo, idx_hi;

always @(*) begin
    // Stage 0: σ₀ += b₁·u − a₁·v
    sigma_next[0] = sigma[0]
                  + mul_q(din_q,  b1_eff)
                  - mul_q(v_full, a_eff[0]);

    // Stages 1..N-1: σ_k += c_{k-1}·σ_{k-1} − a_{k+1}·v
    for (k = 1; k < ORDER; k = k + 1) begin
        sigma_next[k] = sigma[k]
                      + mul_q(sigma[k-1], c_eff[k-1])
                      - mul_q(v_full,     a_eff[k]);
    end

    // Resonator feedback. Each g[j] subtracts g[j]·σ_{idx_hi} from
    // σ_{idx_lo}, with (idx_lo, idx_hi) = (ORDER-2-2j, ORDER-1-2j).
    for (j = 0; j < N_G; j = j + 1) begin
        idx_hi = ORDER - 1 - 2*j;
        idx_lo = idx_hi - 1;
        if (idx_lo >= 0)
            sigma_next[idx_lo] = sigma_next[idx_lo]
                               - mul_q(sigma[idx_hi], DSM_G[j]);
    end
end

// ── State register & output ────────────────────────────────────────────
always @(posedge clk or posedge rst) begin
    if (rst) begin
        for (iv = 0; iv < ORDER; iv = iv + 1) sigma[iv] <= '0;
        din_held <= '0;
        dout     <= 1'b0;
    end else begin
        if (dvalid)
            din_held <= din;
        for (iv = 0; iv < ORDER; iv = iv + 1)
            sigma[iv] <= sigma_next[iv];
        dout <= q_bit;
    end
end

endmodule

`default_nettype wire

