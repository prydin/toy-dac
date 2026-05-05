`timescale 1ns / 1ps
`default_nettype none

// halfband_up2
// ────────────
// Symmetric 15-tap halfband interpolating FIR (×2 upsample).
//
// Architecture:
//
//   in_valid     ─►  shift-register x[0..14]
//                    │
//                    ├──► y_even = Σ h[2k]·x[2k]   (4-MAC FIR over even taps)
//                    │
//                    └──► y_odd  = x[7]            (center tap = h[7] = 1.0,
//                                                   no multiply)
//
//   y_even_q, y_odd_q  → emit on internal schedule:
//       pulse 1 @ in_valid + EMIT_DELAY
//       pulse 2 @ in_valid + EMIT_DELAY + IN_PERIOD_MCLK/2
//
// The two output samples per input are evenly spaced over the input
// period, so a downstream consumer that just latches `out_data` on
// `out_valid` sees a uniform-rate stream at 2× the input rate.
//
// Coefficient design
// ──────────────────
// Hamming-windowed sinc, prototype length 15, cutoff π/2, then
// renormalised so the FIR (even) branch has BIT-EXACT unity DC gain
// in Q1.17. CRITICAL: even a 0.01% gain mismatch between the FIR and
// passthrough polyphase branches modulates every other output sample,
// producing audible IM products that beat with the slowly-drifting
// ASRC phase trajectory. So we enforce 2·(C0+C2+C4+C6) ≡ 131072 exactly:
//
//   C0 = h[0] = h[14] =   -960     (≈ -0.007324)
//   C2 = h[2] = h[12] =  +4258     (≈ +0.032486)
//   C4 = h[4] = h[10] = -18003     (≈ -0.137352)
//   C6 = h[6] = h[8]  = +80241     (≈ +0.612213)
//   Σ check: 2·(-960 + 4258 - 18003 + 80241) = 131072  ✓
//
//   h[7]              = 1.00000    (center tap; passthrough)
//   h[1]=h[3]=h[5]=h[9]=h[11]=h[13] = 0    (halfband property)
//
// Stopband attenuation ~70 dB (vs ~50 dB for the previous 11-tap
// version). Audio-band image rejection at the cascade output is now
// well below the ΔΣ modulator's in-band noise floor.
//
// Resource cost: 4 DSP48 (one per pre-added pair), 14 × DATA_W FFs
// of shift register, plus the small emission FSM. Per stereo cascade
// (2 stages × 2 channels): 16 DSP48 total.

module halfband_up2 #(
    parameter integer DATA_W         = 32,
    parameter integer COEFF_W        = 18,
    // Number of mclks between consecutive in_valid pulses. Used only
    // to time the two output pulses per input. Must match the actual
    // input period or output spacing will be off.
    parameter integer IN_PERIOD_MCLK = 64,
    // mclks from in_valid edge to first out_valid pulse. Must be ≥ 4:
    // 1 cycle for shift-register update, 1 cycle for pre-add register,
    // 1 cycle for product register, 1 cycle for y_*_q register.
    parameter integer EMIT_DELAY     = 4
)(
    input  wire                      clk,
    input  wire                      rst,
    input  wire                      enable,

    input  wire                      in_valid,
    input  wire signed [DATA_W-1:0]  in_data,

    output reg                       out_valid = 1'b0,
    output reg  signed [DATA_W-1:0]  out_data  = {DATA_W{1'b0}}
);

    // ── 15-tap halfband coefficients (Q1.17 signed) ──────────────
    // Renormalised so 2·(C0 + C2 + C4 + C6) = 131072 exactly. See header.
    localparam signed [COEFF_W-1:0] C0 =   -18'sd960;     // ≈ -0.007324
    localparam signed [COEFF_W-1:0] C2 =  +18'sd4258;     // ≈ +0.032486
    localparam signed [COEFF_W-1:0] C4 =  -18'sd18003;    // ≈ -0.137352
    localparam signed [COEFF_W-1:0] C6 =  +18'sd80241;    // ≈ +0.612213

    // ── Input shift register: x_sr[0]=x[m] (latest) … x_sr[14]=x[m-14]
    reg signed [DATA_W-1:0] x_sr [0:14];
    integer i;
    always @(posedge clk) begin
        if (rst) begin
            for (i = 0; i < 15; i = i + 1) x_sr[i] <= {DATA_W{1'b0}};
        end else if (enable && in_valid) begin
            x_sr[0] <= in_data;
            for (i = 1; i < 15; i = i + 1) x_sr[i] <= x_sr[i-1];
        end
    end

    // ── Symmetric pre-adds (DATA_W+1 bits signed) ────────────────
    // Registered to break the long combinational chain from the shift
    // register through the multipliers and adder tree (was failing
    // timing at 108 MHz with DATA_W=32: the pre-add was 33b → multiply
    // didn't fit in a single 25×18 DSP48 and Vivado built it from
    // CARRY4s in fabric, ~17 logic levels).
    reg signed [DATA_W:0] s0_14_q = {(DATA_W+1){1'b0}};
    reg signed [DATA_W:0] s2_12_q = {(DATA_W+1){1'b0}};
    reg signed [DATA_W:0] s4_10_q = {(DATA_W+1){1'b0}};
    reg signed [DATA_W:0] s6_8_q  = {(DATA_W+1){1'b0}};
    always @(posedge clk) begin
        if (rst) begin
            s0_14_q <= {(DATA_W+1){1'b0}};
            s2_12_q <= {(DATA_W+1){1'b0}};
            s4_10_q <= {(DATA_W+1){1'b0}};
            s6_8_q  <= {(DATA_W+1){1'b0}};
        end else begin
            s0_14_q <= x_sr[0]  + x_sr[14];
            s2_12_q <= x_sr[2]  + x_sr[12];
            s4_10_q <= x_sr[4]  + x_sr[10];
            s6_8_q  <= x_sr[6]  + x_sr[8];
        end
    end

    // ── Multiplies, registered (lights up DSP48 M output register) ─
    reg signed [DATA_W+COEFF_W:0] p0_q = {(DATA_W+COEFF_W+1){1'b0}};
    reg signed [DATA_W+COEFF_W:0] p2_q = {(DATA_W+COEFF_W+1){1'b0}};
    reg signed [DATA_W+COEFF_W:0] p4_q = {(DATA_W+COEFF_W+1){1'b0}};
    reg signed [DATA_W+COEFF_W:0] p6_q = {(DATA_W+COEFF_W+1){1'b0}};
    always @(posedge clk) begin
        if (rst) begin
            p0_q <= {(DATA_W+COEFF_W+1){1'b0}};
            p2_q <= {(DATA_W+COEFF_W+1){1'b0}};
            p4_q <= {(DATA_W+COEFF_W+1){1'b0}};
            p6_q <= {(DATA_W+COEFF_W+1){1'b0}};
        end else begin
            p0_q <= s0_14_q * C0;
            p2_q <= s2_12_q * C2;
            p4_q <= s4_10_q * C4;
            p6_q <= s6_8_q  * C6;
        end
    end

    // ── Sum, round, shift, saturate ──────────────────────────────
    // Two extra guard bits over the 4 product terms (4 < 2^2).
    wire signed [DATA_W+COEFF_W+2:0] sum_full = p0_q + p2_q + p4_q + p6_q;
    // Round-to-nearest by adding 0.5 LSB before the right shift.
    wire signed [DATA_W+COEFF_W+2:0] sum_rnd  =
        sum_full + (1 <<< (COEFF_W-2));
    wire signed [DATA_W+3:0]         y_full   = sum_rnd >>> (COEFF_W-1);

    localparam signed [DATA_W-1:0] MAX_VAL =  {1'b0, {(DATA_W-1){1'b1}}};
    localparam signed [DATA_W-1:0] MIN_VAL = ~MAX_VAL;

    wire signed [DATA_W-1:0] y_even =
        (y_full >  $signed({{4{1'b0}}, MAX_VAL})) ? MAX_VAL :
        (y_full <  $signed({{4{1'b1}}, MIN_VAL})) ? MIN_VAL :
                                                    y_full[DATA_W-1:0];

    // Passthrough: y[2k+1] = h[7] · x[m-7] = 1.0 · x_sr[7]
    wire signed [DATA_W-1:0] y_odd = x_sr[7];

    // ── Output register ──────────────────────────────────────────
    // y_even latency from in_valid: x_sr (1) + s_q (1) + p_q (1) + y_even_q (1)
    // = 4 cycles, hence default EMIT_DELAY=4. y_odd has only x_sr (1) +
    // y_odd_q (1) of physical latency but is held in y_odd_q until the
    // next in_valid, so sampling at EMIT_DELAY=4 is safe.
    reg signed [DATA_W-1:0] y_even_q = {DATA_W{1'b0}};
    reg signed [DATA_W-1:0] y_odd_q  = {DATA_W{1'b0}};
    always @(posedge clk) begin
        if (rst) begin
            y_even_q <= {DATA_W{1'b0}};
            y_odd_q  <= {DATA_W{1'b0}};
        end else begin
            y_even_q <= y_even;
            y_odd_q  <= y_odd;
        end
    end

    // ── Emission FSM ─────────────────────────────────────────────
    localparam integer HALF       = IN_PERIOD_MCLK / 2;
    localparam integer EMIT_LAST  = EMIT_DELAY + HALF;
    localparam integer CNT_MAX    = EMIT_LAST + 1;
    localparam integer CNT_W      = $clog2(CNT_MAX + 1);

    reg [CNT_W-1:0] em_cnt    = {CNT_W{1'b0}};
    reg             em_active = 1'b0;

    always @(posedge clk) begin
        if (rst || !enable) begin
            em_cnt    <= {CNT_W{1'b0}};
            em_active <= 1'b0;
            out_valid <= 1'b0;
            out_data  <= {DATA_W{1'b0}};
        end else begin
            out_valid <= 1'b0;
            // Schedule outputs on each in_valid. (in_valid arrival
            // resets the counter even mid-emission, so a glitch in
            // input rate is handled by re-syncing.)
            if (in_valid) begin
                em_cnt    <= {CNT_W{1'b0}};
                em_active <= 1'b1;
            end else if (em_active && em_cnt < CNT_MAX) begin
                em_cnt <= em_cnt + 1'b1;
            end

            if (em_active && em_cnt == EMIT_DELAY[CNT_W-1:0]) begin
                out_valid <= 1'b1;
                out_data  <= y_even_q;
            end else if (em_active && em_cnt == EMIT_LAST[CNT_W-1:0]) begin
                out_valid <= 1'b1;
                out_data  <= y_odd_q;
                em_active <= 1'b0;   // done with this input
            end
        end
    end

endmodule

`default_nettype wire
