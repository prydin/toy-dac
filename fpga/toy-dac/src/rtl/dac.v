`timescale 1ns / 1ps
`default_nettype none

// CIFB ΔΣ modulator with one shared loop and one comparator.
//
// Structure for ORDER N:
//
//   stage 0 :  σ₀[n+1] = σ₀[n] + (din+dither) − a₁·v
//   stage k :  σₖ[n+1] = σₖ[n] +     σₖ₋₁[n] − aₖ₊₁·v   for k = 1..N-1
//   v[n]    =  sign(σₙ₋₁[n])    (registered → one extra z⁻¹ in the loop)
//
// The aₖ are the Pascal-triangle weights C(N, k-1):
//   ORDER=1 :  a = {1}            NTF ≈ (1 − z⁻¹)¹      unconditionally stable
//   ORDER=2 :  a = {1, 2}         NTF ≈ (1 − z⁻¹)²      stable for |din| ≲ 0.9 FS
//   ORDER=3 :  a = {1, 3, 3}      NTF ≈ (1 − z⁻¹)³      stable for |din| ≲ 0.5 FS
//
// All aₖ are integers ≤ 3 → implemented as shift+add, no multipliers.
// Quantizer feedback magnitude is up_inc = 2^WORDLENGTH (so q_fb is always
// at full scale of the input word, matching the original interface).

module dac #(
    parameter WORDLENGTH = 32,
    // Modulator order. Supported: 1, 2, 3. See header for stability bounds.
    parameter ORDER = 3
)(
    input  wire                          clk,
    input  wire                          rst,
    input  wire signed [WORDLENGTH-1:0]  din,
    input  wire                          dvalid,
    input  wire [31:0]                   dither1,
    input  wire [31:0]                   dither2,
    output reg                           dout = 0,
    output wire signed [WORDLENGTH-1:0]  din_held_debug
);

// ── Sanity check on ORDER ──────────────────────────────────────────────
initial begin
    if (ORDER < 1 || ORDER > 3) begin
        $display("ERROR dac.v: ORDER=%0d not supported (must be 1, 2 or 3)", ORDER);
        $finish;
    end
end

// ── Bit widths ─────────────────────────────────────────────────────────
// Integrators need headroom over the input word. With Pascal coefficients
// the worst-case stage gain in a stable 3rd-order is roughly 4×; 12 guard
// bits is comfortable.
localparam GUARD_BITS = 10;
localparam ACCLENGTH  = WORDLENGTH + GUARD_BITS;

// Quantizer levels in ACCLENGTH-bit signed space: ±2^WORDLENGTH.
localparam signed [ACCLENGTH-1:0] up_inc   = $signed({{(GUARD_BITS-1){1'b0}}, 1'b1, {WORDLENGTH{1'b0}}});
localparam signed [ACCLENGTH-1:0] down_inc = -up_inc;

// ── Pascal feedback coefficients ───────────────────────────────────────
// pascal_coef(N, k) = C(N, k) for k = 0 .. N-1
function integer pascal_coef;
    input integer order;
    input integer k;
    begin
        case (order)
            1: pascal_coef = 1;                       // {1}
            2: pascal_coef = (k == 0) ? 1 : 2;        // {1, 2}
            3: pascal_coef = (k == 0) ? 1 : 3;        // {1, 3, 3}
            default: pascal_coef = 1;
        endcase
    end
endfunction

// Multiply v_full by a small integer coefficient (1, 2, or 3) using
// shifts and adds only.
function signed [ACCLENGTH-1:0] coef_times_v;
    input integer                    coef;
    input signed [ACCLENGTH-1:0]     v;
    begin
        case (coef)
            1:       coef_times_v = v;
            2:       coef_times_v = v <<< 1;
            3:       coef_times_v = v + (v <<< 1);
            default: coef_times_v = v;
        endcase
    end
endfunction

// ── State ──────────────────────────────────────────────────────────────
reg  signed [ACCLENGTH-1:0]  sigma [0:ORDER-1];
reg                          q_bit = 1'b0;     // single, registered comparator
wire signed [ACCLENGTH-1:0]  v_full = q_bit ? up_inc : down_inc;

reg  signed [WORDLENGTH-1:0] din_held = 0;
assign din_held_debug = din_held;

// ── Input dither (TPDF, ±2^DITHER_BITS) ────────────────────────────────
localparam DITHER_BITS = 24;
wire signed [DITHER_BITS:0] dither_tpdf =
        $signed(dither1[DITHER_BITS-1:0]) + $signed(dither2[DITHER_BITS-1:0]);

// ── Modulator ──────────────────────────────────────────────────────────
integer k;
integer init_j;

initial begin
    for (init_j = 0; init_j < ORDER; init_j = init_j + 1)
        sigma[init_j] = 0;
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        for (k = 0; k < ORDER; k = k + 1)
            sigma[k] <= {ACCLENGTH{1'b0}};
        q_bit    <= 1'b0;
        din_held <= {WORDLENGTH{1'b0}};
        dout     <= 1'b0;
    end else begin
        if (dvalid)
            din_held <= din;

        // Stage 0: input + dither, minus a₁·v
        sigma[0] <= sigma[0]
                  + din_held
                  + dither_tpdf
                  - coef_times_v(pascal_coef(ORDER, 0), v_full);

        // Stages 1..ORDER-1: previous integrator's value, minus aₖ₊₁·v
        for (k = 1; k < ORDER; k = k + 1)
            sigma[k] <= sigma[k]
                      + sigma[k-1]
                      - coef_times_v(pascal_coef(ORDER, k), v_full);

        // Single quantizer at the last stage
        q_bit <= (sigma[ORDER-1] >= 0) ? 1'b1 : 1'b0;

        // Output the quantizer bit
        dout  <= q_bit;
    end
end

endmodule

`default_nettype wire
`timescale 1ns / 1ps
`default_nettype none

module dac #(
    parameter WORDLENGTH = 32,
    // Modulator order. Each stage adds one integrator.
    // ORDER=2 is unconditionally stable.
    // ORDER=3 is stable for moderate input levels (< ~50% FS).
    // ORDER>3 risks instability and is not recommended without coefficient tuning.
    parameter ORDER = 3
)(
    input  wire                      clk,
    input  wire                      rst,
    input  wire signed [WORDLENGTH-1:0] din,
    input  wire                      dvalid,
    input  wire [31:0]               dither1,
    input  wire [31:0]               dither2,
    output reg                       dout = 0,
    output wire signed [WORDLENGTH-1:0] din_held_debug
);

localparam GUARD_BITS = 8;
localparam ACCLENGTH  = WORDLENGTH + GUARD_BITS;

// Quantizer levels: ±2^WORDLENGTH in ACCLENGTH-bit space
localparam signed [ACCLENGTH-1:0] up_inc   = {{(ACCLENGTH-WORDLENGTH-1){1'b0}}, 1'b1, {WORDLENGTH{1'b0}}};
localparam signed [ACCLENGTH-1:0] down_inc = -up_inc;

// Integrator state — one per stage
reg signed [ACCLENGTH-1:0] sigma [0:ORDER-1];

// Registered quantizer bits — one per stage
reg [ORDER-1:0] q_bit = 0;

// Quantizer feedback as up/down_inc — computed combinatorially from q_bit
wire signed [ACCLENGTH-1:0] q_fb [0:ORDER-1];
genvar gi;
generate
    for (gi = 0; gi < ORDER; gi = gi + 1) begin : fb_gen
        assign q_fb[gi] = q_bit[gi] ? up_inc : down_inc;
    end
endgenerate

reg signed [WORDLENGTH-1:0] din_held = 0;
assign din_held_debug = din_held;

// Input TPDF dither: two uniform PRNG values summed → triangular PDF.
// Peak = ±2^DITHER_BITS added at input scale. Keep DITHER_BITS small
// (8-16) to avoid raising the in-band noise floor.
localparam DITHER_BITS = 24;
wire signed [DITHER_BITS:0] dither_tpdf = $signed(dither1[DITHER_BITS-1:0]) + $signed(dither2[DITHER_BITS-1:0]);

integer k;
integer j;

initial begin
    for (j = 0; j < ORDER; j = j + 1)
        sigma[j] = 0;
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        for (k = 0; k < ORDER; k = k + 1)
            sigma[k] <= {ACCLENGTH{1'b0}};
        q_bit    <= {ORDER{1'b0}};
        din_held <= {WORDLENGTH{1'b0}};
        dout     <= 1'b0;
    end else begin
        if (dvalid)
            din_held <= din;

        // Stage 0: integrates (input + dither) minus its own feedback
        sigma[0] <= sigma[0] + din_held + dither_tpdf - q_fb[0];

        // Stages 1..ORDER-1: each integrates the previous stage's
        // quantized output minus its own feedback (error-shaping chain)
        for (k = 1; k < ORDER; k = k + 1)
            sigma[k] <= sigma[k] + q_fb[k-1] - q_fb[k];

        // Quantize all stages (reads sigma values from previous cycle)
        for (k = 0; k < ORDER; k = k + 1)
            q_bit[k] <= (sigma[k] >= 0) ? 1'b1 : 1'b0;

        // Final output is the last stage's quantizer
        dout <= q_bit[ORDER-1];
    end
end

endmodule

`default_nettype wire
