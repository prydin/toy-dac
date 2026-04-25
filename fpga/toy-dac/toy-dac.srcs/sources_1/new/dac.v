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
