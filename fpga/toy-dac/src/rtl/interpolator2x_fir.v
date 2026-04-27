`timescale 1ns / 1ps
`default_nettype none

// Polyphase half-band x2 interpolation filter.
//
// 33-tap half-band FIR (Blackman-windowed sinc, ~58 dB stopband).
// For each input sample, outputs two samples at 2x the input rate:
//   1. Interpolated (FIR-filtered value between adjacent input samples)
//   2. Passthrough  (original input sample, delayed by group delay)
//
// Polyphase decomposition exploits two half-band properties:
//   - Phase 0 (passthrough): all even taps are zero except center = 0.5,
//     so the output is simply x[n - 8].  No multiply needed.
//   - Phase 1 (interpolation): 16 non-zero odd taps, symmetric, so only
//     8 unique multiply-accumulates using pre-added symmetric pairs.
//
// Port interface matches interpolator2x for drop-in replacement.

module interpolator2x_fir #(
    parameter WIDTH = 32
)(
    input  wire                    aclk,
    // AXI-Stream slave (input)
    input  wire                    s_axis_data_tvalid,
    output wire                    s_axis_data_tready,
    input  wire signed [WIDTH-1:0] s_axis_data_tdata,
    // AXI-Stream master (output)
    output reg                     m_axis_data_tvalid = 0,
    output reg  signed [WIDTH-1:0] m_axis_data_tdata  = 0
);

// ── Filter geometry ──
localparam NUM_UNIQUE = 8;              // unique non-zero half-band coefficients
localparam SR_LEN    = 2 * NUM_UNIQUE;  // 16-deep input shift register
localparam FRAC      = 17;              // coefficient fractional bits (2^17 scale)
localparam CW        = 18;              // coefficient bit width

// ── Coefficients ──
// Blackman-windowed sinc, scaled by 2^17.  c[0] = nearest center, c[7] = outermost.
// Verified: 2 * sum(c[0..7]) = 131086 ≈ 2^17 (0.01% from unity gain).
localparam signed [CW-1:0] C0 =  18'sd82133;   // h[15], h[17]
localparam signed [CW-1:0] C1 = -18'sd24097;   // h[13], h[19]
localparam signed [CW-1:0] C2 =  18'sd11134;   // h[11], h[21]
localparam signed [CW-1:0] C3 = -18'sd5288;    // h[9],  h[23]
localparam signed [CW-1:0] C4 =  18'sd2304;    // h[7],  h[25]
localparam signed [CW-1:0] C5 =  -18'sd847;    // h[5],  h[27]
localparam signed [CW-1:0] C6 =   18'sd224;    // h[3],  h[29]
localparam signed [CW-1:0] C7 =   -18'sd20;    // h[1],  h[31]

// ── Shift register ──
// sr[0] = most recently stored sample, sr[15] = oldest.
// Updated with <= on clock edge; combinational logic reads pre-shift values.
reg signed [WIDTH-1:0] sr [0:SR_LEN-1];

// ── State ──
// phase 0: accept new input, compute & output interpolated sample
// phase 1: output passthrough (delayed original)
reg phase = 0;

assign s_axis_data_tready = ~phase;

// ── Symmetric pair pre-addition (combinational) ──
// After the pending shift, new_sr[0] = input, new_sr[k] = sr[k-1].
// Pairs: new_sr[k] + new_sr[15-k]
wire signed [WIDTH:0] pair0 = s_axis_data_tdata + sr[14];  // new_sr[0]  + new_sr[15]
wire signed [WIDTH:0] pair1 = sr[0]  + sr[13];             // new_sr[1]  + new_sr[14]
wire signed [WIDTH:0] pair2 = sr[1]  + sr[12];             // new_sr[2]  + new_sr[13]
wire signed [WIDTH:0] pair3 = sr[2]  + sr[11];             // new_sr[3]  + new_sr[12]
wire signed [WIDTH:0] pair4 = sr[3]  + sr[10];             // new_sr[4]  + new_sr[11]
wire signed [WIDTH:0] pair5 = sr[4]  + sr[9];              // new_sr[5]  + new_sr[10]
wire signed [WIDTH:0] pair6 = sr[5]  + sr[8];              // new_sr[6]  + new_sr[9]
wire signed [WIDTH:0] pair7 = sr[6]  + sr[7];              // new_sr[7]  + new_sr[8]

// ── Multiply-accumulate (combinational) ──
wire signed [WIDTH+CW:0] prod0 = pair0 * C0;
wire signed [WIDTH+CW:0] prod1 = pair1 * C1;
wire signed [WIDTH+CW:0] prod2 = pair2 * C2;
wire signed [WIDTH+CW:0] prod3 = pair3 * C3;
wire signed [WIDTH+CW:0] prod4 = pair4 * C4;
wire signed [WIDTH+CW:0] prod5 = pair5 * C5;
wire signed [WIDTH+CW:0] prod6 = pair6 * C6;
wire signed [WIDTH+CW:0] prod7 = pair7 * C7;

wire signed [WIDTH+CW+3:0] acc = prod0 + prod1 + prod2 + prod3
                                + prod4 + prod5 + prod6 + prod7;

// Truncate to WIDTH bits by discarding FRAC fractional bits
wire signed [WIDTH-1:0] interp_out = acc[FRAC+WIDTH-1 : FRAC];

// ── Registered passthrough value ──
// Captured from sr[7] (= x[n-8]) before the shift register updates.
reg signed [WIDTH-1:0] passthrough = 0;

integer i;

always @(posedge aclk) begin
    if (!phase) begin
        if (s_axis_data_tvalid) begin
            // Save passthrough: sr[7] is still x[n-8] (pre-shift)
            passthrough <= sr[7];

            // Shift register: push new sample in at [0]
            for (i = SR_LEN-1; i > 0; i = i - 1)
                sr[i] <= sr[i-1];
            sr[0] <= s_axis_data_tdata;

            // Output interpolated sample
            m_axis_data_tdata  <= interp_out;
            m_axis_data_tvalid <= 1'b1;
            phase <= 1'b1;
        end else begin
            m_axis_data_tvalid <= 1'b0;
        end
    end else begin
        // Output passthrough (original sample, group-delay aligned)
        m_axis_data_tdata  <= passthrough;
        m_axis_data_tvalid <= 1'b1;
        phase <= 1'b0;
    end
end

// ── Initialize shift register to zero (FPGA power-on default) ──
initial begin
    for (i = 0; i < SR_LEN; i = i + 1)
        sr[i] = 0;
end

endmodule
