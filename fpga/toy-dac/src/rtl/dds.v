`timescale 1ns / 1ps
`default_nettype none

// -----------------------------------------------------------------------------
// dds — IP-free replacement for the Xilinx DDS Compiler instance.
//
// Generates a fixed ~1 kHz sine wave at the aclk rate (assumed 108 MHz).
//
//   Phase accumulator : 32 bits
//   Increment         : 39_768  → 39768 * 108e6 / 2^32 ≈ 999.9954 Hz
//   Sine table        : 1024 entries × 18-bit signed (full period, no quarter-
//                       wave folding — simple and small enough for a single
//                       block-RAM or distributed-RAM inferral)
//   Output            : sign-extended/scaled to 26-bit signed in tdata[25:0],
//                       upper 6 bits zero — bit-compatible with the IP that
//                       was previously instantiated here.
//   tvalid            : tied high (one new sample per aclk, same as the IP
//                       was configured for).
//
// The downstream code in root.v takes tdata[25:0] and left-shifts it by 5
// into the 32-bit modulator input.  For the tone to sit at the intended
// ~-6 dBFS (peak ~+-2^30), tdata[25:0] must be FULL-SCALE 26-bit signed
// (~+-2^25).  The LUT is generated at its natural 18-bit full-scale
// (+-(2^17 - 1)); we left-shift it by 8 when packing into tdata so it
// reaches full 26-bit scale.  (Merely sign-extending 18 -> 26 leaves the
// tone at +-2^22 ~ -54 dBFS, ~48 dB too quiet.)
// -----------------------------------------------------------------------------
`default_nettype none

module dds (
    input  wire         aclk,
    output wire         m_axis_data_tvalid,
    output wire [31:0]  m_axis_data_tdata
);

    // Phase increment for ~1 kHz at 108 MHz aclk.
    // round(2^32 * 1000 / 108_000_000) = 39_768
    localparam [31:0] PHASE_INC = 32'd39_768;

    reg [31:0] phase = 32'd0;

    always @(posedge aclk) begin
        phase <= phase + PHASE_INC;
    end

    // 1024-entry × 18-bit signed sine LUT, populated at elaboration time.
    reg signed [17:0] sine_lut [0:1023];
    integer i;
    real    pi;
    initial begin
        pi = 3.14159265358979323846;
        for (i = 0; i < 1024; i = i + 1) begin
            // Full-scale 18-bit signed: ±(2^17 - 1) = ±131071
            sine_lut[i] = $rtoi($sin(2.0 * pi * i / 1024.0) * 131071.0);
        end
    end

    // Phase-to-amplitude with LINEAR INTERPOLATION.
    // A bare 10-bit-address lookup truncates the 32-bit phase to its top
    // 10 bits, producing phase-truncation spurs at only ~ -6.02*10 = -60
    // dBc — a visible comb on the analyzer. Interpolating between the two
    // straddling LUT entries with the next 12 fractional phase bits lifts
    // the effective phase resolution toward the full accumulator width;
    // the residual (2nd-order interpolation error over a 1024-point table)
    // sits near -100 dBc, clean enough to use as a jitter/spur reference.
    wire [9:0]  addr0 = phase[31:22];
    wire [9:0]  addr1 = addr0 + 10'd1;      // wraps mod 1024 (sine is periodic)
    wire [11:0] frac  = phase[21:10];       // 12-bit fraction between entries

    reg signed [17:0] s0     = 18'sd0;
    reg signed [17:0] s1     = 18'sd0;
    reg        [11:0] frac_q = 12'd0;
    always @(posedge aclk) begin
        s0     <= sine_lut[addr0];
        s1     <= sine_lut[addr1];
        frac_q <= frac;
    end

    // interp = s0 + (s1 - s0) * frac_q / 2^12   (Q12 fractional blend)
    wire signed [18:0] diff   = s1 - s0;
    wire signed [31:0] prod   = diff * $signed({1'b0, frac_q});
    wire signed [17:0] interp = s0 + (prod >>> 12);

    reg signed [17:0] sine_q = 18'sd0;
    always @(posedge aclk) begin
        sine_q <= interp;
    end

    // Scale the 18-bit signed sample up to full-scale 26-bit signed by
    // left-shifting 8 (= x256), so tdata[25:0] spans ~+-2^25.  root.v then
    // left-shifts by 5 more to place the tone at ~+-2^30 ~ -6 dBFS at the
    // modulator input.  An earlier version sign-extended 18 -> 26 instead,
    // leaving the tone at ~+-2^22 ~ -54 dBFS (showed up as a ~-50 dBV test
    // tone on the analyzer).  Upper 6 bits of tdata are zero, matching the
    // IP's 26-in-32 packing.
    wire signed [25:0] sine_26 = {sine_q, 8'b0};

    assign m_axis_data_tdata  = {6'b0, sine_26};
    assign m_axis_data_tvalid = 1'b1;

endmodule

`default_nettype wire
