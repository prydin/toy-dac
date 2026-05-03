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
// The downstream code in root.v sign-extends bit 25 and left-shifts by 5,
// so we want the LUT to span roughly ±2^17 to give a final amplitude of
// about ±2^22 inside the 26-bit field (≈ −24 dBFS), matching what the IP
// produced when configured for ~−24 dBFS output.  We instead size the LUT
// to its natural full-scale (±(2^17 − 1)) and let root.v's left-shift place
// it at full 26-bit scale — that is what the original IP did too (the IP's
// tdata was full-scale 26-bit signed).
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

    // Top 10 bits index the LUT.
    wire [9:0] lut_addr = phase[31:22];

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

    reg signed [17:0] sine_q = 18'sd0;
    always @(posedge aclk) begin
        sine_q <= sine_lut[lut_addr];
    end

    // Place the 18-bit signed sample into the bottom 26 bits as a signed
    // value (sign-extend from 18 → 26).  Upper 6 bits of tdata are zero,
    // matching the IP's 26-bit-in-32-bit packing.
    wire signed [25:0] sine_26 = {{8{sine_q[17]}}, sine_q};

    assign m_axis_data_tdata  = {6'b0, sine_26};
    assign m_axis_data_tvalid = 1'b1;

endmodule

`default_nettype wire
