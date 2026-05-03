`timescale 1ns / 1ps

//
// Simulation-only 2nd-order Butterworth lowpass (biquad IIR).
// Uses real-number arithmetic — not synthesizable.
//
// Default coefficients: fc ≈ 100 kHz, fs = 128 MHz.
// Computed via bilinear transform.
//

module lpf_biquad (
    input  wire        clk,
    input  wire        rst,
    input  wire signed [31:0] din,
    output reg  signed [31:0] dout
);

    // ── Butterworth 2nd-order LPF coefficients ──
    // fc = 100 kHz, fs = 128 MHz
    // K  = tan(π · fc / fs) ≈ 0.002454374
    // Denominator = 1 + √2·K + K²
    localparam real B0 =  6.0035e-6;
    localparam real B1 =  1.2007e-5;
    localparam real B2 =  6.0035e-6;
    localparam real A1 = -1.993069;
    localparam real A2 =  0.993083;

    real x1, x2;       // input delay line
    real y1, y2;        // output delay line

    initial begin
        x1 = 0.0; x2 = 0.0;
        y1 = 0.0; y2 = 0.0;
    end

    always @(posedge clk) begin
        if (rst) begin
            x1   <= 0.0; x2 <= 0.0;
            y1   <= 0.0; y2 <= 0.0;
            dout <= 32'sd0;
        end else begin : filter
            real x0, y0;
            x0 = $itor(din);
            y0 = B0*x0 + B1*x1 + B2*x2 - A1*y1 - A2*y2;

            // shift delay line
            x2 <= x1;  x1 <= x0;
            y2 <= y1;  y1 <= y0;

            dout <= $rtoi(y0);
        end
    end

endmodule
