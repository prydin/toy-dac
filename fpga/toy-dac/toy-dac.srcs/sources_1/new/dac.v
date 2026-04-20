`timescale 1ns / 1ps
`default_nettype none


module dac #(
	parameter WORDLENGTH = 32
)(

	input wire clk,
	input wire rst,
	input wire signed [WORDLENGTH-1:0] din,
    input wire dvalid, 
    input wire [31:0] dither1,
    input wire [31:0] dither2,
    output reg dout = 0,
    output wire signed [WORDLENGTH-1:0] din_held_debug
);

assign din_held_debug = din_held;

localparam GUARD_BITS = 8; // Number of guard bits for integrator headroom
localparam ACCLENGTH = WORDLENGTH + GUARD_BITS; // Guard bits for integrator headroom

// 2**WORDLENGTH computed in ACCLENGTH bits to avoid 32-bit overflow
localparam signed [ACCLENGTH-1:0] up_inc = {{(ACCLENGTH-WORDLENGTH-1){1'b0}}, 1'b1, {WORDLENGTH{1'b0}}};
localparam signed [ACCLENGTH-1:0] down_inc = -up_inc; 
reg signed [ACCLENGTH-1:0] sigma = 0; 
// First-stage feedback (maps first-stage bitstream to signed value)
        reg signed [ACCLENGTH-1:0] q_feedback1;

// Second sigma-delta stage (cascaded)
reg signed [ACCLENGTH-1:0] sigma2 = 0;
reg dout1 = 0; // first stage internal bitstream
        reg signed [ACCLENGTH-1:0] first_out_signed;
// Second-stage feedback (maps final module bitstream to signed value)
wire signed [ACCLENGTH-1:0] q_feedback2 = (dout ? up_inc : down_inc);

reg signed [WORDLENGTH-1:0] din_held = 0;

// Dither applied at the comparator (quantizer) input to break limit cycles.
// The resulting perturbation is quantization error, which the NTF (1-z^-1)^2
// high-pass shapes to ultrasonic frequencies. This lets us use enough amplitude
// to actually matter without colouring the audio band.
localparam DITHER_BITS = 8; // Number of dither bits (tradeoff: more bits = better noise shaping but more FPGA resources)
wire signed [DITHER_BITS-1:0] dither1_s = $signed(dither1[DITHER_BITS-1:0]);
wire signed [DITHER_BITS-1:0] dither2_s = $signed(dither2[DITHER_BITS-1:0]);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        sigma <= {ACCLENGTH{1'b0}}; 
        sigma2 <= {ACCLENGTH{1'b0}};
        dout <= 1'b0;
        dout1 <= 1'b0;
        din_held <= {WORDLENGTH{1'b0}};
    end else begin
        if (dvalid)
            din_held <= din;

        // First stage: integrate input - feedback (clean accumulator)
        q_feedback1 = (dout1 ? up_inc : down_inc);
        first_out_signed = q_feedback1;
        sigma <= sigma + (din_held - q_feedback1);
        // First-stage bitstream: dither the comparator threshold
        dout1 <= ((sigma + dither1_s) >= 0) ? 1'b1 : 1'b0;

        // Second stage: integrate the first-stage bitstream (clean accumulator)
        sigma2 <= sigma2 + (first_out_signed - q_feedback2);
        // Final output bitstream: dither the comparator threshold
        dout <= ((sigma2 + dither2_s) >= 0) ? 1'b1 : 1'b0;
    end
end 


endmodule
