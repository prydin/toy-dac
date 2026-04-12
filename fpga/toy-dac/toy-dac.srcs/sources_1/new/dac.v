`timescale 1ns / 1ps
`default_nettype none


module dac #(
	parameter WORDLENGTH = 32
)(

	input wire clk,
	input wire rst,
	input wire signed [WORDLENGTH-1:0] din,
    input wire dvalid, 
	output reg dout = 0,
	output wire signed [WORDLENGTH-1:0] din_held_debug
);

assign din_held_debug = din_held;

localparam ACCLENGTH = WORDLENGTH + 8; // Guard bits for integrator headroom

// 2**WORDLENGTH computed in ACCLENGTH bits to avoid 32-bit overflow
localparam signed [ACCLENGTH-1:0] up_inc = {{(ACCLENGTH-WORDLENGTH-1){1'b0}}, 1'b1, {WORDLENGTH{1'b0}}};
localparam signed [ACCLENGTH-1:0] down_inc = -up_inc; 
reg signed [ACCLENGTH-1:0] sigma = 0; 
wire signed [ACCLENGTH-1:0] q_feedback = (dout ? up_inc : down_inc);

reg signed [WORDLENGTH-1:0] din_held = 0;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        sigma <= {ACCLENGTH{1'b0}}; 
        dout <= 1'b0;
        din_held <= {WORDLENGTH{1'b0}};
    end else begin
        if (dvalid)
            din_held <= din;
        sigma <= sigma + (din_held - q_feedback); 
        dout <= (sigma >= 0) ? 1'b1 : 1'b0;
    end
end 


endmodule
