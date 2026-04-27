`timescale 1ns / 1ps
`default_nettype none

// Simple monostable flip-flop 
module mono_ff #(
    parameter FCLK = 54_000_000,
    parameter DELAY_MS = 0,
    parameter DELAY_US = 0,
    parameter DELAY_NS = 0,
    parameter RESETTABLE = 1'b0
) (
    input wire clk,
    input wire rst,
    input wire d,
    output reg q
);

parameter integer EXP_CYC =  (FCLK * DELAY_NS) / 1_000_000_000 + (FCLK * DELAY_US) / 1_000_000 + (FCLK * DELAY_MS) / 1_000;  // Total delay in clock cycles

reg [31:0] counter = 0;
reg last_d = 0;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        counter <= 0;
        q <= 0;
        last_d <= 0;
    end else if (d && ~last_d && (RESETTABLE || ~q)) begin
        last_d <= d; // Update last_d to the new value
        q <= 1; // Capture input after the specified delay
        counter <= EXP_CYC;
    end else if (counter == 0) begin
        q <= 0; // Clear output after delay expires
    end else begin
        counter <= counter - 1; // Decrement counter each clock cycle
    end 
end
endmodule

`default_nettype wire