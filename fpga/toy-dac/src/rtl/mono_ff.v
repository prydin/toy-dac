`timescale 1ns / 1ps
`default_nettype none

// Simple monostable flip-flop 
module mono_ff #(
    parameter FCLK = 54_000_000,
    parameter DELAY = 10_000_000, // Delay in ns for the flip-flop
    parameter RESETTABLE = 1'b0
) (
    input wire clk,
    input wire rst,
    input wire d,
    output reg q
);

reg [31:0] counter = 0;
reg last_d = 0;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        counter <= 0;
        q <= 0;
        last_d <= 0;
    end else begin
        // Always track previous d so edge detection works on every
        // rising edge, not just the first one after reset.
        last_d <= d;
        if (d && ~last_d && (RESETTABLE || ~q)) begin
            q <= 1;
            counter <= (FCLK * DELAY) / 1_000_000_000;
        end else if (counter == 0) begin
            q <= 0;
        end else begin
            counter <= counter - 1;
        end
    end
end
endmodule 

`default_nettype wire