`timescale 1ns / 1ps
`default_nettype none

// mono_ff
// ───────
// Monostable (one-shot) flip-flop. A rising edge on `d` drives `q`
// HIGH for a programmable delay, then `q` returns to 0. Delay is
// specified in any combination of DELAY_MS / DELAY_US / DELAY_NS
// and converted to clock cycles using FCLK at elaboration time.
//
// RESETTABLE selects retrigger behaviour:
//   0 (default) — ignore further `d` edges while q is HIGH
//                 (non-retriggerable: produces a fixed-width pulse).
//   1           — reload the counter on every `d` edge, extending
//                 the pulse (retriggerable: useful for "activity
//                 LED" indicators).
//
// Used in root.v to stretch the ASRC servo's `adjust` strobe so the
// blue status LED stays on long enough to see.

module mono_ff #(
    parameter FCLK = 108_000_000,
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

// Calculate the total delay in clock cycles at elaboration time as float to avoid rounding issues, then convert to integer for the counter.
localparam real TOTAL_DELAY_S = (DELAY_MS / 1_000.0) + (DELAY_US / 1_000_000.0) + (DELAY_NS / 1_000_000_000.0);
localparam integer EXP_CYC = $rtoi(TOTAL_DELAY_S * FCLK);

reg [31:0] counter = 0;
reg last_d = 0;

always @(posedge clk) begin
    if(rst) begin
        last_d <= 0;
    end else begin
        last_d <= d;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        counter <= 0;
        q <= 0;
    end else if (d && ~last_d && (RESETTABLE || ~q)) begin
        q <= 1; // Capture input after the specified delay
        counter <= EXP_CYC;
    end else if (counter == 0) begin
        q <= 0; // Clear output after delay expires
        counter <= counter; // Hold counter at 0 until next trigger
    end else begin
        counter <= counter - 1; // Decrement counter each clock cycle
    end 
end
endmodule

`default_nettype wire