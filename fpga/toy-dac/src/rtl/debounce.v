`timescale 1ns / 1ps
`default_nettype none

// debounce
// ────────
// Simple time-based switch debouncer. `btn_out` only follows
// `btn_in` after the input has held a stable level for DEBOUNCE_MS
// milliseconds (clocked off CLK_FREQ). Any glitch resets the
// stability counter, so contact bounce (typically <5 ms on tactile
// switches) is filtered out cleanly.

module debounce #(
    parameter CLK_FREQ = 108_000_000,
    parameter DEBOUNCE_MS = 20
)(
    input  wire clk,
    input  wire rst,
    input  wire btn_in,
    output reg  btn_out = 0
);

    localparam COUNT_MAX = CLK_FREQ / 1000 * DEBOUNCE_MS - 1;
    localparam CW = $clog2(COUNT_MAX + 1);

    reg [CW-1:0] count = 0;
    reg btn_sync0 = 0;
    reg btn_sync1 = 0;

    // Two-FF synchronizer
    always @(posedge clk) begin
        btn_sync0 <= btn_in;
        btn_sync1 <= btn_sync0;
    end

    always @(posedge clk) begin
        if (rst) begin
            count   <= 0;
            btn_out <= 0;
        end else if (btn_sync1 != btn_out) begin
            if (count == COUNT_MAX[CW-1:0]) begin
                btn_out <= btn_sync1;
                count   <= 0;
            end else begin
                count <= count + 1;
            end
        end else begin
            count <= 0;
        end
    end

endmodule

`default_nettype wire
