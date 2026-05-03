`timescale 1ns / 1ps
`default_nettype none

// i2s
// ───
// I2S receiver. Synchronizes the asynchronous bclk / lrclk / din
// pins to the local mclk domain via 2-FF synchronizers (`flop_sync`,
// defined below), shifts data bits in on each bclk rising edge, and
// emits one signed sample per channel per audio frame.
//
// Output handshake follows AXI-Stream tvalid/tready conventions:
// `*_valid` rises when a new sample is latched and stays HIGH until
// the corresponding `*_ready` accepts it (clearing it on the next
// cycle). With both ready inputs tied HIGH the valid lines collapse
// to 1-cycle accept pulses, which is what the downstream fractional
// `asrc` wants.
//
// I2S timing convention (standard — not left-justified):
//   * Data bit is launched on bclk falling edge by the source.
//   * MSB of each channel arrives ONE bclk after the lrclk transition
//     (the "one-bit delay" rule). The shift register is left-shifted
//     by one extra position on output to undo this.
//
// Edge-pulse outputs (`bclk_pos_edge`, `lrclk_pos_edge`, etc.) are
// exposed so callers (e.g. rate_detect, frame-integrity probes in
// root.v) can reuse the same synchronizers instead of re-syncing
// the raw pins themselves.

module flop_sync (
    input wire clk,
    input wire rst,
    input wire in,
    output reg out,
    output wire neg_edge,
    output wire pos_edge
);
reg sync_1;
reg prev_out;
always @(posedge clk or posedge rst) begin
    if (rst) begin
        sync_1 <= 0;
        out <= 0;
    end else begin
        sync_1 <= in; // First stage of synchronization
        out <= sync_1; // Second stage of synchronization
        prev_out <= out;
    end
end

// Edge detection logic
assign neg_edge = ~out & prev_out;
assign pos_edge = out & ~prev_out;

endmodule

module i2s #(
    parameter WORDLENGTH = 32
    )(
    input wire                   clk,               // 100MHz clock from crystal
    input wire                   rst,               // Reset signal
    input wire                   bclk,              // Bit clock for I2S
    input wire                   lrclk,             // Left-right clock for I2S
    input  wire                  din,               // Serial data input for I2S
    output reg signed [WORDLENGTH-1:0]  out_left = 0,    // Left channel word
    output reg signed [WORDLENGTH-1:0]  out_right = 0,   // Right channel word
    output reg                   input_active = 0,  // Indicates if at least one sample has been received
    output reg                   left_valid = 0,    // AXI-Stream tvalid: held HIGH until left_ready
    output reg                   right_valid = 0,   // AXI-Stream tvalid: held HIGH until right_ready
    input wire                   left_ready,        // AXI-Stream tready from downstream
    input wire                   right_ready,       // AXI-Stream tready from downstream
    output wire clean_bclk,                         // Cleaned and synchronized bit clock
    output wire clean_lrclk,                        // Cleaned and synchronized left-right clock
    output wire bclk_neg_edge,                      // Pulses HIGH for one clk cycle on bclk falling edge
    output wire bclk_pos_edge,                      // Pulses HIGH for one clk cycle on bclk rising edge
    output wire lrclk_neg_edge,                     // Pulses HIGH for one clk cycle on lrclk falling edge
    output wire lrclk_pos_edge                      // Pulses HIGH for one clk cycle on lrclk rising edge
);

wire bclk_sync;
wire lrclk_sync;
wire din_sync;
assign clean_bclk = bclk_sync;
assign clean_lrclk = lrclk_sync;


flop_sync bclk_flop (
    .clk(clk),
    .rst(1'b0), 
    .in(bclk),
    .out(bclk_sync),
    .neg_edge(bclk_neg_edge),
    .pos_edge(bclk_pos_edge)
);

flop_sync lrclk_flop (
    .clk(clk),
    .rst(1'b0), 
    .in(lrclk),
    .out(lrclk_sync),
    .neg_edge(lrclk_neg_edge),
    .pos_edge(lrclk_pos_edge)
);

flop_sync din_flop (
    .clk(clk),
    .rst(1'b0), 
    .in(din),
    .out(din_sync)
);

reg signed [WORDLENGTH-1:0] shift_left  = 0; // Shift register for left channel
reg signed [WORDLENGTH-1:0] shift_right = 0; // Shift register for right channel

always @(posedge clk) begin
    if (left_valid && left_ready)
        left_valid <= 1'b0;
    if (right_valid && right_ready)
        right_valid <= 1'b0;
    if(rst) begin
        out_left <= 0;
        out_right <= 0;
        shift_left <= 0;
        shift_right <= 0;
        input_active <= 1'b0;
    end else if (bclk_pos_edge) begin   // Detect rising edge of bclk
        if (lrclk_sync == 0) begin                // Left channel
            shift_left  <= {shift_left[WORDLENGTH-2:0], din_sync};   // Shift in new bit
        end else begin                        // Right channel
            shift_right <= {shift_right[WORDLENGTH-2:0], din_sync};  // Shift in new bit
        end
    end else if (lrclk_pos_edge) begin    // lrclk rising: left channel ready
        out_left    <= shift_left << 1;     // Shift left by 1 to compensate for I2S one-bit delay
        shift_left  <= 0;                   // Clear left shift register after output
        input_active <= 1'b1;               // Mark that we have received at least one sample
        left_valid <= 1'b1;
    end else if (lrclk_neg_edge) begin // lrclk falling: right channel ready
        out_right   <= shift_right << 1;    // Shift left by 1 to compensate for I2S one-bit delay
        shift_right <= 0;                   // Clear right shift register after output
        input_active <= 1'b1;               // Mark that we have received at least one sample
        right_valid <= 1'b1;
    end
end    
endmodule