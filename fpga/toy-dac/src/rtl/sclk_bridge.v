`timescale 1ns / 1ps
`default_nettype none

// sclk_bridge
// ───────────
// The single, isolated clock-domain-crossing point between the
// system clock (mclk, from the onboard MMCM — runs rate_detect /
// rate_manager / housekeeping) and the sample pipeline clock (sclk,
// one of the two external audio-family oscillators selected by
// `external_clock`'s BUFGMUX).
//
// Rate switches are rare, debounced, out-of-band events (rate_manager
// requires MATCHES_REQUIRED consecutive matching measurements before
// committing a new `rate_code`), and audio is expected to mute/reset
// across a switch rather than stay glitch-free through it. That lets
// this bridge be a simple mute-switch-settle sequencer instead of a
// continuous data-path CDC:
//
//   1. Watch rm_rate_code (mclk domain) for a committed change to a
//      different clock FAMILY (44.1k vs 48k; 32k/invalid stay on the
//      currently active family since they are muted upstream anyway).
//   2. Flip `clk_sel` and hold `switching` for SETTLE_CYCLES mclk
//      cycles — long enough for the BUFGMUX output to settle and for
//      the oscillator already running to be stable.
//   3. `switching` async-asserts a reset synchronizer in the sclk
//      domain (`pipe_rst`), which the pipeline top-level ORs into
//      its own reset — this flushes the I2S receiver / ASRC ring
//      buffer / DSM state on every family change.
//   4. `family_sel` is a plain 2-FF synchronized copy of `clk_sel`
//      into the sclk domain. SETTLE_CYCLES (ms-scale) is far longer
//      than a 2-FF sync latency, so it is guaranteed stable well
//      before `pipe_rst` deasserts.
//
// Everything else (audio sample data, ASRC, DSM) lives entirely
// inside the sclk domain and never crosses back — there is no
// continuous inter-clock sample stream to bridge.

module sclk_bridge #(
    parameter integer SETTLE_CYCLES = 32'd50_000  // mclk cycles to hold reset after a switch
)(
    input  wire        sys_clk,     // mclk
    input  wire        sys_rst,     // mclk-domain reset (~mclk_locked)
    input  wire [1:0]  rate_code,   // rate_manager's committed classification
    output reg         clk_sel = 1'b0,  // to external_clock's BUFGMUX select

    input  wire        sclk,        // currently-selected pipeline clock
    output wire        pipe_rst,    // sclk-domain reset (assert-async/deassert-sync)
    output wire        family_sel   // sclk-domain copy of clk_sel
);

    localparam integer SETTLE_W = $clog2(SETTLE_CYCLES + 1);

    // rate_code: 0=32k 1=44.1k 2=48k 3=invalid (see rate_manager.v).
    // Only a confirmed 48k classification selects the 48k-family
    // oscillator; everything else (32k/invalid/44.1k) selects the
    // 44.1k-family oscillator. 32k/invalid are muted upstream by
    // rate_manager regardless of which clock is active.
    wire target_family = (rate_code == 2'd2);

    reg [1:0]          rate_code_prev = 2'd1;
    reg                switching      = 1'b0;
    reg [SETTLE_W-1:0] settle_cnt     = {SETTLE_W{1'b0}};

    always @(posedge sys_clk) begin
        if (sys_rst) begin
            clk_sel        <= 1'b0;
            rate_code_prev <= 2'd1;
            switching      <= 1'b0;
            settle_cnt     <= {SETTLE_W{1'b0}};
        end else if (switching) begin
            if (settle_cnt == {SETTLE_W{1'b0}})
                switching <= 1'b0;
            else
                settle_cnt <= settle_cnt - 1'b1;
        end else begin
            rate_code_prev <= rate_code;
            if ((rate_code != rate_code_prev) && (target_family != clk_sel)) begin
                clk_sel    <= target_family;
                switching  <= 1'b1;
                settle_cnt <= SETTLE_CYCLES[SETTLE_W-1:0];
            end
        end
    end

    // Reset synchronizer: assert async the instant a switch begins
    // OR while the system clock itself is still unreset (MMCM not
    // locked yet), release synchronously to sclk once both have
    // been clear for a couple of sclk edges.
    reg [2:0] pipe_rst_sync = 3'b111;
    always @(posedge sclk or posedge switching or posedge sys_rst) begin
        if (switching || sys_rst)
            pipe_rst_sync <= 3'b111;
        else
            pipe_rst_sync <= {pipe_rst_sync[1:0], 1'b0};
    end
    assign pipe_rst = pipe_rst_sync[2];

    reg [1:0] family_sync = 2'b00;
    always @(posedge sclk) family_sync <= {family_sync[0], clk_sel};
    assign family_sel = family_sync[1];

endmodule

`default_nettype wire
