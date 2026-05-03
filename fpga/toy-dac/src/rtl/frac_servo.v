`timescale 1ns / 1ps
`default_nettype none

// frac_servo
// ──────────
// Pure-proportional servo for the fractional-phase ASRC.
//
// The plant is an integrator: the input ring buffer fills at the
// producer's rate (Fs_in_producer) and drains at the rate the DUT
// consumes samples, which is
//
//     Fs_consumed = step · Fs_out / 2^32
//
// where `step` is the Q0.32 fractional-phase increment fed to
// `fractional_asrc.step` and Fs_out is the fixed output strobe rate
// (mclk / OUT_DIV).  This module reads the input FIFO fill level,
// runs a P controller against a setpoint of FIFO_DEPTH/2, and emits
// `step = step_nominal + step_adj`, clamped to a safe range.
//
// Sign convention
// ───────────────
//   err = fifo_count − SETPOINT
//     err > 0  ⇒  buffer too full  ⇒  consume faster ⇒  step ↑
//     err < 0  ⇒  buffer too empty ⇒  consume slower ⇒  step ↓
//


module frac_servo #(
    parameter integer FIFO_DEPTH         = 64,
    parameter integer SETPOINT           = FIFO_DEPTH/2,
    parameter integer MCLK_HZ            = 108_000_000,
    parameter integer UPDATE_HZ          = 2,
    parameter integer KP                 = 256,
    parameter integer ERR_DEADBAND       = 16,
    parameter signed [31:0] STEP_ADJ_MAX =  32'sd16777216,   //  +2^24
    parameter signed [31:0] STEP_ADJ_MIN = -32'sd16777216,   //  -2^24

    // Output slew-rate limiter. The P loop is allowed to compute a
    // new target every UPDATE_HZ tick, but the step value handed to
    // the ASRC is only allowed to change by 1 LSB per
    // STEP_SLEW_PERIOD mclks. This converts every P "jump" into a
    // smooth ramp, eliminating the FM impulse that an instantaneous
    // step change would inject around the audio carrier.
    //   default 4096 mclks / LSB @ 108 MHz ≈  38 µs per LSB
    //                                      ≈  26 k LSB / s slew
    //                                      ≈  10 Hz / s frequency-chirp ceiling
    // (1 LSB of step = Fs_out/2^32 ≈ 3.93e-4 Hz of consumed-rate change)
    parameter integer STEP_SLEW_PERIOD   = 16384,

    // LED / `adjust` quantisation. The output `adjust` pulse fires
    // only when the smoothed step crosses an N-LSB boundary, so the
    // LED blinks ~once per `2^ADJUST_QUANTUM_LOG2` LSB of cumulative
    // step movement instead of once per P update. Default = 14 →
    // one pulse per 16384 LSB ≈ 6.4 Hz of consumed-rate adjustment,
    // i.e. "big enough that you'd actually want to know about it".
    parameter integer ADJUST_QUANTUM_LOG2 = 14
)(
    input  wire                              clk,
    input  wire                              rst,
    input  wire                              enable,

    // Nominal step (Q0.32) from rate_manager.  step_nominal_in == 0
    // means "use STEP_NOMINAL parameter" (none is supplied here as a
    // hard default; the wrapper is expected to drive a real value).
    input  wire        [31:0]                step_nominal_in,

    // FIFO fill level (input side of the ASRC).
    input  wire [$clog2(FIFO_DEPTH+1)-1:0]   fifo_count,

    // Effective step to feed fractional_asrc.step.
    output wire        [31:0]                step,

    // Diagnostics
    output wire signed [15:0]                dbg_error,
    output wire signed [31:0]                dbg_step_adj,
    output reg                               adjust = 1'b0
);

    // ── P state ─────────────────────────────────────────────────
    reg signed [31:0] pi_out  = 32'sd0;

    // ── Update timer ────────────────────────────────────────────
    localparam integer UPDATE_DIV = MCLK_HZ / UPDATE_HZ;
    localparam integer DIV_W      = $clog2(UPDATE_DIV);
    reg [DIV_W-1:0] upd_cnt = 0;
    wire upd_tick = (upd_cnt == UPDATE_DIV - 1);

    // ── Error + deadband ────────────────────────────────────────
    wire signed [15:0] err_raw = $signed({1'b0, fifo_count})
                               - $signed(SETPOINT[15:0]);
    wire signed [15:0] err_c =
        (err_raw >  $signed(ERR_DEADBAND[15:0])) ? err_raw :
        (err_raw < -$signed(ERR_DEADBAND[15:0])) ? err_raw :
        16'sd0;

    // ── Pipelined P compute ─────────────────────────────────────
    // The P chain (subtract → deadband → 16×16 mul → clamp) is
    // short, but we keep it pipelined into 2 registered stages
    // (err_q → prop_q → pi_n_q) so the timing margin against the
    // 108 MHz mclk stays comfortable. Updates fire only every
    // UPDATE_DIV cycles, so the few-cycle latency is irrelevant.
    reg signed [15:0]  err_q  = 16'sd0;
    reg signed [31:0]  prop_q = 32'sd0;
    reg signed [31:0]  pi_n_q = 32'sd0;

    wire signed [31:0] pi_clamped =
        (prop_q > STEP_ADJ_MAX) ? STEP_ADJ_MAX :
        (prop_q < STEP_ADJ_MIN) ? STEP_ADJ_MIN :
        prop_q;

    always @(posedge clk) begin
        if (rst || !enable) begin
            err_q  <= 16'sd0;
            prop_q <= 32'sd0;
            pi_n_q <= 32'sd0;
        end else begin
            err_q  <= err_c;
            prop_q <= $signed(KP[15:0]) * err_q;
            pi_n_q <= pi_clamped;
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            upd_cnt <= 0;
            pi_out  <= 32'sd0;
        end else if (!enable) begin
            // Hold step at nominal (zero adjustment).
            upd_cnt <= 0;
            pi_out  <= 32'sd0;
        end else begin
            upd_cnt <= upd_tick ? {DIV_W{1'b0}} : upd_cnt + 1'b1;
            if (upd_tick)
                pi_out <= pi_n_q;
        end
    end

    // ── Output slew-rate limiter ────────────────────────────────
    // pi_out is the "target" the P loop wants. pi_smooth is what we
    // actually hand to the ASRC, and it ramps toward pi_out by 1 LSB
    // per STEP_SLEW_PERIOD mclks. Any single P jump is therefore
    // applied as a smooth chirp over (|delta| × STEP_SLEW_PERIOD)
    // mclks instead of a single-cycle discontinuity.
    localparam integer SLEW_W = $clog2(STEP_SLEW_PERIOD);
    reg [SLEW_W-1:0]  slew_cnt  = {SLEW_W{1'b0}};
    reg signed [31:0] pi_smooth = 32'sd0;
    wire slew_tick = (slew_cnt == STEP_SLEW_PERIOD - 1);

    always @(posedge clk) begin
        if (rst || !enable) begin
            slew_cnt  <= {SLEW_W{1'b0}};
            pi_smooth <= 32'sd0;
        end else begin
            slew_cnt <= slew_tick ? {SLEW_W{1'b0}} : slew_cnt + 1'b1;
            if (slew_tick) begin
                if      (pi_smooth < pi_out) pi_smooth <= pi_smooth + 32'sd1;
                else if (pi_smooth > pi_out) pi_smooth <= pi_smooth - 32'sd1;
            end
        end
    end

    // ── `adjust` LED: hunting indicator ─────────────────────────
    // Lights only when pi_smooth_q REVERSES direction (real hunting),
    // not while it monotonically slews toward a new target. A long
    // monotonic ramp — which is the normal lock-acquisition behaviour
    // when the slew limiter is slow — leaves the LED dark.
    //
    // State machine on the sign of (pi_smooth_q − prev):
    //   last_dir ∈ {+1, −1, unknown}
    //   When we see motion, compare its sign to last_dir.
    //     same / unknown  → not hunting, just update last_dir.
    //     opposite        → reversal: pulse adjust, update last_dir.
    //   No motion does not change last_dir, so a brief pause followed
    //   by resumed motion in the same direction stays "not hunting".
    //
    // Each reversal stretches the LED for HOLD_CYCLES so it's visible.
    wire signed [31:0] pi_smooth_q      = pi_smooth >>> ADJUST_QUANTUM_LOG2;
    reg  signed [31:0] pi_smooth_q_prev = 32'sd0;
    reg                last_dir         = 1'b0;   // 0 = down, 1 = up
    reg                last_dir_valid   = 1'b0;

    // Pulse stretch: ~100 ms at MCLK_HZ.
    localparam integer HOLD_CYCLES = MCLK_HZ / 10;
    localparam integer HOLD_W      = $clog2(HOLD_CYCLES + 1);
    reg [HOLD_W-1:0]   hold_cnt    = {HOLD_W{1'b0}};

    wire moved_up   = (pi_smooth_q >  pi_smooth_q_prev);
    wire moved_down = (pi_smooth_q <  pi_smooth_q_prev);
    wire reversal   = last_dir_valid &&
                      (( moved_up   && last_dir == 1'b0) ||
                       ( moved_down && last_dir == 1'b1));

    always @(posedge clk) begin
        if (rst || !enable) begin
            pi_smooth_q_prev <= 32'sd0;
            last_dir         <= 1'b0;
            last_dir_valid   <= 1'b0;
            hold_cnt         <= {HOLD_W{1'b0}};
            adjust           <= 1'b0;
        end else begin
            pi_smooth_q_prev <= pi_smooth_q;

            if (moved_up) begin
                last_dir       <= 1'b1;
                last_dir_valid <= 1'b1;
            end else if (moved_down) begin
                last_dir       <= 1'b0;
                last_dir_valid <= 1'b1;
            end

            if (reversal)
                hold_cnt <= HOLD_CYCLES[HOLD_W-1:0];
            else if (hold_cnt != {HOLD_W{1'b0}})
                hold_cnt <= hold_cnt - 1'b1;

            adjust <= (hold_cnt != {HOLD_W{1'b0}});
        end
    end

    // ── Compose effective step: step_nominal + pi_smooth ─────────
    // Add as 33-bit signed then truncate; step_nominal_in is unsigned
    // Q0.32 and pi_smooth is the slew-rate-limited signed adjustment.
    wire signed [32:0] step_sum = $signed({1'b0, step_nominal_in}) + pi_smooth;
    assign step = step_sum[31:0];

    assign dbg_error    = err_q;
    assign dbg_step_adj = pi_smooth;

endmodule

`default_nettype wire
