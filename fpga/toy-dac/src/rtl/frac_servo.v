`timescale 1ns / 1ps
`default_nettype none

// frac_servo
// ──────────
// PI servo for the fractional-phase ASRC.
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
// runs a PI controller against a setpoint of FIFO_DEPTH/2, and
// emits `step = step_nominal + step_adj`, clamped to a safe range.
//
// Sign convention
// ───────────────
//   err = fifo_count − SETPOINT
//     err > 0  ⇒  buffer too full  ⇒  consume faster ⇒  step ↑
//     err < 0  ⇒  buffer too empty ⇒  consume slower ⇒  step ↓
//
// Update rate / gains
// ───────────────────
//   The servo updates at UPDATE_HZ (default 2 Hz). Defaults are
//   KP = 8, KI = 0 — a pure-proportional loop. The plant is a pure
//   integrator (FIFO depth = ∫rate_error), so:
//
//     P-only on an integrator plant:  loop gain = KP/s
//                                     → first-order exponential decay,
//                                     → no overshoot by construction.
//     Steady-state error is zero because the plant itself integrates:
//     any nonzero step adjustment drifts the fill, so the only stable
//     equilibrium is err = 0.
//
//   (A pure-integral controller on an integrator plant gives loop
//   gain KI/s² — a harmonic oscillator, marginally stable, rings
//   forever. Don't do that.)
//
//   The slew-rate limiter below dominates the closed-loop dynamics,
//   so the response time is set by STEP_SLEW_PERIOD, not by KP.
//
//   Sensitivity:
//     ∂Fs_consumed / ∂step  =  Fs_out / 2^32
//   For Fs_out = 1.6875 MHz that's ≈ 3.93·10⁻⁴ Hz per step-LSB, so
//   STEP_ADJ_MAX = ±2^24 covers about ±6.6 kHz of input-rate
//   correction — far more than typical crystal drift (< 100 ppm).
//
// `enable` low pauses servo updates and freezes step at step_nominal
// (integrator + proportional both forced to zero).  `rst` clears
// state.

module frac_servo #(
    parameter integer FIFO_DEPTH         = 64,
    parameter integer SETPOINT           = FIFO_DEPTH/2,
    parameter integer MCLK_HZ            = 108_000_000,
    parameter integer UPDATE_HZ          = 2,
    parameter integer KP                 = 8,
    parameter integer KI                 = 0,
    parameter integer ERR_DEADBAND       = 2,
    parameter signed [31:0] STEP_ADJ_MAX =  32'sd16777216,   //  +2^24
    parameter signed [31:0] STEP_ADJ_MIN = -32'sd16777216,   //  -2^24

    // Output slew-rate limiter. The PI loop is allowed to compute a
    // new target every UPDATE_HZ tick, but the step value handed to
    // the ASRC is only allowed to change by 1 LSB per
    // STEP_SLEW_PERIOD mclks. This converts every PI "jump" into a
    // smooth ramp, eliminating the FM impulse that an instantaneous
    // step change would inject around the audio carrier.
    //   default 4096 mclks / LSB @ 108 MHz ≈  38 µs per LSB
    //                                      ≈  26 k LSB / s slew
    //                                      ≈  10 Hz / s frequency-chirp ceiling
    // (1 LSB of step = Fs_out/2^32 ≈ 3.93e-4 Hz of consumed-rate change)
    parameter integer STEP_SLEW_PERIOD   = 4096,

    // LED / `adjust` quantisation. The output `adjust` pulse fires
    // only when the smoothed step crosses an N-LSB boundary, so the
    // LED blinks ~once per `2^ADJUST_QUANTUM_LOG2` LSB of cumulative
    // step movement instead of once per PI update. Default = 14 →
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

    // ── PI state ─────────────────────────────────────────────────
    reg signed [31:0] integ   = 32'sd0;
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

    // ── Pipelined PI compute ────────────────────────────────────
    // The full PI chain (subtract → deadband → 16×16 mul → 48-bit
    // add → clamp → 16×16 mul → add → clamp → compare) is far too
    // long to close at 108 MHz in one cycle (was 31 logic levels,
    // WNS −5.9 ns). Updates fire only every UPDATE_DIV cycles
    // (~1.08 M @ 100 Hz), so we run the same combinational network
    // every cycle but break it into 4 registered stages and snapshot
    // the result into `integ`/`pi_out` on `upd_tick`. Pipeline
    // latency (~4 cycles) is completely negligible vs the 10 ms
    // update period.
    reg signed [15:0]  err_q       = 16'sd0;
    reg signed [47:0]  integ_raw_q = 48'sd0;
    reg signed [31:0]  prop_q      = 32'sd0;
    reg signed [31:0]  integ_n_q   = 32'sd0;
    reg signed [33:0]  pi_raw_q    = 34'sd0;
    reg signed [31:0]  pi_n_q      = 32'sd0;

    wire signed [31:0] integ_clamped =
        (integ_raw_q > STEP_ADJ_MAX) ? STEP_ADJ_MAX :
        (integ_raw_q < STEP_ADJ_MIN) ? STEP_ADJ_MIN :
        integ_raw_q[31:0];

    wire signed [31:0] pi_clamped =
        (pi_raw_q > STEP_ADJ_MAX) ? STEP_ADJ_MAX :
        (pi_raw_q < STEP_ADJ_MIN) ? STEP_ADJ_MIN :
        pi_raw_q[31:0];

    always @(posedge clk) begin
        if (rst || !enable) begin
            err_q       <= 16'sd0;
            integ_raw_q <= 48'sd0;
            prop_q      <= 32'sd0;
            integ_n_q   <= 32'sd0;
            pi_raw_q    <= 34'sd0;
            pi_n_q      <= 32'sd0;
        end else begin
            err_q       <= err_c;
            integ_raw_q <= integ + ($signed(KI[15:0]) * err_q);
            prop_q      <= $signed(KP[15:0]) * err_q;
            integ_n_q   <= integ_clamped;
            pi_raw_q    <= integ_n_q + prop_q;
            pi_n_q      <= pi_clamped;
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            upd_cnt <= 0;
            integ   <= 32'sd0;
            pi_out  <= 32'sd0;
        end else if (!enable) begin
            // Hold step at nominal (zero adjustment) and reset
            // integrator so re-enable starts clean.
            upd_cnt <= 0;
            integ   <= 32'sd0;
            pi_out  <= 32'sd0;
        end else begin
            upd_cnt <= upd_tick ? {DIV_W{1'b0}} : upd_cnt + 1'b1;
            if (upd_tick) begin
                integ  <= integ_n_q;
                pi_out <= pi_n_q;
            end
        end
    end

    // ── Output slew-rate limiter ────────────────────────────────
    // pi_out is the "target" the PI loop wants. pi_smooth is what we
    // actually hand to the ASRC, and it ramps toward pi_out by 1 LSB
    // per STEP_SLEW_PERIOD mclks. Any single PI jump is therefore
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
