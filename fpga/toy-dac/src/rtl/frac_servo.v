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
//   The servo updates at UPDATE_HZ (default 100 Hz). Defaults are
//   KP = 2048, KI = 64.  Sensitivity:
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
    parameter integer UPDATE_HZ          = 100,
    parameter integer KP                 = 2048,
    parameter integer KI                 = 64,
    parameter integer ERR_DEADBAND       = 1,
    parameter signed [31:0] STEP_ADJ_MAX =  32'sd16777216,   //  +2^24
    parameter signed [31:0] STEP_ADJ_MIN = -32'sd16777216    //  -2^24
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
            adjust  <= 1'b0;
        end else if (!enable) begin
            // Hold step at nominal (zero adjustment) and reset
            // integrator so re-enable starts clean.
            upd_cnt <= 0;
            integ   <= 32'sd0;
            pi_out  <= 32'sd0;
            adjust  <= 1'b0;
        end else begin
            upd_cnt <= upd_tick ? {DIV_W{1'b0}} : upd_cnt + 1'b1;
            adjust  <= 1'b0;
            if (upd_tick) begin
                integ  <= integ_n_q;
                pi_out <= pi_n_q;
                adjust <= (pi_n_q != pi_out);
            end
        end
    end

    // ── Compose effective step: step_nominal + pi_out ───────────
    // Add as 33-bit signed then truncate; step_nominal_in is unsigned
    // Q0.32 and pi_out is signed adjustment in the same units.
    wire signed [32:0] step_sum = $signed({1'b0, step_nominal_in}) + pi_out;
    assign step = step_sum[31:0];

    assign dbg_error    = err_q;
    assign dbg_step_adj = pi_out;

endmodule

`default_nettype wire
