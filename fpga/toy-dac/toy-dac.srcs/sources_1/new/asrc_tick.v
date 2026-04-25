`timescale 1ns / 1ps
`default_nettype none

// NCO-based sample-tick generator with a PI rate servo.
//
// Architecture
// ────────────
// A 32-bit phase accumulator advances by (INC_NOMINAL + pi_out) each
// mclk cycle. Every accumulator overflow is one "sample tick" — used
// to pop the FIFO and push a sample into the interpolator. Average
// tick rate:
//
//     f_tick = mclk_hz × (INC_NOMINAL + pi_out) / 2^32
//
// A slow PI controller updated at UPDATE_HZ (default 100 Hz) reads the
// FIFO fill level and adjusts pi_out so that (fifo_count − SETPOINT)
// → 0. The plant is integrating (rate error integrates into depth),
// so a pure I controller would be marginally stable; the proportional
// term provides phase margin.
//
// Sign convention
// ───────────────
// err = fifo_count − SETPOINT
//   err > 0 ⇒ depth too high  ⇒ consume faster ⇒ increase tick rate
//   err < 0 ⇒ depth too low   ⇒ consume slower ⇒ decrease tick rate
//
// INC_NOMINAL
// ───────────
// Hardcoded for 44.1 kHz at 54 MHz mclk:
//     round(2^32 × 44100 / 54_000_000) = 3_507_557
// For other rates:
//     48000 / 54 MHz → 3_817_748
//     44100 / 48 MHz → 3_945_752    (regenerate with the right MCLK)
//
// inc_adj resolution
// ──────────────────
// 1 LSB of pi_out ≈ 0.01258 Hz of sample-rate adjustment.
// INC_ADJ limit of ±2^19 (=524288) gives ±~6.6 kHz of correction,
// far more than needed for typical clock drift (< 100 ppm = 4.4 Hz).

module asrc_tick #(
    parameter integer FIFO_DEPTH        = 64,
    parameter integer SETPOINT          = FIFO_DEPTH/2,
    parameter integer MCLK_HZ           = 54_000_000,
    parameter integer UPDATE_HZ         = 100,
    parameter [31:0]  INC_NOMINAL       = 32'd3_507_557,   // 44.1 kHz @ 54 MHz
    parameter integer KP                = 32,
    parameter integer KI                = 4,
    parameter signed [31:0] INC_ADJ_MAX =  32'sd524288,
    parameter signed [31:0] INC_ADJ_MIN = -32'sd524288
)(
    input  wire                              clk,
    input  wire                              rst,
    input  wire                              enable,        // pause servo when low
    input  wire [$clog2(FIFO_DEPTH+1)-1:0]   fifo_count,
    output reg                               tick = 1'b0,

    // Debug
    output wire signed [15:0]                dbg_error,
    output wire signed [31:0]                dbg_inc_adj,
    output wire        [31:0]                dbg_inc_eff
);

    // ── Prime gate ────────────────────────────────────────────────
    // Hold the NCO and PI servo until the FIFO has filled to the
    // setpoint. Otherwise the NCO ticks before the first I2S sample
    // lands, drains it instantly, and the depth never builds up.
    //
    // Re-arm if the FIFO stays chronically near-empty (input
    // disconnected, or PI integrator wound up during a transient and
    // is now draining everything as soon as it arrives). We use
    // "fifo_count < LOW_MARK" rather than "== 0" because once the
    // NCO is running too fast, depth oscillates between 0 and 1 and
    // never sits at exactly 0 long enough to trigger.
    localparam integer EMPTY_REARM_TICKS = MCLK_HZ / 10;   // 100 ms
    localparam integer REARM_W           = $clog2(EMPTY_REARM_TICKS+1);
    localparam integer LOW_MARK          = (SETPOINT > 8) ? 8 : (SETPOINT/2);
    reg [REARM_W-1:0] empty_cnt = 0;
    reg primed = 1'b0;
    always @(posedge clk) begin
        if (rst) begin
            primed    <= 1'b0;
            empty_cnt <= 0;
        end else begin
            if (fifo_count < LOW_MARK[$clog2(FIFO_DEPTH+1)-1:0]) begin
                if (empty_cnt == EMPTY_REARM_TICKS[REARM_W-1:0])
                    primed <= 1'b0;
                else
                    empty_cnt <= empty_cnt + 1'b1;
            end else begin
                empty_cnt <= 0;
                // Re-prime as soon as the FIFO climbs above the
                // low-water mark. Using SETPOINT here would leave us
                // wedged at, e.g., count=30 with the NCO halted — the
                // servo can pull depth from 8 to 32 just fine on its
                // own once enabled.
                primed <= 1'b1;
            end
        end
    end

    // ── NCO ─────────────────────────────────────────────────────────
    reg  [31:0] acc = 32'd0;

    // PI controller state
    reg  signed [31:0] integ  = 32'sd0;   // I term (persistent)
    reg  signed [31:0] pi_out = 32'sd0;   // I + P, refreshed each update

    // Effective NCO increment
    wire signed [32:0] inc_eff_s = $signed({1'b0, INC_NOMINAL}) + pi_out;
    wire        [31:0] inc_eff   = inc_eff_s[31:0];

    wire        [32:0] acc_next  = {1'b0, acc} + {1'b0, inc_eff};

    always @(posedge clk) begin
        if (rst || !primed) begin
            acc  <= 32'd0;
            tick <= 1'b0;
        end else begin
            acc  <= acc_next[31:0];
            tick <= acc_next[32];          // carry-out = sample period elapsed
        end
    end

    // ── PI servo (runs at UPDATE_HZ) ────────────────────────────────
    localparam integer UPDATE_DIV = MCLK_HZ / UPDATE_HZ;
    localparam integer DIV_W      = $clog2(UPDATE_DIV);
    reg [DIV_W-1:0] upd_cnt = 0;
    wire upd_tick = (upd_cnt == UPDATE_DIV - 1);

    // Error in samples (sign-extended)
    wire signed [15:0] err = $signed({1'b0, fifo_count}) - $signed(SETPOINT[15:0]);

    // Integrator step: integ += KI × err, with clamp
    wire signed [47:0] integ_n_raw = integ + ($signed(KI[15:0]) * err);
    wire signed [31:0] integ_n =
        (integ_n_raw > INC_ADJ_MAX) ? INC_ADJ_MAX :
        (integ_n_raw < INC_ADJ_MIN) ? INC_ADJ_MIN :
        integ_n_raw[31:0];

    // Proportional term: KP × err (transient, not accumulated)
    wire signed [31:0] prop = $signed(KP[15:0]) * err;
    wire signed [33:0] pi_n_raw = integ_n + prop;
    wire signed [31:0] pi_n =
        (pi_n_raw > INC_ADJ_MAX) ? INC_ADJ_MAX :
        (pi_n_raw < INC_ADJ_MIN) ? INC_ADJ_MIN :
        pi_n_raw[31:0];

    always @(posedge clk) begin
        if (rst) begin
            upd_cnt <= 0;
            integ   <= 32'sd0;
            pi_out  <= 32'sd0;
        end else if (!primed) begin
            // While re-priming, reset the PI state so we start the
            // next NCO run from the nominal increment.
            upd_cnt <= 0;
            integ   <= 32'sd0;
            pi_out  <= 32'sd0;
        end else begin
            upd_cnt <= upd_tick ? {DIV_W{1'b0}} : upd_cnt + 1'b1;
            if (upd_tick && enable) begin
                integ  <= integ_n;
                pi_out <= pi_n;
            end
        end
    end

    assign dbg_error   = err;
    assign dbg_inc_adj = integ;
    assign dbg_inc_eff = inc_eff;

endmodule

`default_nettype wire
