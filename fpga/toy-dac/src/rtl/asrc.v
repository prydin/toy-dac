`timescale 1ns / 1ps
`default_nettype none

// Asynchronous Sample-Rate Conversion wrapper (fractional-phase),
// followed by a 4× halfband interpolator cascade.
//
// Architecture:
//
//   I2S in ─► fractional_asrc L ─► halfband_up2 ─► halfband_up2 ─► dac_left
//                                  (×2)            (×2)
//   I2S in ─► fractional_asrc R ─► halfband_up2 ─► halfband_up2 ─► dac_right
//
//   Engine output rate     = mclk / OUT_DIV          = 1.6875 MHz @ OUT_DIV=64
//   After 1st halfband     = mclk / (OUT_DIV/2)      = 3.375  MHz
//   After 2nd halfband     = mclk / (OUT_DIV/4)      = 6.75   MHz   ← dac_dv_*
//
// Each halfband is a symmetric 11-tap FIR (3 DSP48), with the
// passthrough "odd" branch implemented as a pure delayed sample
// (h[5] = 1.0). The cascade quadruples the rate at which the ΔΣ
// modulator's `din_held` updates, which lets its noise-shaping
// filter act on the signal at much finer time granularity than
// when fed a 1.6875 MHz step-held input.
//
//                ▲   ▲
//                │   │ step (Q0.32)
//                │   └── frac_servo (PI loop on samples_avail)
//                │
//                └── out_strobe (mclk/OUT_DIV counter)
//
// Each `fractional_asrc` instance owns its own input ring buffer
// (2*TAPS = 128 samples deep), polyphase ROM, MAC engine, and phase
// accumulator. They share a single `step` value driven by `frac_servo`
// and a common `out_strobe`, so L and R consume input samples in
// lockstep and produce output samples on the same grid.
//
// `frac_servo` measures the L engine's `samples_avail` (unconsumed
// samples in the ring buffer) and adjusts `step` around the supplied
// `step_nominal` to keep that depth at SAMP_SETPOINT. L and R have
// identical input rates and identical step, so their `samples_avail`
// trajectories match.
//
// What was removed from the old `asrc`:
//   - `nco` instance (PI servo + dual NCO ticks)
//   - `tick_x100` and the 100x output rate generator
//   - `out_primed` gate
//   - input FIFOs (the engine's internal ring buffer is the
//     elasticity buffer)
//   - external interpolator AXI loop (src_*/interp_*)
//   - output rate-leveling FIFOs

module asrc #(
    parameter integer WIDTH         = 32,
    parameter integer COEFF_W       = 18,
    parameter integer PHASES        = 256,
    parameter integer TAPS          = 64,
    parameter         COEFF_FILE    = "frac_asrc.mem",
    parameter integer MCLK_HZ       = 108_000_000,
    parameter integer OUT_DIV       = 64,                 // Fs_out = mclk / OUT_DIV
    // PI servo target: depth of the engine's internal ring buffer.
    // Safe operating range is [1, SAMP_DEPTH - TAPS] = [1, 64];
    // mid-range is the natural setpoint.
    parameter integer SAMP_SETPOINT = TAPS,        // mid-depth of 2*TAPS ring
    // PI servo update rate. 100 Hz is fine in real hardware; bumping
    // to 1000+ Hz makes simulation testbenches converge in ms.
    parameter integer SERVO_UPDATE_HZ = 100,
    // Default 44.1 kHz step constant (Q0.32) for Fs_out = 1.6875 MHz.
    // round(44100 / 1687500 * 2^32) = 112_261_131.
    parameter [31:0]  STEP_NOMINAL  = 32'd112_261_131
)(
    input  wire                    clk,
    input  wire                    rst,
    input  wire                    enable,

    // Runtime nominal step (Q0.32). 0 means "use STEP_NOMINAL parameter".
    input  wire [31:0]             step_nominal_in,

    // ── I2S input side ──
    input  wire signed [WIDTH-1:0] i2s_left,
    input  wire signed [WIDTH-1:0] i2s_right,
    input  wire                    left_valid,
    input  wire                    right_valid,

    // ── DAC output (rate-leveled on mclk/OUT_DIV grid) ──
    output wire signed [WIDTH-1:0] dac_left,
    output wire signed [WIDTH-1:0] dac_right,
    output wire                    dac_dv_left,
    output wire                    dac_dv_right,

    // ── Diagnostics ──
    output wire        [15:0]      dbg_samples_avail_l,
    output wire        [15:0]      dbg_samples_avail_r,
    output wire                    in_consumed,            // L engine carry (== R)
    output wire        [31:0]      dbg_step,
    output wire signed [15:0]      dbg_servo_error,
    output wire signed [31:0]      dbg_servo_step_adj,
    output wire                    adjust                  // 1-cycle pulse on PI update
);

    // ── Effective nominal step (runtime override) ───────────────
    wire [31:0] step_nom_eff =
        (step_nominal_in != 32'd0) ? step_nominal_in : STEP_NOMINAL;

    // ── Output strobe: 1-cycle pulse every OUT_DIV mclks ────────
    localparam integer DIV_W = $clog2(OUT_DIV);
    reg [DIV_W-1:0] out_div_cnt = {DIV_W{1'b0}};
    reg             out_strobe  = 1'b0;
    always @(posedge clk) begin
        if (rst) begin
            out_div_cnt <= {DIV_W{1'b0}};
            out_strobe  <= 1'b0;
        end else if (out_div_cnt == OUT_DIV - 1) begin
            out_div_cnt <= {DIV_W{1'b0}};
            out_strobe  <= 1'b1;
        end else begin
            out_div_cnt <= out_div_cnt + 1'b1;
            out_strobe  <= 1'b0;
        end
    end

    // ── Frac servo (PI on L engine's ring-buffer depth) ─────────
    wire [15:0] samp_avail_l;
    wire [15:0] samp_avail_r;
    wire [31:0] step_eff;

    // Servo only sees the low bits of samples_avail (it cannot exceed
    // the ring-buffer depth SAMP_DEPTH = 2*TAPS in normal operation,
    // but cap defensively so a transient overshoot can't garble the
    // servo's error math).
    localparam integer SAMP_DEPTH = 2 * TAPS;
    wire [$clog2(SAMP_DEPTH+1)-1:0] samp_avail_clamped =
        (samp_avail_l > SAMP_DEPTH[15:0]) ? SAMP_DEPTH[$clog2(SAMP_DEPTH+1)-1:0]
                                          : samp_avail_l[$clog2(SAMP_DEPTH+1)-1:0];

    frac_servo #(
        .FIFO_DEPTH  (SAMP_DEPTH),
        .SETPOINT    (SAMP_SETPOINT),
        .MCLK_HZ     (MCLK_HZ),
        .UPDATE_HZ   (SERVO_UPDATE_HZ)
    ) servo_inst (
        .clk             (clk),
        .rst             (rst),
        .enable          (enable),
        .step_nominal_in (step_nom_eff),
        .fifo_count      (samp_avail_clamped),
        .step            (step_eff),
        .dbg_error       (dbg_servo_error),
        .dbg_step_adj    (dbg_servo_step_adj),
        .adjust          (adjust)
    );

    // ── Stereo fractional ASRC engines ──────────────────────────
    // Both engines share `step` and `out_strobe`, so they consume
    // input samples in lockstep. Each owns its own coefficient
    // ROM (loaded from the same .mem file).
    wire signed [WIDTH-1:0] dout_l;
    wire signed [WIDTH-1:0] dout_r;
    wire                    dvalid_l;
    wire                    dvalid_r;
    wire                    in_consumed_l;
    wire                    in_consumed_r;

    fractional_asrc #(
        .DATA_W    (WIDTH),
        .COEFF_W   (COEFF_W),
        .PHASES    (PHASES),
        .TAPS      (TAPS),
        .COEFF_FILE(COEFF_FILE)
    ) eng_l (
        .clk              (clk),
        .rst              (rst),
        .enable           (enable),
        .sample_in        (i2s_left),
        .sample_valid     (left_valid),
        .out_strobe       (out_strobe),
        .step             (step_eff),
        .data_out         (dout_l),
        .dvalid_out       (dvalid_l),
        .in_consumed      (in_consumed_l),
        .dbg_phase_acc    (),
        .dbg_mac_cyc      (),
        .dbg_samples_avail(samp_avail_l)
    );

    fractional_asrc #(
        .DATA_W    (WIDTH),
        .COEFF_W   (COEFF_W),
        .PHASES    (PHASES),
        .TAPS      (TAPS),
        .COEFF_FILE(COEFF_FILE)
    ) eng_r (
        .clk              (clk),
        .rst              (rst),
        .enable           (enable),
        .sample_in        (i2s_right),
        .sample_valid     (right_valid),
        .out_strobe       (out_strobe),
        .step             (step_eff),
        .data_out         (dout_r),
        .dvalid_out       (dvalid_r),
        .in_consumed      (in_consumed_r),
        .dbg_phase_acc    (),
        .dbg_mac_cyc      (),
        .dbg_samples_avail(samp_avail_r)
    );

    // ── 4× halfband interpolator cascade per channel ─────────
    // Two stages: 1.6875 MHz → 3.375 MHz → 6.75 MHz (with OUT_DIV=64).
    // Each stage timed by its IN_PERIOD_MCLK so its two outputs per
    // input sit evenly spaced over the input period.
    wire signed [WIDTH-1:0] h1_l_data, h1_r_data;
    wire                    h1_l_dv,   h1_r_dv;
    wire signed [WIDTH-1:0] h2_l_data, h2_r_data;
    wire                    h2_l_dv,   h2_r_dv;

    halfband_up2 #(
        .DATA_W         (WIDTH),
        .IN_PERIOD_MCLK (OUT_DIV)
    ) hb1_l (
        .clk      (clk),
        .rst      (rst),
        .enable   (enable),
        .in_valid (dvalid_l),
        .in_data  (dout_l),
        .out_valid(h1_l_dv),
        .out_data (h1_l_data)
    );

    halfband_up2 #(
        .DATA_W         (WIDTH),
        .IN_PERIOD_MCLK (OUT_DIV)
    ) hb1_r (
        .clk      (clk),
        .rst      (rst),
        .enable   (enable),
        .in_valid (dvalid_r),
        .in_data  (dout_r),
        .out_valid(h1_r_dv),
        .out_data (h1_r_data)
    );

    halfband_up2 #(
        .DATA_W         (WIDTH),
        .IN_PERIOD_MCLK (OUT_DIV / 2)
    ) hb2_l (
        .clk      (clk),
        .rst      (rst),
        .enable   (enable),
        .in_valid (h1_l_dv),
        .in_data  (h1_l_data),
        .out_valid(h2_l_dv),
        .out_data (h2_l_data)
    );

    halfband_up2 #(
        .DATA_W         (WIDTH),
        .IN_PERIOD_MCLK (OUT_DIV / 2)
    ) hb2_r (
        .clk      (clk),
        .rst      (rst),
        .enable   (enable),
        .in_valid (h1_r_dv),
        .in_data  (h1_r_data),
        .out_valid(h2_r_dv),
        .out_data (h2_r_data)
    );

    // ── Outputs ──────────────────────────────────────
    assign dac_left            = h2_l_data;
    assign dac_right           = h2_r_data;
    assign dac_dv_left         = h2_l_dv;
    assign dac_dv_right        = h2_r_dv;
    assign dbg_samples_avail_l = samp_avail_l;
    assign dbg_samples_avail_r = samp_avail_r;
    assign in_consumed         = in_consumed_l;
    assign dbg_step            = step_eff;

endmodule

`default_nettype wire
