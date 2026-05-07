module top(
    input wire clk,        // 100MHz clock from crystal
    output wire dac_out_l,     // DAC delta/sigma out (+)
    output wire dac_out_l_fast,     // DAC delta/sigma out (+)
    output wire dac_out_r,     // Main clock output for debugging
    output wire dac_out_ln,    // DAC delta/sigma out (−), complement of dac_out_l
    output wire dac_out_ln_fast,    // DAC delta/sigma out (−), complement of dac_out_l_fast
    output wire dac_out_rn,    // Complement of dac_out_r
    output wire debug1,        // Debug output 
    output wire debug2,        // Debug output 
    output wire debug3,        // Debug output
    output wire debug4,        // Debug output
    input wire bclk,       // I2S bit clock
    input wire lrclk,       // I2S left-right clock
    input wire din,        // I2S serial data in
    input wire bypass,     // A/B test: HIGH = bypass ASRC, clock samples directly from I2S
    input wire [1:0] btn,     // Push buttons
    output wire [3:0] led,    // LEDs for mode and dither indication
    output wire [4:0] fifo_led, // 5-step thermometer of ASRC ring-buffer fill (pio16-20)
    output wire led0_b        // RGB LED blue channel — blinks on each ASRC adjust
);


parameter I2S_WORDLENGTH = 32;
parameter MODULATOR_WORDLENGTH = 32;
parameter MODULATOR_ORDER = 2;  // 1/2/3 supported by dac.v.
                                // Sim shows ORDER=3 with Pascal coeffs (NTF=(1-z^-1)^3,
                                // OBG=8) is unstable for ANY input — integrators saturate
                                // and the output is broadband noise. ORDER=2 (OBG=4) is
                                // stable and gives ~70 dB SNR in audio band at 108 MHz/64.

// ── Master clock & rate-manager defaults ──────────────────────
// MCLK_HZ is the single source of truth. The audio sample rate is
// detected at runtime by `rate_detect` and committed to the ASRC's
// step generator by `rate_manager`. The constants below are only
// used as fallbacks (test modes, DDS divider, etc.).
//
// MCLK bumped from 54 MHz → 108 MHz for the fractional-phase ASRC
// rewrite (Phase 4): with OUT_DIV = 64 the per-channel MAC engine
// gets 64 mclk cycles per output strobe (1.6875 MHz output rate).
// IMPORTANT: the `clock` IP inside src/ip/clock/ must be regenerated
// in Vivado to produce 108 MHz at its `mclk` output before this
// design will function on hardware. Simulation testbenches drive
// their own clocks and are not affected.
localparam integer MCLK_HZ          = 108_000_000;
localparam integer ASRC_OUT_DIV     = 64;
localparam integer FS_OUT_HZ        = MCLK_HZ / ASRC_OUT_DIV;   // 1_687_500
localparam integer FS_DEFAULT_HZ    = 44_100;
localparam [31:0]  INC_NOMINAL_44_1 =
    (((64'd1 << 32) * FS_DEFAULT_HZ) + (MCLK_HZ / 2)) / MCLK_HZ;
localparam [31:0]  STEP_NOMINAL_44_1 =
    (((64'd1 << 32) * FS_DEFAULT_HZ) + (FS_OUT_HZ / 2)) / FS_OUT_HZ;
localparam integer SAMPLE_DIV       = MCLK_HZ / FS_DEFAULT_HZ;

// ── Mode selector: btn[0] cycles through four signal sources ──
// Mode 0 = I2S, Mode 1 = DDS 1 kHz test tone, Mode 2 = DC (1/1000 of max), Mode 3 = Off (0V)
localparam MODE_I2S  = 2'd0;
localparam MODE_DDS  = 2'd1;
localparam MODE_DC   = 2'd2;
localparam MODE_OFF  = 2'd3;
localparam NUM_MODES = 4;

// I2S signals
wire signed [I2S_WORDLENGTH-1:0] i2s_left;
wire signed [I2S_WORDLENGTH-1:0] i2s_right;
wire input_active;
wire left_valid;
wire right_valid;
// Legacy AXI-Stream backpressure from the (removed) FIR Compiler IP.
// The new fractional `asrc` has its own input ring buffer and never
// stalls the producer, and the DDS sample-rate pacer is also always
// ready, so we tie these high.
wire output_ready_left  = 1'b1;
wire output_ready_right = 1'b1;

// ── Clock ────────────────────────────────────────────────────────
// Single MMCM driven from the 12 MHz crystal produces the 54 MHz
// mclk. Everything (DAC pipeline, FIR, I2S receiver, FIFO, ASRC NCO)
// runs on mclk — rate matching is purely digital via the asrc module.
wire mclk;
wire mclk_locked;

// Hold everything in reset until the PLL locks and the clock is
// stable. Without this, the DAC/FIR run on a glitching clock during
// PLL startup and accumulate corrupt state they never recover from.
wire rst = ~mclk_locked;

// Forward declarations for rate_manager outputs — referenced by LED
// and debug assigns earlier in the file than the rate_manager
// instantiation itself.
wire [31:0] rm_inc_nominal;
wire [31:0] rm_step_nominal;
wire        rm_asrc_rst;
wire        rm_mute;
wire        rm_rate_locked;
wire        rm_unsupported;
wire [1:0]  rm_rate_code;
// Forward decl: 1-cycle pulse on each accepted I2S left sample.
// Defined later (= left_valid & output_ready_left); referenced by
// the debug-pin block above the i2s instantiation.
wire        i2s_left_accept;
// Forward decl: 1-mclk pulse on lrclk rising edge from the I2S receiver's
// internal synchronizer; referenced by the frame-integrity probe below.
wire        lrclk_pos_edge_w;

// Forward declarations for asrc instance outputs / legacy diagnostic
// aliases — referenced by recovered_clk / debug assigns below before
// the asrc instance proper.
wire        asrc_in_consumed;
wire signed [31:0] asrc_dbg_step_adj;
wire [15:0] asrc_samp_avail_l;
wire        asrc_tick_l = asrc_in_consumed;
wire signed [31:0] asrc_dbg_inc_adj = asrc_dbg_step_adj;   // legacy alias
wire fifo_empty_l = (asrc_samp_avail_l == 16'd0);

// Phase-shift port left tied off — the digital ASRC does the rate
// matching, so mclk is held at its nominal frequency.
wire ps_done_unused;

// Explicit input buffer. The MMCM IP is configured with "No buffer"
// on its input, so we instantiate the IBUF here.
wire clk_ibuf;
IBUF clk_ibuf_inst (.I(clk), .O(clk_ibuf));

// ── Button debounce and mode cycling ──
wire btn0_db;
debounce btn0_debounce (
    .clk(mclk),
    .rst(rst),
    .btn_in(btn[0]),
    .btn_out(btn0_db)
);

reg btn0_prev = 0;
reg [1:0] mode = MODE_I2S;
always @(posedge mclk) begin
    if (rst) begin
        btn0_prev <= 0;
        mode      <= MODE_I2S;
    end else begin
        btn0_prev <= btn0_db;
        // Rising edge of debounced button
        if (btn0_db && !btn0_prev) begin
            if (mode == NUM_MODES - 1)
                mode <= 0;
            else
                mode <= mode + 1;
        end
    end
end

// ── btn[1]: toggle dither on/off ──
wire btn1_db;
debounce btn1_debounce (
    .clk(mclk),
    .rst(rst),
    .btn_in(btn[1]),
    .btn_out(btn1_db)
);

reg btn1_prev = 0;
reg dither_en = 1;
always @(posedge mclk) begin
    if (rst) begin
        btn1_prev <= 0;
        dither_en <= 1;
    end else begin
        btn1_prev <= btn1_db;
        if (btn1_db && !btn1_prev)
            dither_en <= ~dither_en;
    end
end

// LEDs: [1:0] = mode, [2] = mclk_locked, [3] = dither_en
//   (audio-rate-lock indication is still available on fifo_led, which
//    switches to a rate-code display once the rate detector locks.)
assign led = {dither_en, mclk_locked, mode};

// Debug pins — GLITCH HUNT v2 (April 28, frame-integrity probes).
//   debug1 = mclk_locked            — should never drop.
//   debug2 = i2s_sample_toggle      — toggles on each accepted I2S left
//                                     sample. ~22 kHz square wave when
//                                     audio is flowing; freezing = source
//                                     stopped.
//   debug3 = frame_size_anomaly     — stretched 5 ms HIGH whenever the
//                                     bclk-edge count between two
//                                     successive lrclk rising edges is
//                                     not equal to BCLK_PER_FRAME (set
//                                     to 64 for 32-bit-per-channel I2S
//                                     with 64-bit frames; the receiver
//                                     auto-learns the size on first frame).
//                                     Pulses HIGH ⇒ I2S framing is
//                                     intermittently broken (bclk or
//                                     lrclk glitching).
//   debug4 = i2s_zero_word          — pulses on each accepted sample whose
//                                     upper 24 bits are all-zero or all-
//                                     ones (true zero or near-zero).
//assign debug1 = mclk_locked;

reg debug2_r = 1'b0;
always @(posedge mclk) begin
    if (rst)                  debug2_r <= 1'b0;
    else if (i2s_left_accept) debug2_r <= ~debug2_r;
end
// assign debug2 = debug2_r;

// ── Frame-integrity probe ────────────────────────────────────────
// Use the same edge pulses i2s.v consumes. We want bclk_pos_edge and
// lrclk_pos_edge as 1-cycle mclk-synchronous pulses; flop_sync inside
// i2s.v exposes lrclk_pos_edge already (lrclk_pos_edge_w). For bclk
// edge counts we re-synchronize bclk locally so we don't have to
// thread an extra port through i2s.v.
reg bclk_s1 = 1'b0, bclk_s2 = 1'b0, bclk_s3 = 1'b0;
always @(posedge mclk) begin
    bclk_s1 <= bclk;
    bclk_s2 <= bclk_s1;
    bclk_s3 <= bclk_s2;
end
wire bclk_rise_local = bclk_s2 & ~bclk_s3;

reg [7:0] bclk_in_frame  = 8'd0;
reg [7:0] last_frame_len = 8'd0;
reg [7:0] frame_len_ref  = 8'd0;     // latched on first observed frame
reg       frame_ref_set  = 1'b0;
reg       anomaly_pulse  = 1'b0;
always @(posedge mclk) begin
    if (rst) begin
        bclk_in_frame  <= 8'd0;
        last_frame_len <= 8'd0;
        frame_len_ref  <= 8'd0;
        frame_ref_set  <= 1'b0;
        anomaly_pulse  <= 1'b0;
    end else begin
        anomaly_pulse <= 1'b0;
        if (lrclk_pos_edge_w) begin
            last_frame_len <= bclk_in_frame;
            bclk_in_frame  <= 8'd0;
            if (!frame_ref_set && bclk_in_frame != 8'd0) begin
                frame_len_ref <= bclk_in_frame;
                frame_ref_set <= 1'b1;
            end else if (frame_ref_set && bclk_in_frame != frame_len_ref) begin
                anomaly_pulse <= 1'b1;
            end
        end else if (bclk_rise_local) begin
            bclk_in_frame <= bclk_in_frame + 1'b1;
        end
    end
end

// 5 ms stretch on anomaly_pulse so the scope can see it.
reg [19:0] anom_stretch = 20'd0;
localparam [19:0] ANOM_PULSE = 20'd540_000;
always @(posedge mclk) begin
    if (rst) anom_stretch <= 20'd0;
    else if (anomaly_pulse) anom_stretch <= ANOM_PULSE;
    else if (anom_stretch != 20'd0) anom_stretch <= anom_stretch - 1'b1;
end
assign debug3 = (anom_stretch != 20'd0);

assign debug4 = i2s_left_accept &&
                (i2s_left[I2S_WORDLENGTH-1 -: 24] == 24'd0 ||
                 i2s_left[I2S_WORDLENGTH-1 -: 24] == 24'hFFFFFF);

wire signed [MODULATOR_WORDLENGTH-1:0] dac_held_left;
wire signed [MODULATOR_WORDLENGTH-1:0] dac_held_right;

// ── Sink-domain clock (phase-shifted by soft_pll) ────────────────
// 54 MHz mclk with dynamic phase shift enabled. psclk is tied to mclk
// itself, which is supported by the 7-series MMCM (psen/psincdec are
// sampled on psclk; soft_pll lives on mclk so this keeps everything in
// one domain).
clock main_clock (
    .mclk(mclk),                // output mclk (phase-shifted)
    .reset(1'b0),               // PLL starts freely; rst is derived from locked
    .locked(mclk_locked),       // output locked
    .clk_in1(clk_ibuf),         // input ref clock (shared IBUFG output)
    .psclk(mclk),               // PS port clocked by mclk
    .psen(1'b0),                // ASRC handles rate matching, no PS pulses
    .psincdec(1'b0),
    .psdone(ps_done_unused)
);

// ── I2S input ────────────────────────────────────────────────────
// Runs on mclk. Its AXI-stream output goes two places:
//   1. Into the legacy `interpolator100x` (Xilinx FIR Compiler IP)
//      via the bypass-path `src_*` mux below. That IP uses proper
//      AXI-Stream backpressure, so we MUST drive `i2s.*_ready` from
//      the interpolator's `tready`. Tying ready high breaks the
//      handshake and causes silent sample drops whenever the FIR
//      core's input FIFO momentarily de-asserts ready (audible as
//      random clicks in bypass mode).
//   2. Into the new fractional ASRC (`asrc_inst`). That module wants
//      a 1-cycle `sample_valid` pulse per new sample, NOT held tvalid.
//      We derive that below from the AXI accept event
//      (left_valid & output_ready_left), which is exactly 1 cycle wide.

i2s #(
    .WORDLENGTH(I2S_WORDLENGTH)
) i2s_inst (
    .clk(mclk),
    .rst(rst),
    .bclk(bclk),
    .lrclk(lrclk),
    .din(din),
    .out_left(i2s_left),
    .out_right(i2s_right),
    .input_active(input_active),
    .left_valid(left_valid),
    .right_valid(right_valid),
    // Backpressure follows the legacy interpolator's AXI tready so
    // its input FIFO never silently drops samples.
    .left_ready(output_ready_left),
    .right_ready(output_ready_right),
    .lrclk_pos_edge(lrclk_pos_edge_w)
);

// 1-cycle AXI "sample accepted" pulses for the new fractional ASRC.
assign i2s_left_accept  = left_valid  & output_ready_left;
wire   i2s_right_accept = right_valid & output_ready_right;

// ── Rate detection & management ───────────────────────────
// rate_detect averages mclk cycles over WINDOW_SIZE lrclk periods
// (~5.8 ms @ 44.1 kHz) and emits `rate_valid` + `period`. rate_manager
// classifies into 32k/44.1k/48k, debounces, and drives the ASRC's
// inc_nominal + reset + audio mute lines.
wire        rd_valid;
wire [31:0] rd_window_period;
wire [15:0] rd_period;

rate_detect #(
    .WINDOW_SIZE(256)
) rate_det (
    .clk           (mclk),
    .rst           (rst),
    .lrclk_pos_edge(lrclk_pos_edge_w),
    .dvalid        (1'b1),
    .rate_valid    (rd_valid),
    .window_period (rd_window_period),
    .period        (rd_period)
);

rate_manager #(
    .MCLK_HZ         (MCLK_HZ),
    .FS_OUT_HZ       (FS_OUT_HZ),
    .MATCHES_REQUIRED(3),
    .SETTLE_MS       (250)
) rate_mgr (
    .clk         (mclk),
    .rst         (rst),
    .rate_valid  (rd_valid),
    .period      (rd_period),
    .inc_nominal (rm_inc_nominal),
    .step_nominal(rm_step_nominal),
    .asrc_rst    (rm_asrc_rst),
    .mute        (rm_mute),
    .rate_locked (rm_rate_locked),
    .unsupported (rm_unsupported),
    .rate_code   (rm_rate_code)
);

// In test modes (DDS, DC) we don't care about the input sample rate —
// the upstream DDS divider runs at the design's nominal 44.1 kHz, so
// the ASRC's inc_nominal stays at INC_NOMINAL_44_1 and rate-manager's
// reset/mute outputs are ignored.
wire        i2s_mode      = (mode == MODE_I2S);

// ── A/B bypass switch ────────────────────────────────────────────
// `bypass` is tied to 3.3V (HIGH) to enable A/B test mode: the ASRC
// FIFOs and NCO are bypassed entirely, and I2S samples drive the
// interpolator directly on their natural lrclk-paced valid strobes.
// The interpolator output then drives the DAC directly (no output
// FIFO rate-leveling). This gives a clean reference signal to compare
// against the regenerated/rate-matched path.
//
// Synchronize the static-ish input through 2 FFs to mclk to avoid
// metastability if it's toggled live.
reg [1:0] bypass_sync = 2'b00;
always @(posedge mclk) bypass_sync <= {bypass_sync[0], bypass};
wire bypass_s = bypass_sync[1];
wire bypass_active = i2s_mode & bypass_s;

wire        asrc_rst_in   = rst | (i2s_mode & rm_asrc_rst);
wire [31:0] asrc_step_nom = i2s_mode ? rm_step_nominal : STEP_NOMINAL_44_1;
// Bypass mode forces audio through immediately, so we override the
// rate-manager's mute (which would otherwise hold us silent for the
// 250 ms settle window every time rate_manager re-locks).
wire        audio_mute    = (i2s_mode & ~bypass_s) ? rm_mute : 1'b0;

// ── Digital ASRC (fractional-phase, Phase 4 rewrite) ────────────
// The new `asrc` owns its own input ring buffers, polyphase filter
// banks, MAC engines and a PI servo on internal ring-buffer depth.
// It emits stereo on a fixed mclk/ASRC_OUT_DIV grid (1.6875 MHz @
// 108 MHz mclk), bypassing the old NCO / AXI-loop / output-FIFO
// chain entirely. The legacy `interpolator100x` path is kept alive
// only for the bypass switch (A/B reference during bring-up).
localparam integer FIFO_DEPTH = 64;       // legacy diagnostic only
localparam integer FIFO_CW    = $clog2(FIFO_DEPTH+1);

wire signed [I2S_WORDLENGTH-1:0] dac_in_left_paced;
wire signed [I2S_WORDLENGTH-1:0] dac_in_right_paced;
wire dac_dv_left;
wire dac_dv_right;
// (asrc_in_consumed, asrc_dbg_step_adj, asrc_samp_avail_l declared
// earlier as forward-decls for legacy-alias wires.)
wire asrc_adjust;
wire signed [15:0] asrc_dbg_error;
wire [31:0]        asrc_dbg_step;
wire [15:0]        asrc_samp_avail_r;
// Remaining legacy diagnostic alias (depends on FIFO_CW/FIFO_DEPTH).
wire [FIFO_CW-1:0] fifo_rd_count_l =
    (asrc_samp_avail_l > FIFO_DEPTH[15:0]) ? FIFO_DEPTH[FIFO_CW-1:0]
                                           : asrc_samp_avail_l[FIFO_CW-1:0];

// Run the servo whenever we're in I2S mode.
wire asrc_enable = (mode == MODE_I2S);

asrc #(
    .WIDTH          (I2S_WORDLENGTH),
    .COEFF_W        (18),
    .PHASES         (256),
    .TAPS           (64),
    .COEFF_FILE     ("frac_asrc.mem"),
    .MCLK_HZ        (MCLK_HZ),
    .OUT_DIV        (ASRC_OUT_DIV),
    .SAMP_SETPOINT  (128),                         // mid-safe-range. Buffer is 4*TAPS=256 deep but the FIR
                                                   // can only see samples [C-63, C], so safe range is
                                                   // samp_avail in [1, 193]. Setpoint=128 leaves ~64 samples
                                                   // of slack on either side for crystal drift before
                                                   // hitting a rail.
    .SERVO_UPDATE_HZ(100),                         // pure-P loop, slew-limited; see frac_servo.v header
    .STEP_NOMINAL   (STEP_NOMINAL_44_1)
) asrc_inst (
    .clk                 (mclk),
    .rst                 (asrc_rst_in),
    .enable              (asrc_enable),
    .step_nominal_in     (asrc_step_nom),

    .i2s_left            (i2s_left),
    .i2s_right           (i2s_right),
    .left_valid          (i2s_left_accept),
    .right_valid         (i2s_right_accept),

    .dac_left            (dac_in_left_paced),
    .dac_right           (dac_in_right_paced),
    .dac_dv_left         (dac_dv_left),
    .dac_dv_right        (dac_dv_right),

    .dbg_samples_avail_l (asrc_samp_avail_l),
    .dbg_samples_avail_r (asrc_samp_avail_r),
    .in_consumed         (asrc_in_consumed),
    .dbg_step            (asrc_dbg_step),
    .dbg_servo_error     (asrc_dbg_error),
    .dbg_servo_step_adj  (asrc_dbg_step_adj),
    .adjust              (asrc_adjust)
);

// Blue LED: stretched pulse on every ASRC servo adjust. Each adjust
// is a 1-cycle pulse at the 100 Hz servo update rate; the mono_ff
// stretches it to ~30 ms so it's visible to the eye. With the servo
// locked and the input rate stable, blinks should be sparse; faster
// blinking means the servo is actively chasing input drift.
wire led0_b_pulse;
mono_ff #(
    .FCLK     (MCLK_HZ),
    .DELAY_MS (50),
    .RESETTABLE(0)
) blue_led_stretch (
    .clk (mclk),
    .rst (rst),
    .d   (asrc_adjust),
    .q   (led0_b_pulse)
);
assign led0_b = ~led0_b_pulse;
assign debug1 = led0_b_pulse;
assign debug2 = asrc_adjust;

// (debug2 reassigned above for glitch hunt; the older asrc_rst probe is
// removed.)



// ── Internal DDS test tone (1 kHz sine) ──
// DDS output is 26-bit Two's Complement in a 32-bit tdata field.
// Xilinx zero-pads [31:26].  Left-shift by 6 so the 26-bit sine
// fills the full 32-bit range the DAC expects.
wire [31:0] dds_raw;
wire        dds_valid;

dds test_signal (
    .aclk(mclk),
    .m_axis_data_tvalid(dds_valid),
    .m_axis_data_tdata(dds_raw)
);

// Sign-extend and left-shift by 5 (not 6) to leave ~6 dB headroom.
// Peak ≈ ±2^30, well below the FIR's ±2^31 output clamp.
wire signed [31:0] dds_data = {{1{dds_raw[25]}}, dds_raw[25:0], 5'b0};

// Sample-rate divider: ~FS_HZ tick from mclk.
// SAMPLE_DIV is derived from MCLK_HZ/FS_HZ at the top of this module.
reg [$clog2(SAMPLE_DIV)-1:0] sample_cnt = 0;
reg        test_valid = 0;
reg signed [31:0] test_held = 0;

wire test_accepted = test_valid & output_ready_left;

always @(posedge mclk) begin
    if (rst) begin
        sample_cnt <= 0;
        test_valid <= 0;
        test_held  <= 0;
    end else begin
        if (test_accepted)
            test_valid <= 0;
        if (sample_cnt == SAMPLE_DIV - 1) begin
            sample_cnt <= 0;
            test_valid <= 1;
            test_held  <= dds_data;
        end else begin
            sample_cnt <= sample_cnt + 1;
        end
    end
end

// ── DC test signal: 1/1000 of positive full-scale ──
localparam signed [I2S_WORDLENGTH-1:0] DC_LEVEL = 32'sh7FFF_FFFF / 1000;

// ── Source mux: select based on runtime mode ──
reg signed [I2S_WORDLENGTH-1:0] src_left;
reg signed [I2S_WORDLENGTH-1:0] src_right;
reg src_valid_left;
reg src_valid_right;

always @(*) begin
    case (mode)
        MODE_DDS: begin
            src_left       = test_held;
            src_right      = test_held;
            src_valid_left  = test_valid;
            src_valid_right = test_valid;
        end
        MODE_DC: begin
            src_left       = DC_LEVEL;
            src_right      = DC_LEVEL;
            src_valid_left  = test_valid;
            src_valid_right = test_valid;
        end
        default: begin // MODE_I2S — the new fractional asrc owns the
                       // DAC, so the legacy interpolator path stays
                       // idle. In `bypass_active` the I2S receiver
                       // drives the interpolator directly (A/B
                       // reference path).
            if (bypass_active) begin
                src_left        = i2s_left;
                src_right       = i2s_right;
                src_valid_left  = left_valid;
                src_valid_right = right_valid;
            end else begin
                src_left        = {I2S_WORDLENGTH{1'b0}};
                src_right       = {I2S_WORDLENGTH{1'b0}};
                src_valid_left  = 1'b0;
                src_valid_right = 1'b0;
            end
        end
    endcase
end

// (Tick-driven FIFO pop and AXI-stream presentation now live inside
// the asrc module.)

// ── Diagnostic: 5-step peak-hold thermometer of ASRC ring fill ──
// Always on (regardless of lock state) so we can directly observe
// FIFO overflow / underflow on rate changes or servo instability.
// Track the high-water mark of asrc_samp_avail_l over a refresh
// window, then drive 5 LEDs as a thermometer of the peak fill.
// Thresholds centred on SAMP_SETPOINT=128 in a 4*TAPS=256 ring;
// safe range is [1, 193]:
//
//   bit0 >= 32    not nearly empty
//   bit1 >= 96    below setpoint
//   bit2 >= 128   at/above setpoint
//   bit3 >= 160   above setpoint
//   bit4 >= 192   OVERFLOW (FIR safe upper is 193)
//
// Refresh ~1 s so a single excursion stays visible to the eye.
// Bit 4 — if it ever lights, the FIFO has been driven to the rail.
localparam integer LED_WIN_BITS = 27;   // 2^27 / 108e6 ≈ 1.24 s
reg [15:0]  samp_avail_max     = 16'd0;
reg [15:0]  samp_avail_max_lat = 16'd0;
reg [LED_WIN_BITS-1:0] led_refresh = 0;
always @(posedge mclk) begin
    if (rst) begin
        samp_avail_max     <= 16'd0;
        samp_avail_max_lat <= 16'd0;
        led_refresh        <= 0;
    end else begin
        if (asrc_samp_avail_l > samp_avail_max)
            samp_avail_max <= asrc_samp_avail_l;
        led_refresh <= led_refresh + 1'b1;
        if (led_refresh == {LED_WIN_BITS{1'b0}}) begin
            samp_avail_max_lat <= samp_avail_max;
            samp_avail_max     <= asrc_samp_avail_l;
        end
    end
end

reg [4:0] fifo_led_r = 5'b00000;
always @(posedge mclk) begin
    if (rst) begin
        fifo_led_r <= 5'b10101;   // visible startup pattern
    end else begin
        fifo_led_r[0] <= (samp_avail_max_lat >= 16'd32);
        fifo_led_r[1] <= (samp_avail_max_lat >= 16'd96);
        fifo_led_r[2] <= (samp_avail_max_lat >= 16'd128);
        fifo_led_r[3] <= (samp_avail_max_lat >= 16'd160);
        fifo_led_r[4] <= (samp_avail_max_lat >= 16'd192);
    end
end
assign fifo_led = fifo_led_r;


// (Legacy interpolator100x / FIR Compiler IP removed. The fractional
//  `asrc` is now the sole interpolation path for MODE_I2S; the
//  bypass switch routes raw I2S samples straight to the DAC instead
//  of through the old 100x upsampler.)

wire [31:0] dither1_raw;
wire [31:0] dither2_raw;

random #(
    .SEED1(64'hcafebabe01234567),
    .SEED2(64'hdeadbeef89abcdef)
) rng1 (
    .clk(mclk),
    .rst(rst),
    .dout(dither1_raw)
);

random #(
    .SEED1(64'h0f1e2d3c4b5a6978),
    .SEED2(64'h87654321fedcba98)
) rng2 (
    .clk(mclk),
    .rst(rst),
    .dout(dither2_raw)
);

// Gate dither with toggle
wire [31:0] dither1 = dither_en ? dither1_raw : 32'd0;
wire [31:0] dither2 = dither_en ? dither2_raw : 32'd0;


// The DAC itself
wire dac_raw_l;
wire dac_raw_r;

// DAC input/dvalid mux:
//   MODE_I2S, bypass=0 : ASRC output
//   MODE_I2S, bypass=1 : raw I2S samples (no resampling)
//   MODE_DDS / MODE_DC : the source mux directly (the ASRC is
//                        disabled in these modes, so its dvalid
//                        never asserts and din_held would be stuck
//                        at zero — must use src_valid_left/right
//                        instead).
wire i2s_path = (mode == MODE_I2S);
wire signed [I2S_WORDLENGTH-1:0] dac_din_left  =
        (i2s_path && bypass_active) ? i2s_left            :
        (i2s_path)                  ? dac_in_left_paced   :
                                      src_left;
wire signed [I2S_WORDLENGTH-1:0] dac_din_right =
        (i2s_path && bypass_active) ? i2s_right           :
        (i2s_path)                  ? dac_in_right_paced  :
                                      src_right;
wire dac_dvalid_l =
        (i2s_path && bypass_active) ? left_valid          :
        (i2s_path)                  ? dac_dv_left         :
                                      src_valid_left;
wire dac_dvalid_r =
        (i2s_path && bypass_active) ? right_valid         :
        (i2s_path)                  ? dac_dv_right        :
                                      src_valid_right;

dac #(
    .WORDLENGTH(MODULATOR_WORDLENGTH),
    .ORDER(MODULATOR_ORDER)
) dac_left (
    .clk(mclk),
    .rst(rst),
    .din(dac_din_left), 
    .dvalid(dac_dvalid_l),
    .dither1(dither1),
    .dither2(dither2),
    .dout(dac_raw_l)
//    .din_held_debug(dac_held_left)
);


dac #(
    .WORDLENGTH(MODULATOR_WORDLENGTH),
    .ORDER(MODULATOR_ORDER)
) dac_right (
    .clk(mclk),
    .rst(rst),
    .din(dac_din_right), 
    .dvalid(dac_dvalid_r),
    .dither1(dither1),
    .dither2(dither2),
    .dout(dac_raw_r)
//    .din_held_debug(dac_held_right)
);

// Output registers for DAC — these get packed into IOB flip-flops
// via the IOB=TRUE constraint, ensuring + and − switch simultaneously.
//
// Each pad gets its own dedicated IOB flop, all clocked off the same
// mclk edge. The inversion happens BEFORE the flop on the (−) legs so
// the +/− pair has matched clock-to-out delay (sub-ps skew). The
// `_fast` pads on PMOD JA1/JA2 are physically separate pins from the
// A4/A3 pair, so they need their own flops too — sharing one register
// across two pads forces fabric routing on one of them and reintroduces
// +/− skew that shows up as 2nd-harmonic distortion.
(* IOB = "TRUE" *) reg dac_out_l_r       = 0;
(* IOB = "TRUE" *) reg dac_out_ln_r      = 0;
(* IOB = "TRUE" *) reg dac_out_r_r       = 0;
(* IOB = "TRUE" *) reg dac_out_rn_r      = 0;
(* IOB = "TRUE" *) reg dac_out_l_fast_r  = 0;
(* IOB = "TRUE" *) reg dac_out_ln_fast_r = 0;

always @(posedge mclk) begin
    if (rst || mode == MODE_OFF || audio_mute) begin
        dac_out_l_r       <= 1'b0;
        dac_out_ln_r      <= 1'b0;
        dac_out_r_r       <= 1'b0;
        dac_out_rn_r      <= 1'b0;
        dac_out_l_fast_r  <= 1'b0;
        dac_out_ln_fast_r <= 1'b0;
    end else begin
        dac_out_l_r       <=  dac_raw_l;
        dac_out_ln_r      <= ~dac_raw_l;
        dac_out_r_r       <=  dac_raw_r;
        dac_out_rn_r      <= ~dac_raw_r;
        dac_out_l_fast_r  <=  dac_raw_l;
        dac_out_ln_fast_r <= ~dac_raw_l;
    end
end

assign dac_out_l       = dac_out_l_r;
assign dac_out_ln      = dac_out_ln_r;
assign dac_out_l_fast  = dac_out_l_fast_r;
assign dac_out_ln_fast = dac_out_ln_fast_r;

assign dac_out_r  = dac_out_r_r;
assign dac_out_rn = dac_out_rn_r;
    
endmodule