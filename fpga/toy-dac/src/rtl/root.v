module top(
    input wire clk,             // 12MHz clock from crystal
    output wire dac_out_l,      // DAC delta/sigma out (+)
    output wire dac_out_r,      // Main clock output for debugging
    output wire dac_out_ln,     // DAC delta/sigma out (−), complement of dac_out_l
    output wire dac_out_rn,     // Complement of dac_out_r
    output wire debug1,         // Debug output 
    output wire debug2,         // Debug output 
    output wire debug3,         // Debug output
    output wire debug4,         // Debug output
    input wire bclk,            // I2S bit clock
    input wire lrclk,           // I2S left-right clock
    input wire din,             // I2S serial data in
    input wire bypass,          // A/B test: HIGH = bypass ASRC, clock samples directly from I2S
    input wire [1:0] btn,       // Push buttons
    output wire [3:0] led,      // LEDs for mode, clock/status, and dither indication
    inout  wire i2c_scl,        // I2C SCL (input only; no clock stretching)
    inout  wire i2c_sda,        // I2C SDA (open-drain)
    output wire led0_b,         // RGB LED blue channel — blinks on each ASRC adjust
    output wire ext_clk1_enable,// Enable external clock 1
    output wire ext_clk2_enable,// Enable external clock 2
    input wire ext_clk1,        // External clock 1 input
    input wire ext_clk2         // External clock 2 input
);


parameter I2S_WORDLENGTH = 32;
parameter MODULATOR_WORDLENGTH = 32;
`include "dsm_coeffs.vh"        // DSM loop filter coefficients
parameter MODULATOR_ORDER = DSM_ORDER; // Get order from coefficient file

// Modulator update-rate divider. The 1-bit loop and the output pin
// advance once every MODULATOR_RATE_DIV sclks. sclk is now one of
// the two external audio oscillators (22.5792/24.576 MHz) instead of
// the old 54 MHz fabric mclk, so =2 lands close to the previous
// ~13.5 MHz modulation rate that bench testing preferred (~11.3/
// 12.3 MHz here). FIRST-PASS ESTIMATE — re-run the divider sweep
// bench experiment for the new sclk frequencies before trusting this.
// The IOB output flops still clock on sclk, preserving matched +/-
// skew.
parameter integer MODULATOR_RATE_DIV = 2;
// Clamp clocked digital silence to exact zero before the ASRC/bypass mux.
// The QA403 can leave low-level / stale LSB patterns while DATA appears
// silent; those patterns produced a 3 kHz family at 48 kHz. Treat very
// small received samples as silence and remove the lower-bit residue.
// Threshold is 2^(32-20): about -120 dBFS, far below real program 
// material but comfortably above stale-LSB residue.
parameter integer I2S_SILENCE_CLAMP = 0; 
parameter integer I2S_SILENCE_BITS  = 20;
// Mute/reset the I2S audio path if LRCLK disappears. Without this,
// rate_manager can remain in its last locked state forever because
// rate_detect only produces updates on LRCLK edges.
parameter integer I2S_CLOCK_TIMEOUT_MS = 50;

// ── Master clock (mclk) parameters ────────────────────────────
// mclk is the 54 MHz MMCM output; it now only runs housekeeping
// (buttons, rate_detect, rate_manager, LEDs). See `sclk_bridge`
// below for how the sample pipeline's own clock (sclk) is selected
// from the two external audio oscillators.
localparam integer MCLK_HZ          = 54_000_000;

// ── Sample pipeline (sclk) parameters ─────────────────────────
// sclk is whichever external oscillator is currently selected:
// ext_clk1 = 22.5792 MHz (44.1 kHz family), ext_clk2 = 24.576 MHz
// (48 kHz family) — both exactly 512 x their nominal audio rate.
// Unlike the old single-mclk design these ratios are now exact,
// family-independent constants rather than per-rate computed values:
//   SAMPLE_DIV   = sclk / Fs_nominal = 512 (both families)
//   ASRC_OUT_DIV = 64  -> Fs_out = sclk/64 = 8 x Fs_nominal (exact)
//   STEP_NOMINAL = round(2^32 x Fs_in/Fs_out) = round(2^32/8)
// SCLK_HZ_NOM only sizes elaboration-time-only timing (debounce/
// LED-stretch/DDS tone/CDC-timeout counters) that doesn't need to
// track the ~8% difference between the two family rates.
localparam integer SCLK_HZ_NOM       = 22_579_200;
localparam integer ASRC_OUT_DIV      = 64;
localparam integer SAMPLE_DIV        = 512;
localparam [31:0]  STEP_NOMINAL_ASRC = 32'h2000_0000;  // round(2^32/8)

// ── Mode selector: btn[0] cycles through four signal sources ──
// Mode 0 = I2S, Mode 1 = direct DDS 1 kHz test tone,
// Mode 2 = DDS through ASRC diagnostic, Mode 3 = Off (0V)
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
// mclk: 54 MHz from the onboard MMCM, drives housekeeping only
// (buttons, rate_detect, rate_manager, LEDs).
wire mclk;
wire mclk_locked;

// ── Pipeline (sample-domain) clock ──────────────────────────────
// sclk is selected between the two external audio-family
// oscillators by `sclk_bridge`/`external_clock`, based on the rate
// detected on mclk. The ENTIRE sample pipeline (I2S receiver, ASRC,
// DAC/DSM, output pads) runs on sclk so there is no continuous
// sample-data clock crossing — only the rare, debounced family-
// change event crosses domains (see sclk_bridge.v).
wire        sclk;
wire        prst;         // pipeline reset (sclk domain), incl. clock-switch settle
wire        family_sclk;  // 0 = 44.1k family (ext_clk1), 1 = 48k family (ext_clk2) — unused, kept for future per-family tuning
wire        clk_sel;      // to external_clock mux (mclk domain)
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
wire signed [31:0] dds_data;
reg         test_valid;
// Forward decl: 1-cycle pulse on each accepted I2S left sample.
// Defined later (= left_valid & output_ready_left); referenced by
// the debug-pin block above the i2s instantiation.
wire        i2s_left_accept;
// Forward decl: 1-mclk pulse on lrclk rising edge from the I2S receiver's
// internal synchronizer; referenced by the frame-integrity probe below.
wire        lrclk_pos_edge_w;
// Forward decl: I2C control-register (0x03) write pulse + data,
// referenced by the mode/dither_en housekeeping block below but
// driven by the registers instance further down.
wire        i2c_reg3_wr;
wire [7:0]  i2c_reg3_wdata;

// Forward declarations for asrc instance outputs / legacy diagnostic
// aliases — referenced by recovered_clk / debug assigns below before
// the asrc instance proper.
wire        asrc_in_consumed;
wire signed [31:0] asrc_dbg_step_adj;
wire [15:0]        asrc_dbg_servo_fifo_count;
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
wire btn1_db;
debounce #(
    .CLK_FREQ(MCLK_HZ)
) btn0_debounce (
    .clk(mclk),
    .rst(rst),
    .btn_in(btn[0]),
    .btn_out(btn0_db)
);

debounce #(
    .CLK_FREQ(MCLK_HZ)
) btn1_debounce (
    .clk(mclk),
    .rst(rst),
    .btn_in(btn[1]),
    .btn_out(btn1_db)
);

reg btn0_prev = 0;
reg btn1_prev = 0;
reg [1:0] mode = MODE_I2S;
reg dither_en = 1'b1;
// New I2C-only control-register bits (0x03 bits 2/3/4). No physical
// button drives these, so they only ever change on an I2C write.
reg bypass_interp_ctrl = 1'b0;   // bit2: bypass upsample interp filter (stub, not yet wired)
reg output_mute_ctrl   = 1'b0;   // bit3: force output pins low
reg input_mute_ctrl    = 1'b0;   // bit4: force I2S input data to zero
always @(posedge mclk) begin
    if (rst) begin
        btn0_prev          <= 0;
        btn1_prev          <= 0;
        mode               <= MODE_I2S;
        dither_en          <= 1'b1;
        bypass_interp_ctrl <= 1'b0;
        output_mute_ctrl   <= 1'b0;
        input_mute_ctrl    <= 1'b0;
    end else begin
        btn0_prev <= btn0_db;
        btn1_prev <= btn1_db;
        // Rising edge of debounced button
        if (btn0_db && !btn0_prev) begin
            if (mode == NUM_MODES - 1)
                mode <= 0;
            else
                mode <= mode + 1;
        end
        if (btn1_db && !btn1_prev)
            dither_en <= ~dither_en;

        // I2C write to control register 0x03 — last-writer-wins against
        // the buttons for the bits they share (dither/mode).
        if (i2c_reg3_wr) begin
            dither_en          <= i2c_reg3_wdata[0];
            mode               <= i2c_reg3_wdata[1] ? MODE_DDS : MODE_I2S;
            bypass_interp_ctrl <= i2c_reg3_wdata[2];
            output_mute_ctrl   <= i2c_reg3_wdata[3];
            input_mute_ctrl    <= i2c_reg3_wdata[4];
        end
    end
end

// LEDs: [1:0] = mode, [2] = mclk_locked, [3] = dither enabled.
assign led = {dither_en, mclk_locked, mode};

// mode / dither_en are housekeeping state (button-driven, mclk
// domain) consumed by pipeline muxes on sclk. They only change on
// rare manual button presses, so a plain 2-FF sync per bit is
// acceptable — worst case is one extra sclk cycle of the old value,
// no different in kind from any manual mode switch causing a blip.
reg [1:0] mode_sync = MODE_I2S, mode_sclk_r = MODE_I2S;
always @(posedge sclk) begin
    mode_sync   <= mode;
    mode_sclk_r <= mode_sync;
end
wire [1:0] mode_sclk = mode_sclk_r;

reg dither_en_sync = 1'b1, dither_en_sclk_r = 1'b1;
always @(posedge sclk) begin
    dither_en_sync   <= dither_en;
    dither_en_sclk_r <= dither_en_sync;
end
wire dither_en_sclk = dither_en_sclk_r;

reg output_mute_sync = 1'b0, output_mute_sclk_r = 1'b0;
always @(posedge sclk) begin
    output_mute_sync   <= output_mute_ctrl;
    output_mute_sclk_r <= output_mute_sync;
end
wire output_mute_sclk = output_mute_sclk_r;

reg input_mute_sync = 1'b0, input_mute_sclk_r = 1'b0;
always @(posedge sclk) begin
    input_mute_sync   <= input_mute_ctrl;
    input_mute_sclk_r <= input_mute_sync;
end
wire input_mute_sclk = input_mute_sclk_r;

// Debug pins.
//   debug1 = stretched ASRC servo-adjust pulse.
//   debug2 =Selected external clock (sclk) for oscilloscope probing. 
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
// ── Frame-integrity probe ────────────────────────────────────────
// Runs on sclk (same domain as i2s_inst now). Use the same edge
// pulses i2s.v consumes. We want bclk_pos_edge and lrclk_pos_edge as
// 1-cycle sclk-synchronous pulses; flop_sync inside i2s.v exposes
// lrclk_pos_edge already (lrclk_pos_edge_w). For bclk edge counts we
// re-synchronize bclk locally so we don't have to thread an extra
// port through i2s.v.
reg bclk_s1 = 1'b0, bclk_s2 = 1'b0, bclk_s3 = 1'b0;
always @(posedge sclk) begin
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
always @(posedge sclk) begin
    if (prst) begin
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
localparam integer ANOM_PULSE = SCLK_HZ_NOM / 200;
always @(posedge sclk) begin
    if (prst) anom_stretch <= 20'd0;
    else if (anomaly_pulse) anom_stretch <= ANOM_PULSE;
    else if (anom_stretch != 20'd0) anom_stretch <= anom_stretch - 1'b1;
end
assign debug3 = (anom_stretch != 20'd0);

assign debug4 = i2s_left_accept &&
                (i2s_left[I2S_WORDLENGTH-1 -: 24] == 24'd0 ||
                 i2s_left[I2S_WORDLENGTH-1 -: 24] == 24'hFFFFFF);

// ── Sink-domain clock (phase-shifted by soft_pll) ────────────────
// mclk with dynamic phase shift enabled. psclk is tied to mclk
// itself, which is supported by the 7-series MMCM (psen/psincdec are
// sampled on psclk; soft_pll lives on mclk so this keeps everything in
// one domain).
clock main_clock (
    .mclk(mclk),                // output mclk (54 MHz)
    .reset(1'b0),               // PLL starts freely; rst is derived from locked
    .locked(mclk_locked),       // output locked
    .clk_in1(clk_ibuf),         // input ref clock (shared IBUFG output)
    .psclk(mclk),               // PS port clocked by mclk
    .psen(1'b0),                // ASRC handles rate matching, no PS pulses
    .psincdec(1'b0),
    .psdone(ps_done_unused)
);

// -- External clock ----
external_clock ext_clk_inst (
    .ext_clk1_in(ext_clk1),
    .ext_clk2_in(ext_clk2),
    .ext_clk1_enable(ext_clk1_enable),
    .ext_clk2_enable(ext_clk2_enable),
    .clk_sel(clk_sel),
    .clk_out(sclk)
);

// ── I2S input ────────────────────────────────────────────────────
// Runs on sclk. Its AXI-stream output goes two places:
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

// Runs on sclk (the selected external audio-family clock) — see
// the pipeline clock comment above.
i2s #(
    .WORDLENGTH(I2S_WORDLENGTH)
) i2s_inst (
    .clk(sclk),
    .rst(prst),
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

// ── Rate detection & management (mclk domain, housekeeping) ────
// rate_detect needs its own tap on the raw lrclk pin, independent
// of i2s_inst's synchronizer: i2s_inst now runs on sclk, whichever
// external clock is currently selected — not necessarily the right
// one until rate_detect/rate_manager have classified the incoming
// rate. rate_detect averages mclk cycles over WINDOW_SIZE lrclk
// periods (~5.8 ms @ 44.1 kHz) and emits `rate_valid` + `period`.
// rate_manager classifies into 32k/44.1k/48k, debounces, and drives
// `rate_code`, which `sclk_bridge` turns into the external-clock
// family select.
wire rate_lrclk_pos_edge;
flop_sync rate_lrclk_sync (
    .clk(mclk),
    .rst(rst),
    .in(lrclk),
    .out(),
    .neg_edge(),
    .pos_edge(rate_lrclk_pos_edge)
);

wire        rd_valid;
wire [31:0] rd_window_period;
wire [15:0] rd_period;

rate_detect #(
    .WINDOW_SIZE(256)
) rate_det (
    .clk           (mclk),
    .rst           (rst),
    .lrclk_pos_edge(rate_lrclk_pos_edge),
    .dvalid        (1'b1),
    .rate_valid    (rd_valid),
    .window_period (rd_window_period),
    .period        (rd_period)
);

// FS_OUT_HZ left at its module default: rm_step_nominal/rm_inc_nominal
// are legacy NCO-era outputs no longer consumed downstream now that
// the pipeline derives its own family-independent STEP_NOMINAL_ASRC.
rate_manager #(
    .MCLK_HZ         (MCLK_HZ),
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

// sclk_bridge: the sole clock-domain-crossing point between mclk
// (housekeeping) and sclk (sample pipeline) — see sclk_bridge.v.
sclk_bridge #(
    .SETTLE_CYCLES(MCLK_HZ / 1000)   // ~1 ms @ mclk
) sclk_bridge_inst (
    .sys_clk    (mclk),
    .sys_rst    (rst),
    .rate_code  (rm_rate_code),
    .clk_sel    (clk_sel),
    .sclk       (sclk),
    .pipe_rst   (prst),
    .family_sel (family_sclk)
);

// rm_mute / rm_asrc_rst are computed on mclk and only change on
// debounced (ms-scale) rate-lock transitions, so a 2-FF sync per
// bit into sclk is sufficient.
reg [1:0] rm_mute_sync = 2'b11;
always @(posedge sclk) rm_mute_sync <= {rm_mute_sync[0], rm_mute};
wire rm_mute_sclk = rm_mute_sync[1];

reg [1:0] rm_asrc_rst_sync = 2'b11;
always @(posedge sclk) rm_asrc_rst_sync <= {rm_asrc_rst_sync[0], rm_asrc_rst};
wire rm_asrc_rst_sclk = rm_asrc_rst_sync[1];

// sclk-domain mode decode (mode_sclk is the synchronized copy of the
// button-driven `mode` register).
wire        i2s_mode      = (mode_sclk == MODE_I2S);
wire        dds_asrc_mode = (mode_sclk == MODE_DC);

// ── A/B bypass switch ────────────────────────────────────────────
// `bypass` is tied to 3.3V (HIGH) to enable A/B test mode: the ASRC
// FIFOs and NCO are bypassed entirely, and I2S samples drive the
// interpolator directly on their natural lrclk-paced valid strobes.
// The interpolator output then drives the DAC directly (no output
// FIFO rate-leveling). This gives a clean reference signal to compare
// against the regenerated/rate-matched path.
//
// Synchronize the static-ish input through 2 FFs to sclk to avoid
// metastability if it's toggled live.
reg [1:0] bypass_sync = 2'b00;
always @(posedge sclk) bypass_sync <= {bypass_sync[0], bypass};
wire bypass_s = bypass_sync[1];
wire bypass_active = i2s_mode & bypass_s;

localparam integer I2S_CLOCK_TIMEOUT_CYCLES = (SCLK_HZ_NOM / 1000) * I2S_CLOCK_TIMEOUT_MS;
localparam integer I2S_CLOCK_TIMEOUT_W      = $clog2(I2S_CLOCK_TIMEOUT_CYCLES + 1);
reg [I2S_CLOCK_TIMEOUT_W-1:0] i2s_clock_timeout_cnt = I2S_CLOCK_TIMEOUT_CYCLES;

always @(posedge sclk) begin
    if (prst) begin
        i2s_clock_timeout_cnt <= I2S_CLOCK_TIMEOUT_CYCLES;
    end else if (lrclk_pos_edge_w) begin
        i2s_clock_timeout_cnt <= {I2S_CLOCK_TIMEOUT_W{1'b0}};
    end else if (i2s_clock_timeout_cnt < I2S_CLOCK_TIMEOUT_CYCLES) begin
        i2s_clock_timeout_cnt <= i2s_clock_timeout_cnt + 1'b1;
    end
end

wire i2s_clock_alive = (i2s_clock_timeout_cnt < I2S_CLOCK_TIMEOUT_CYCLES);

wire        asrc_rst_in   = prst | (i2s_mode & (rm_asrc_rst_sclk | ~i2s_clock_alive));
// Both families give the same Fs_in/Fs_out ratio (SAMPLE_DIV=512,
// ASRC_OUT_DIV=64 -> 1/8 exactly), so one STEP_NOMINAL_ASRC constant
// now serves every mode/family — no per-rate mux needed.
wire [31:0] asrc_step_nom = STEP_NOMINAL_ASRC;
// Bypass mode forces audio through immediately, so we override the
// rate-manager's mute (which would otherwise hold us silent for the
// 250 ms settle window every time rate_manager re-locks).
wire        audio_mute    = i2s_mode ? ((~bypass_s & rm_mute_sclk) | ~i2s_clock_alive) : 1'b0;

// ── Digital ASRC (fractional-phase, Phase 4 rewrite) ────────────
// The new `asrc` owns its own input ring buffers, polyphase filter
// banks, MAC engines and a PI servo on internal ring-buffer depth.
// It emits stereo on a fixed sclk/ASRC_OUT_DIV grid (8 x Fs_nominal
// exactly), bypassing the old NCO / AXI-loop / output-FIFO
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

localparam signed [I2S_WORDLENGTH-1:0] I2S_SILENCE_POS =
    32'sd1 <<< (I2S_WORDLENGTH - I2S_SILENCE_BITS);
localparam signed [I2S_WORDLENGTH-1:0] I2S_SILENCE_NEG = -I2S_SILENCE_POS;
wire i2s_left_silent  = (i2s_left  < I2S_SILENCE_POS) && (i2s_left  > I2S_SILENCE_NEG);
wire i2s_right_silent = (i2s_right < I2S_SILENCE_POS) && (i2s_right > I2S_SILENCE_NEG);

wire signed [I2S_WORDLENGTH-1:0] i2s_left_for_asrc  =
    (((I2S_SILENCE_CLAMP != 0) && i2s_left_silent)  || input_mute_sclk) ? {I2S_WORDLENGTH{1'b0}} : i2s_left;
wire signed [I2S_WORDLENGTH-1:0] i2s_right_for_asrc =
    (((I2S_SILENCE_CLAMP != 0) && i2s_right_silent) || input_mute_sclk) ? {I2S_WORDLENGTH{1'b0}} : i2s_right;

wire signed [I2S_WORDLENGTH-1:0] asrc_left_in =
    dds_asrc_mode ? dds_data : i2s_left_for_asrc;
wire signed [I2S_WORDLENGTH-1:0] asrc_right_in =
    dds_asrc_mode ? dds_data : i2s_right_for_asrc;
wire asrc_left_valid  = dds_asrc_mode ? test_valid : i2s_left_accept;
wire asrc_right_valid = dds_asrc_mode ? test_valid : i2s_right_accept;

// Run the ASRC for the normal I2S path and the internal DDS-through-ASRC diagnostic.
wire asrc_enable = i2s_mode | dds_asrc_mode;

asrc #(
    .WIDTH          (I2S_WORDLENGTH),
    .COEFF_W        (18),
    .PHASES         (256),
    .TAPS           (64),
    .COEFF_FILE     ("frac_asrc.mem"),
    .MCLK_HZ        (SCLK_HZ_NOM),
    .OUT_DIV        (ASRC_OUT_DIV),
    .SAMP_SETPOINT  (128),                         // mid-safe-range. Buffer is 4*TAPS=256 deep but the FIR
                                                   // can only see samples [C-63, C], so safe range is
                                                   // samp_avail in [1, 193]. Setpoint=128 leaves ~64 samples
                                                   // of slack on either side for crystal drift before
                                                   // hitting a rail.
    .SERVO_UPDATE_HZ(100),                         // pure-P loop, slew-limited; see frac_servo.v header
    .SERVO_ENABLE   (1),
    .STEP_NOMINAL   (STEP_NOMINAL_ASRC)
) asrc_inst (
    .clk                 (sclk),
    .rst                 (asrc_rst_in),
    .enable              (asrc_enable),
    .step_nominal_in     (asrc_step_nom),

    .i2s_left            (asrc_left_in),
    .i2s_right           (asrc_right_in),
    .left_valid          (asrc_left_valid),
    .right_valid         (asrc_right_valid),

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
    .dbg_servo_fifo_count (asrc_dbg_servo_fifo_count),
    .adjust              (asrc_adjust)
);

// Blue LED: stretched pulse on every ASRC servo adjust. Each adjust
// is a 1-cycle pulse at the 100 Hz servo update rate; the mono_ff
// stretches it to ~30 ms so it's visible to the eye. With the servo
// locked and the input rate stable, blinks should be sparse; faster
// blinking means the servo is actively chasing input drift.
wire led0_b_pulse;
mono_ff #(
    .FCLK     (SCLK_HZ_NOM),
    .DELAY_MS (50),
    .RESETTABLE(0)
) blue_led_stretch (
    .clk (sclk),
    .rst (prst),
    .d   (asrc_adjust),
    .q   (led0_b_pulse)
);
assign led0_b = ~led0_b_pulse;
assign debug1 = led0_b_pulse;
assign debug2 = sclk;

// ── Internal DDS test tone (1 kHz sine) ──
// DDS output is 26-bit Two's Complement in a 32-bit tdata field.
// Xilinx zero-pads [31:26].  Left-shift by 6 so the 26-bit sine
// fills the full 32-bit range the DAC expects.
wire [31:0] dds_raw;
wire        dds_valid;

dds #(
    .ACLK_HZ(SCLK_HZ_NOM)
) test_signal (
    .aclk(sclk),
    .m_axis_data_tvalid(dds_valid),
    .m_axis_data_tdata(dds_raw)
);

// Sign-extend and left-shift by 5 (not 6) to leave ~6 dB headroom.
// Peak ≈ ±2^30, well below the FIR's ±2^31 output clamp.
assign dds_data = {{1{dds_raw[25]}}, dds_raw[25:0], 5'b0};

// Sample-rate divider: ~Fs_nominal tick from sclk. SAMPLE_DIV=512 is
// exact for both families (sclk = 512 x Fs_nominal).
reg [$clog2(SAMPLE_DIV)-1:0] sample_cnt = 0;
initial test_valid = 0;

wire test_accepted = test_valid & output_ready_left;

always @(posedge sclk) begin
    if (prst) begin
        sample_cnt <= 0;
        test_valid <= 0;
    end else begin
        if (test_accepted)
            test_valid <= 0;
        if (sample_cnt == SAMPLE_DIV - 1) begin
            sample_cnt <= 0;
            test_valid <= 1;
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
    case (mode_sclk)
        MODE_DDS: begin
            src_left       = dds_data;
            src_right      = dds_data;
            src_valid_left  = dds_valid;
            src_valid_right = dds_valid;
        end
        MODE_DC: begin
            src_left       = {I2S_WORDLENGTH{1'b0}};
            src_right      = {I2S_WORDLENGTH{1'b0}};
            src_valid_left  = 1'b0;
            src_valid_right = 1'b0;
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


// ── I2C control/status interface ────────────────────────────────
// The I2C pins are top-level ports; register map and status synchronization
// live in registers.v.

registers #(
    .SCLK_HZ_NOM(SCLK_HZ_NOM)
) registers_inst (
    .mclk             (mclk),
    .rst              (rst),
    .sclk             (sclk),
    .prst             (prst),
    .rate_locked      (rm_rate_locked),
    .rate_code        (rm_rate_code),
    .asrc_enable      (asrc_enable),
    .asrc_samp_avail_l(asrc_samp_avail_l),
    .asrc_samp_avail_r(asrc_samp_avail_r),
    .dither_en        (dither_en),
    .mode             (mode),
    .bypass_interp_ctrl(bypass_interp_ctrl),
    .output_mute_ctrl (output_mute_ctrl),
    .input_mute_ctrl  (input_mute_ctrl),
    .servo_error      (asrc_dbg_error),
    .servo_step_adj   (asrc_dbg_step_adj),
    .servo_fifo_count (asrc_dbg_servo_fifo_count),
    .i2c_scl          (i2c_scl),
    .i2c_sda          (i2c_sda),
    .i2c_reg3_wr      (i2c_reg3_wr),
    .i2c_reg3_wdata   (i2c_reg3_wdata)
);


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
    .clk(sclk),
    .rst(prst),
    .dout(dither1_raw)
);

random #(
    .SEED1(64'h0f1e2d3c4b5a6978),
    .SEED2(64'h87654321fedcba98)
) rng2 (
    .clk(sclk),
    .rst(prst),
    .dout(dither2_raw)
);

wire [31:0] dither1 = dither_en_sclk ? dither1_raw : 32'd0;
wire [31:0] dither2 = dither_en_sclk ? dither2_raw : 32'd0;


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
wire i2s_path = (mode_sclk == MODE_I2S);
wire asrc_path = i2s_path | dds_asrc_mode;
wire signed [I2S_WORDLENGTH-1:0] dac_din_left  =
    (i2s_path && bypass_active) ? i2s_left_for_asrc   :
        (asrc_path)                 ? dac_in_left_paced   :
                                      src_left;
wire signed [I2S_WORDLENGTH-1:0] dac_din_right =
    (i2s_path && bypass_active) ? i2s_right_for_asrc  :
        (asrc_path)                 ? dac_in_right_paced  :
                                      src_right;
wire dac_dvalid_l =
        (i2s_path && bypass_active) ? left_valid          :
        (asrc_path)                 ? dac_dv_left         :
                                      src_valid_left;
wire dac_dvalid_r =
        (i2s_path && bypass_active) ? right_valid         :
        (asrc_path)                 ? dac_dv_right        :
                                      src_valid_right;

dac #(
    .WORDLENGTH(MODULATOR_WORDLENGTH),
    .ORDER(MODULATOR_ORDER),
    .COEFF_W(DSM_COEFF_W),
    .COEFF_FRAC(DSM_COEFF_FRAC),
    .N_G(DSM_N_G),
    .RATE_DIV(MODULATOR_RATE_DIV),
    .DSM_A(DSM_A),
    .DSM_C(DSM_C),
    .DSM_B1(DSM_B[0]),
    .DSM_G(DSM_G)
) dac_left (
    .clk(sclk),
    .rst(prst),
    .din(dac_din_left), 
    .dvalid(dac_dvalid_l),
    .dither1(dither1),
    .dither2(dither2),
    .dout(dac_raw_l)
);


dac #(
    .WORDLENGTH(MODULATOR_WORDLENGTH),
    .ORDER(MODULATOR_ORDER),
    .COEFF_W(DSM_COEFF_W),
    .COEFF_FRAC(DSM_COEFF_FRAC),
    .N_G(DSM_N_G),
    .RATE_DIV(MODULATOR_RATE_DIV),
    .DSM_A(DSM_A),
    .DSM_C(DSM_C),
    .DSM_B1(DSM_B[0]),
    .DSM_G(DSM_G)
) dac_right (
    .clk(sclk),
    .rst(prst),
    .din(dac_din_right), 
    .dvalid(dac_dvalid_r),
    .dither1(dither1),
    .dither2(dither2),
    .dout(dac_raw_r)
);

// Output registers for DAC — these get packed into IOB flip-flops
// via the IOB=TRUE constraint, ensuring + and − switch simultaneously.
//
// Each pad gets its own dedicated IOB flop, all clocked off the same
// mclk edge. The inversion happens BEFORE the flop on the (−) legs so
// the +/− pair has matched clock-to-out delay (sub-ps skew). 
(* IOB = "TRUE" *) reg dac_out_l_r       = 0;
(* IOB = "TRUE" *) reg dac_out_ln_r      = 0;
(* IOB = "TRUE" *) reg dac_out_r_r       = 0;
(* IOB = "TRUE" *) reg dac_out_rn_r      = 0;

always @(posedge sclk) begin
    if (prst || mode_sclk == MODE_OFF || audio_mute || output_mute_sclk) begin
        dac_out_l_r       <= 1'b0;
        dac_out_ln_r      <= 1'b0;
        dac_out_r_r       <= 1'b0;
        dac_out_rn_r      <= 1'b0;
    end else begin
        dac_out_l_r       <=  dac_raw_l;
        dac_out_ln_r      <= ~dac_raw_l;
        dac_out_r_r       <=  dac_raw_r;
        dac_out_rn_r      <= ~dac_raw_r;
    end
end

assign dac_out_l       = dac_out_l_r;
assign dac_out_ln      = dac_out_ln_r;

assign dac_out_r  = dac_out_r_r;
assign dac_out_rn = dac_out_rn_r;
    
endmodule