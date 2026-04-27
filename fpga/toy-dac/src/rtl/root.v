module top(
    input wire clk,        // 100MHz clock from crystal
    output wire dac_out_l,     // DAC delta/sigma out (+)
    output wire dac_out_r,     // Main clock output for debugging
    output wire dac_out_ln,    // DAC delta/sigma out (−), complement of dac_out_l
    output wire dac_out_rn,    // Complement of dac_out_r
    output wire debug1,        // Debug output 
    output wire debug2,        // Debug output 
    output wire debug3,        // Debug output
    output wire debug4,        // Debug output
    input wire bclk,       // I2S bit clock
    input wire lrclk,       // I2S left-right clock
    input wire din,        // I2S serial data in
    input wire [1:0] btn,     // Push buttons
    output wire [3:0] led,    // LEDs for mode and dither indication
    output wire [3:0] fifo_led, // Thermometer indicator of FIFO fill (pio16-19)
    output wire led0_b        // RGB LED blue channel — blinks on each ASRC adjust
);


parameter I2S_WORDLENGTH = 32;
parameter MODULATOR_WORDLENGTH = 32;
parameter MODULATOR_ORDER = 4;  // 2=safe/stable, 3=good SNR if input < ~50% FS

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
wire output_ready_left;
wire output_ready_right;

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

// LEDs: [1:0] = mode, [2] = mclk_locked, [3] = dither enabled
assign led = {dither_en, mclk_locked, mode};

// Debug pins — ASRC servo state.
assign debug1 = mclk_locked;          // PLL locked?
//assign debug2 = asrc_adjust;          // 1-cycle pulse per pi_out change
assign debug3 = asrc_tick_l;          // sample tick — should be ~44.1 kHz
assign debug4 = ~fifo_empty_l;        // anything in the FIFO?

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
// Runs on mclk. Its AXI-stream output feeds the FIFO, which acts
// as an elastic buffer between I2S writes (paced by the upstream
// source's bclk/lrclk) and the NCO-paced reads inside asrc.
wire fifo_full_l;
wire fifo_full_r;

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
    // Backpressure: stop accepting new samples when the FIFO is full.
    // (If we get here, the soft_pll loop has lost lock or hasn't
    // converged yet; the I2S receiver simply stalls.)
    .left_ready(~fifo_full_l),
    .right_ready(~fifo_full_r)
);

// ── Digital ASRC ────────────────────────────────────────────────
// The asrc module wraps:
//   - input FIFOs  (elastic buffer between bclk-paced I2S writes
//                   and NCO-paced reads)
//   - nco          (NCO + PI rate servo, plus a phase-locked 100×
//                   tick that paces the post-interpolator output)
//   - tick-driven  pop-and-hold + AXI-stream presentation
//   - output FIFOs (rate-leveling between the interpolator's bursty
//                   output and the uniformly-spaced DAC consumption)
//
// The interpolator stays external because it's a Xilinx IP block.
// The asrc module presents an AXI-stream loop around it.
localparam integer FIFO_DEPTH = 64;
localparam integer FIFO_CW    = $clog2(FIFO_DEPTH+1);

wire [FIFO_CW-1:0] fifo_rd_count_l;
wire fifo_empty_l;
wire asrc_tick_l;
wire asrc_adjust;
wire signed [15:0] asrc_dbg_error;
wire signed [31:0] asrc_dbg_inc_adj;

// AXI-stream loop through the external interpolator.
wire signed [I2S_WORDLENGTH-1:0] i2s_src_left;
wire signed [I2S_WORDLENGTH-1:0] i2s_src_right;
wire i2s_src_valid_l;
wire i2s_src_valid_r;

wire signed [I2S_WORDLENGTH-1:0] dac_in_left_paced;
wire signed [I2S_WORDLENGTH-1:0] dac_in_right_paced;
wire dac_dv_left;
wire dac_dv_right;

// Run the servo whenever we're in I2S mode. We deliberately do NOT
// gate on a non-zero fifo_count: when the FIFO is empty, the negative
// error is exactly what should be ramping the integrator down to slow
// the NCO and let depth recover.
wire asrc_enable = (mode == MODE_I2S);

asrc #(
    .WIDTH         (I2S_WORDLENGTH),
    .FIFO_DEPTH    (FIFO_DEPTH),
    .OUT_FIFO_DEPTH(64),
    .MCLK_HZ       (54_000_000),
    .INC_NOMINAL   (32'd3_507_557)   // 44.1 kHz @ 54 MHz mclk
) asrc_inst (
    .clk              (mclk),
    .rst              (rst),
    .enable           (asrc_enable),

    .i2s_left         (i2s_left),
    .i2s_right        (i2s_right),
    .left_valid       (left_valid),
    .right_valid      (right_valid),
    .fifo_full_l      (fifo_full_l),
    .fifo_full_r      (fifo_full_r),

    .src_left         (i2s_src_left),
    .src_right        (i2s_src_right),
    .src_valid_left   (i2s_src_valid_l),
    .src_valid_right  (i2s_src_valid_r),
    .src_ready_left   (output_ready_left),
    .src_ready_right  (output_ready_right),

    .interp_left      (dac_in_left),
    .interp_right     (dac_in_right),
    .interp_valid_left (dac_dv_left_raw),
    .interp_valid_right(dac_dv_right_raw),

    .dac_left         (dac_in_left_paced),
    .dac_right        (dac_in_right_paced),
    .dac_dv_left      (dac_dv_left),
    .dac_dv_right     (dac_dv_right),

    .fifo_count       (fifo_rd_count_l),
    .fifo_empty_l     (fifo_empty_l),
    .tick             (asrc_tick_l),
    .adjust           (asrc_adjust),
    .dbg_error        (asrc_dbg_error),
    .dbg_inc_adj      (asrc_dbg_inc_adj)
);

// Flash the blue LED on each ASRC adjust — this lets us visually confirm that
mono_ff #(
    .FCLK(54_000_000),
    .DELAY(50_000_000), // 50 ms delay
    .RESETTABLE(1'b0)
) asrc_adjust_sync_inst (
    .clk(mclk),
    .rst(rst),
    .d(asrc_adjust),
    .q(led0_b) // Blink the blue LED on each ASRC adjust
);

// Also route the ASRC adjust pulse to a debug pin for oscilloscope viewing. Stretch it to 1 ms so it's easier to see on the scope.
mono_ff #(
    .FCLK(54_000_000),
    .DELAY(100_000_000), // 1 ms delay
    .RESETTABLE(1'b0)
) asrc_adjust_sync_scope_inst (
    .clk(mclk),
    .rst(rst),
    .d(asrc_adjust),
    .q(debug2) // Blink the blue LED on each ASRC adjust
);



// ── Internal DDS test tone (1 kHz sine) ──
// DDS output is 26-bit Two's Complement in a 32-bit tdata field.
// Xilinx zero-pads [31:26].  Left-shift by 6 so the 26-bit sine
// fills the full 32-bit range the DAC expects.
wire [31:0] dds_raw;
wire        dds_valid;

dds_compiler_0 test_signal (
    .aclk(mclk),
    .m_axis_data_tvalid(dds_valid),
    .m_axis_data_tdata(dds_raw)
);

// Sign-extend and left-shift by 5 (not 6) to leave ~6 dB headroom.
// Peak ≈ ±2^30, well below the FIR's ±2^31 output clamp.
wire signed [31:0] dds_data = {{1{dds_raw[25]}}, dds_raw[25:0], 5'b0};

// Sample-rate divider: ~44.1 kHz tick from mclk.
// SAMPLE_DIV = mclk_freq / 44100.  Must update when PLL changes.
// 27 MHz → 612,  54 MHz → 1224
localparam SAMPLE_DIV = 1224;
reg [10:0] sample_cnt = 0;
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

// ── Set to 1 to bypass interpolator and feed DDS straight into the DAC ──
parameter BYPASS_INTERPOLATOR = 0;

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
        default: begin // MODE_I2S — samples are paced by the asrc module
            src_left        = i2s_src_left;
            src_right       = i2s_src_right;
            src_valid_left  = i2s_src_valid_l;
            src_valid_right = i2s_src_valid_r;
        end
    endcase
end

// (Tick-driven FIFO pop and AXI-stream presentation now live inside
// the asrc module.)

// ── Diagnostic: track the high-water mark of fifo_rd_count_l ────
// The thermometer was instantaneous and might have been missed by
// the eye. This holds the maximum observed count, refreshed once a
// second so we can see if the FIFO ever fills meaningfully. Each
// LED bit corresponds to a count threshold of >=8, >=16, >=32, >=48.
reg [FIFO_CW-1:0] count_max     = 0;
reg [FIFO_CW-1:0] count_max_lat = 0;
reg [25:0]        max_refresh   = 0;   // ~1.2 s at 54 MHz
always @(posedge mclk) begin
    if (rst) begin
        count_max     <= 0;
        count_max_lat <= 0;
        max_refresh   <= 0;
    end else begin
        if (fifo_rd_count_l > count_max)
            count_max <= fifo_rd_count_l;
        max_refresh <= max_refresh + 1'b1;
        if (max_refresh == 26'd0) begin
            count_max_lat <= count_max;
            count_max     <= 0;
        end
    end
end

reg [3:0] fifo_led_r = 4'b0000;
always @(posedge mclk) begin
    if (rst) begin
        fifo_led_r <= 4'b1010;  // visible startup pattern
    end else begin
        fifo_led_r[0] <= (count_max_lat >= 7'd8);
        fifo_led_r[1] <= (count_max_lat >= 7'd16);
        fifo_led_r[2] <= (count_max_lat >= 7'd32);
        fifo_led_r[3] <= (count_max_lat >= 7'd48);
    end
end
assign fifo_led = fifo_led_r;


wire filtered_valid_left;
wire signed [I2S_WORDLENGTH-1:0] filtered_left;

interpolator100x #(
  .WIDTH(I2S_WORDLENGTH)
 ) interpolator_left (
  .aclk(mclk),
  .s_axis_data_tvalid(src_valid_left),
  .s_axis_data_tready(output_ready_left),
  .s_axis_data_tdata(src_left),
  .m_axis_data_tvalid(filtered_valid_left),
  .m_axis_data_tdata(filtered_left)
);

wire filtered_valid_right;
wire signed [I2S_WORDLENGTH-1:0] filtered_right;

interpolator100x #(
  .WIDTH(I2S_WORDLENGTH)
) interpolator_right (
  .aclk(mclk),
  .s_axis_data_tvalid(src_valid_right),
  .s_axis_data_tready(output_ready_right),
  .s_axis_data_tdata(src_right),
  .m_axis_data_tvalid(filtered_valid_right),
  .m_axis_data_tdata(filtered_right)
);


// ── DAC input mux: interpolator output or direct DDS bypass ──
wire signed [I2S_WORDLENGTH-1:0] dac_in_left  = BYPASS_INTERPOLATOR ? src_left  : filtered_left;
wire signed [I2S_WORDLENGTH-1:0] dac_in_right = BYPASS_INTERPOLATOR ? src_right : filtered_right;
wire dac_dv_left_raw  = BYPASS_INTERPOLATOR ? src_valid_left  : filtered_valid_left;
wire dac_dv_right_raw = BYPASS_INTERPOLATOR ? src_valid_right : filtered_valid_right;

// (Output rate-leveling FIFOs now live inside the asrc module.
// dac_in_left/right + dac_dv_*_raw are wired to asrc.interp_*; the
// asrc module exposes dac_in_left_paced / dac_dv_left etc. for the
// DAC instances below.)

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

dac #(
    .WORDLENGTH(MODULATOR_WORDLENGTH),
    .ORDER(MODULATOR_ORDER)
) dac_left (
    .clk(mclk),
    .rst(rst),
    .din(dac_in_left_paced), 
    .dvalid(dac_dv_left),
    .dither1(dither1),
    .dither2(dither2),
    .dout(dac_raw_l),
    .din_held_debug(dac_held_left)
);


dac #(
    .WORDLENGTH(MODULATOR_WORDLENGTH),
    .ORDER(MODULATOR_ORDER)
) dac_right (
    .clk(mclk),
    .rst(rst),
    .din(dac_in_right_paced), 
    .dvalid(dac_dv_right),
    .dither1(dither1),
    .dither2(dither2),
    .dout(dac_raw_r),
    .din_held_debug(dac_held_right)
);

// Output registers for DAC — these get packed into IOB flip-flops
// via the IOB=TRUE constraint, ensuring + and − switch simultaneously.
(* IOB = "TRUE" *) reg dac_out_l_r  = 0;
(* IOB = "TRUE" *) reg dac_out_ln_r = 0;
(* IOB = "TRUE" *) reg dac_out_r_r  = 0;
(* IOB = "TRUE" *) reg dac_out_rn_r = 0;

always @(posedge mclk) begin
    if (rst || mode == MODE_OFF) begin
        dac_out_l_r  <= 1'b0;
        dac_out_ln_r <= 1'b0;
        dac_out_r_r  <= 1'b0;
        dac_out_rn_r <= 1'b0;
    end else begin
        dac_out_l_r  <=  dac_raw_l;
        dac_out_ln_r <= ~dac_raw_l;
        dac_out_r_r  <=  dac_raw_r;
        dac_out_rn_r <= ~dac_raw_r;
    end
end

assign dac_out_l  = dac_out_l_r;
assign dac_out_ln = dac_out_ln_r;
assign dac_out_r  = dac_out_r_r;
assign dac_out_rn = dac_out_rn_r;
    
endmodule