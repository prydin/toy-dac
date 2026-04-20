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
    output wire [3:0] led     // LEDs for mode and dither indication
);


parameter I2S_WORDLENGTH = 32;
parameter MODULATOR_WORDLENGTH = 32;

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

// Clock signals
wire mclk;
wire locked;

// Hold everything in reset until the PLL locks and mclk is stable.
// Without this, the DAC/FIR run on a glitching clock during PLL startup
// and accumulate corrupt state they never recover from.
wire rst = ~locked;

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

// LEDs: [1:0] = mode (00 = I2S, 01 = DDS, 10 = DC), [3] = dither enabled
assign led = {dither_en, 1'b0, mode};

// Debug connections – verify interpolator is producing output
wire signed [MODULATOR_WORDLENGTH-1:0] dac_held_left;
assign debug1 = filtered_valid_left;  // Interpolator output valid – should burst 100 pulses per input
assign debug2 = src_valid_left;       // Input valid to interpolator
assign debug3 = output_ready_left;    // Interpolator tready (high when idle)
assign debug4 = filtered_left[31];    // Interpolator output sign bit

// Main clock 
// Currently running at 128MHz which is a wild swag. Probably need to revisit
// this as we learn more.
clock main_clock
   (
    .mclk(mclk),            // output mclk
    .reset(1'b0),           // PLL starts freely; rst is derived from locked
    .locked(locked),        // output locked
    .clk_in1(clk)           // input clk_in1
);


// I2S input
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
    .left_ready(output_ready_left),
    .right_ready(output_ready_right)
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
        default: begin // MODE_I2S
            src_left       = i2s_left;
            src_right      = i2s_right;
            src_valid_left  = left_valid;
            src_valid_right = right_valid;
        end
    endcase
end


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
wire dac_dv_left  = BYPASS_INTERPOLATOR ? src_valid_left  : filtered_valid_left;
wire dac_dv_right = BYPASS_INTERPOLATOR ? src_valid_right : filtered_valid_right;

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
wire signed [MODULATOR_WORDLENGTH-1:0] dac_held_right;

wire dac_raw_l;
wire dac_raw_r;

dac #(
    .WORDLENGTH(MODULATOR_WORDLENGTH)
) dac_left (
    .clk(mclk),
    .rst(rst),
    .din(dac_in_left), 
    .dvalid(dac_dv_left),
    .dither1(dither1),
    .dither2(dither2),
    .dout(dac_raw_l),
    .din_held_debug(dac_held_left)
);


dac #(
    .WORDLENGTH(MODULATOR_WORDLENGTH)
) dac_right (
    .clk(mclk),
    .rst(rst),
    .din(dac_in_right), 
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