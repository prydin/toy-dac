module top(
    input wire clk,        // 100MHz clock from crystal
    output wire dac_out_l,     // DAC delta/sigma out
    output wire dac_out_r,     // Main clock output for debugging
    output wire debug1,        // Debug output 
    output wire debug2,        // Debug output 
    output wire debug3,        // Debug output
    output wire debug4,        // Debug output
    input wire bclk,       // I2S bit clock
    input wire lrclk,       // I2S left-right clock
    input wire din        // I2S serial data in
);


parameter I2S_WORDLENGTH = 32;
parameter MODULATOR_WORDLENGTH = 32;

// ── Set to 1 to bypass I2S and feed the interpolator from the internal
//    DDS test oscillator.  Set to 0 for normal I2S operation. ──
parameter USE_TEST_TONE = 0;

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

// ── Source mux: select I2S or internal test tone ──
wire signed [I2S_WORDLENGTH-1:0] src_left  = USE_TEST_TONE ? test_held  : i2s_left;
wire signed [I2S_WORDLENGTH-1:0] src_right = USE_TEST_TONE ? test_held  : i2s_right;
wire src_valid_left  = USE_TEST_TONE ? test_valid  : left_valid;
wire src_valid_right = USE_TEST_TONE ? test_valid  : right_valid;


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

// The DAC itself
wire signed [MODULATOR_WORDLENGTH-1:0] dac_held_right;

dac #(
    .WORDLENGTH(MODULATOR_WORDLENGTH)
) dac_left (
    .clk(mclk),
    .rst(rst),
    .din(dac_in_left), 
    .dvalid(dac_dv_left),
    .dout(dac_out_l),
    .din_held_debug(dac_held_left)
);


dac #(
    .WORDLENGTH(MODULATOR_WORDLENGTH)
) dac_right (
    .clk(mclk),
    .rst(rst),
    .din(dac_in_right), 
    .dvalid(dac_dv_right),
    .dout(dac_out_r),
    .din_held_debug(dac_held_right)
);
    
endmodule