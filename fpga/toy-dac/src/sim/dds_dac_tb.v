`timescale 1ns / 1ps

//
// Simple testbench: DDS → DAC (no filters, no I2S)
// Run for ~5 ms to see several 1 kHz cycles.
//

module dds_dac_tb;

// ── Clock: 54 MHz ──
reg clk = 0;
always #9.259259 clk = ~clk;   // 54 MHz

// ── Reset ──
reg rst = 1;
initial begin
    #100;
    rst = 0;
end

// ── Simulation length ──
initial begin
    #5_000_000;
    $display("=== Simulation complete ===");
    $finish;
end

// ====================================================================
//  DDS – 1 kHz sine
// ====================================================================
wire signed [31:0] dds_data;
wire               dds_valid;

dds_compiler_0 dds_inst (
    .aclk            (clk),
    .m_axis_data_tvalid (dds_valid),
    .m_axis_data_tdata  (dds_data)
);

// ====================================================================
//  Dither (PRNG)
// ====================================================================
wire signed [31:0] dither1;
wire signed [31:0] dither2;

random #(
    .SEED1(64'hcafebabe01234567),
    .SEED2(64'hdeadbeef89abcdef)
) rng1 (
    .clk  (clk),
    .rst  (rst),
    .dout (dither1)
);

random #(
    .SEED1(64'h0f1e2d3c4b5a6978),
    .SEED2(64'h87654321fedcba98)
) rng2 (
    .clk  (clk),
    .rst  (rst),
    .dout (dither2)
);

// ====================================================================
//  DAC
// ====================================================================
wire        dac_out;
wire signed [31:0] din_held_debug;

dac #(
    .WORDLENGTH(32),
    .ORDER(2)
) dac_inst (
    .clk            (clk),
    .rst            (rst),
    .din            (dds_data),
    .dvalid         (dds_valid),
    .dither1        (dither1),
    .dither2        (dither2),
    .dout           (dac_out),
    .din_held_debug (din_held_debug)
);

// ====================================================================
//  Lowpass filter on PDM bitstream (simulation-only biquad IIR)
//  Maps 1-bit PDM to ±full-scale, then filters at ~100 kHz cutoff.
// ====================================================================
wire signed [31:0] pdm_signed = dac_out ? 32'sh7FFF_FFFF : 32'sh8000_0000;
wire signed [31:0] filtered_out;

lpf_biquad #(
    .FC(500000.0)
) lpf_inst (
    .clk  (clk),
    .rst  (rst),
    .din  (pdm_signed),
    .dout (filtered_out)
);

endmodule
