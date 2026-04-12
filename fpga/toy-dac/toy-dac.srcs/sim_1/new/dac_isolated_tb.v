`timescale 1ns / 1ps

//
// Isolated testbench: DDS → interpolator → DAC
// Also includes a direct DDS → DAC path (no filter) for comparison.
// Run for ~5ms to see several 1kHz cycles.
//

module dac_isolated_tb;

// ── Clock: 128 MHz (period ≈ 7.8125 ns) ──
reg clk = 0;
always #3.90625 clk = ~clk;   // 128 MHz

// ── Reset ──
reg rst = 1;
initial begin
    #100;
    rst = 0;
end

// ── Simulation length ──
initial begin
    #5_000_000;    // 5 ms — enough for several 1 kHz cycles
    $display("=== Simulation complete ===");
    $finish;
end


// ====================================================================
//  DDS – 1 kHz sine (same IP used in top)
// ====================================================================
wire signed [31:0] dds_data;
wire               dds_valid;

dds_compiler_0 dds_inst (
    .aclk(clk),
    .m_axis_data_tvalid(dds_valid),
    .m_axis_data_tdata(dds_data)
);


// ====================================================================
//  PATH A: DDS → DAC  (no filter, direct – baseline reference)
// ====================================================================
wire dac_direct_out;

dac #(
    .WORDLENGTH(32)
) dac_direct (
    .clk(clk),
    .rst(rst),
    .din(dds_data),
    .dvalid(dds_valid),
    .dout(dac_direct_out)
);


// ====================================================================
//  PATH B: DDS → interpolator → DAC  (the chain under test)
// ====================================================================

// -- Sample-rate divider: pulse at ~192 kHz (128 MHz / 667 ≈ 191.9 kHz) --
// Proper AXI-Stream: hold tvalid HIGH until tready acknowledges the transfer.
localparam SAMPLE_DIV = 667;       // 128e6 / 192e3 ≈ 667
reg [9:0] sample_cnt = 0;
reg       sample_valid = 0;
reg signed [31:0] dds_held = 0;

wire sample_accepted = sample_valid & interp_ready;  // AXI handshake

always @(posedge clk) begin
    if (rst) begin
        sample_cnt <= 0;
        sample_valid <= 0;
        dds_held <= 0;
    end else begin
        if (sample_accepted)
            sample_valid <= 0;                     // transfer done, deassert
        if (sample_cnt == SAMPLE_DIV - 1) begin
            sample_cnt <= 0;
            sample_valid <= 1;
            dds_held <= dds_data << 6;             // sample-and-hold the DDS output
        end else begin
            sample_cnt <= sample_cnt + 1;
        end
    end
end

// -- Interpolator --
wire        interp_ready;
wire        interp_valid;
wire signed [31:0] interp_data;

interpolator441 interp_inst (
    .aclk(clk),
    .s_axis_data_tvalid(sample_valid),
    .s_axis_data_tready(interp_ready),
    .s_axis_data_tdata(dds_held),
    .m_axis_data_tvalid(interp_valid),
    .m_axis_data_tdata(interp_data)
);

// -- DAC fed from interpolator --
wire dac_filtered_out;

dac #(
    .WORDLENGTH(32)
) dac_filtered (
    .clk(clk),
    .rst(rst),
    .din(interp_data),
    .dvalid(interp_valid),
    .dout(dac_filtered_out)
);


// ====================================================================
//  Monitoring – add these signals to your wave viewer
// ====================================================================
// dds_data        – raw DDS sine (signed 32-bit, look at analog view)
// dds_valid       – DDS tvalid
// interp_data     – post-filter sine
// interp_valid    – filter output tvalid
// dac_direct_out  – PDM bitstream from direct path (reference)
// dac_filtered_out– PDM bitstream from filtered path (under test)
//
// Also useful internal signals:
// dac_direct.din_held
// dac_direct.sigma
// dac_filtered.din_held
// dac_filtered.sigma

endmodule
