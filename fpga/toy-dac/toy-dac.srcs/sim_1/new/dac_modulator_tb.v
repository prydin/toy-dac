`timescale 1ns / 1ps
`default_nettype none

// dac_modulator_tb
// ────────────────
// Self-contained sim of dac.v alone. No IP, no asrc, no I2S.
// Reproduces the three test-mode inputs the hardware sees:
//   * DC = 0          (silent — should produce noise-shaped quantization
//                      noise at high frequency only, low audio-band noise)
//   * DC = +FS/1000   (MODE_DC)
//   * 1 kHz sine at -6 dBFS (MODE_DDS-equivalent)
//
// For each phase, we measure:
//   * mean of dout (should be ≈ din/FS in the [-1,+1] mapping → 0.5±din/2FS bit prob.)
//   * peak |sigma[k]| during the run (catches modulator instability)
//   * whether the comparator output toggled at all (catches a stuck bit)
//
// Also dumps a binary stream of dout samples into a .bin file for offline
// FFT/THD analysis if needed.
//
// Run:
//   xvlog -sv ../../src/rtl/dac.v dac_modulator_tb.v
//   xelab -debug typical dac_modulator_tb -s dac_sim
//   xsim dac_sim -R

module dac_modulator_tb;

    localparam integer WORDLENGTH = 32;
    localparam integer ORDER      = 2;

    // 108 MHz mclk to match HW exactly
    localparam real T_CLK_NS = 9.259259;   // 1e9 / 108e6
    reg clk = 1'b0;
    always #(T_CLK_NS/2.0) clk = ~clk;

    reg rst = 1'b1;

    reg  signed [WORDLENGTH-1:0] din    = 0;
    reg                           dvalid = 1'b0;
    // Real, time-varying dither (xoroshiro-style 32-bit LFSR pair).
    // Just enough randomness to make the modulator behave like it does
    // on hardware. Two independent Xorshift PRNGs.
    reg [31:0] dither1 = 32'hCAFEBABE;
    reg [31:0] dither2 = 32'hDEADBEEF;
    reg [31:0] x1, x2;
    always @(posedge clk) begin
        x1       = dither1 ^ (dither1 << 13);
        x1       = x1      ^ (x1      >> 17);
        dither1 <= x1      ^ (x1      << 5);

        x2       = dither2 ^ (dither2 << 17);
        x2       = x2      ^ (x2      >> 7);
        dither2 <= x2      ^ (x2      << 11);
    end

    wire dout;

    dac #(
        .WORDLENGTH(WORDLENGTH),
        .ORDER     (ORDER)
    ) dut (
        .clk    (clk),
        .rst    (rst),
        .din    (din),
        .dvalid (dvalid),
        .dither1(dither1),
        .dither2(dither2),
        .dout   (dout)
    );

    // ── Stats accumulators (reset between phases) ─────────────────
    integer phase = 0;
    integer ones_cnt = 0;
    integer toggle_cnt = 0;
    integer samples_cnt = 0;
    reg     last_dout = 1'b0;
    real    peak_sigma [0:2];
    integer i;

    // Probe internal sigma states (depth = WORDLENGTH+10 bits, signed)
    localparam integer ACCLENGTH = WORDLENGTH + 10;
    real sigma_real;

    initial begin
        for (i = 0; i < ORDER; i = i + 1) peak_sigma[i] = 0.0;
    end

    // Sigma magnitude tracker: poke each integrator each cycle.
    // We address them via hierarchical reference (Vivado xsim supports this).
    always @(posedge clk) begin
        if (!rst) begin
            // sigma[0] always exists
            sigma_real = $itor($signed(dut.sigma[0]));
            if (sigma_real <  0.0) sigma_real = -sigma_real;
            if (sigma_real > peak_sigma[0]) peak_sigma[0] = sigma_real;

            if (ORDER >= 2) begin
                sigma_real = $itor($signed(dut.sigma[1]));
                if (sigma_real <  0.0) sigma_real = -sigma_real;
                if (sigma_real > peak_sigma[1]) peak_sigma[1] = sigma_real;
            end

            if (ORDER >= 3) begin
                sigma_real = $itor($signed(dut.sigma[2]));
                if (sigma_real <  0.0) sigma_real = -sigma_real;
                if (sigma_real > peak_sigma[2]) peak_sigma[2] = sigma_real;
            end

            // dout statistics
            samples_cnt <= samples_cnt + 1;
            if (dout) ones_cnt <= ones_cnt + 1;
            if (dout != last_dout) toggle_cnt <= toggle_cnt + 1;
            last_dout <= dout;
        end
    end

    // ── Sine-wave generator (1 kHz @ 108 MHz mclk) ────────────────
    real sine_phase = 0.0;
    real sine_inc   = 6.283185307 * 1000.0 * T_CLK_NS * 1e-9;  // 2π·1k·dt
    integer sine_int;
    always @(posedge clk) begin
        if (phase == 3) begin
            sine_phase = sine_phase + sine_inc;
            if (sine_phase > 6.283185307) sine_phase = sine_phase - 6.283185307;
            // -6 dBFS sine → amplitude 0.5 × 2^31
            sine_int = $rtoi($cos(sine_phase) * (32'sh40000000));
            din    <= sine_int;
            dvalid <= 1'b1;
        end
    end

    // ── Test sequencer ───────────────────────────────────────────
    task reset_stats;
        integer kk;
        begin
            ones_cnt    = 0;
            toggle_cnt  = 0;
            samples_cnt = 0;
            for (kk = 0; kk < ORDER; kk = kk + 1) peak_sigma[kk] = 0.0;
        end
    endtask

    task report_phase;
        input [255:0] label;
        real frac_ones, toggle_rate;
        real sigma_max_norm; // as fraction of 2^WORDLENGTH (the input full scale)
        begin
            frac_ones    = $itor(ones_cnt) / $itor(samples_cnt);
            toggle_rate  = $itor(toggle_cnt) / $itor(samples_cnt);
            sigma_max_norm = peak_sigma[0];
            if (peak_sigma[1] > sigma_max_norm) sigma_max_norm = peak_sigma[1];
            if (peak_sigma[2] > sigma_max_norm) sigma_max_norm = peak_sigma[2];
            sigma_max_norm = sigma_max_norm / 2147483648.0;  // 2^31
            $display("=== %0s ===", label);
            $display("  samples           : %0d", samples_cnt);
            $display("  fraction of 1's   : %f  (expect ~0.5 for din=0, ~0.5+din/(2FS) otherwise)", frac_ones);
            $display("  bit-toggle rate   : %f  (expect ~0.5 for healthy noise-shaping)", toggle_rate);
            $display("  peak |sigma[0]|/FS: %f", peak_sigma[0] / 2147483648.0);
            $display("  peak |sigma[1]|/FS: %f", peak_sigma[1] / 2147483648.0);
            $display("  peak |sigma[2]|/FS: %f", peak_sigma[2] / 2147483648.0);
            if (sigma_max_norm > 8.0)
                $display("  >>> WARNING: integrator state > 8x FS — modulator may be unstable <<<");
            if (toggle_rate < 0.05)
                $display("  >>> WARNING: dout barely toggles — modulator may be stuck <<<");
        end
    endtask

    initial begin
        $display("dac_modulator_tb starting");
        // wait for reset deassert
        #200 rst = 1'b0;
        #50;

        // ── Phase 1: din = 0, no dvalid pulses (din_held stays 0) ──
        $display("Phase 1: din_held = 0 (mimics MODE_DDS/DC with broken dvalid)");
        phase  = 1;
        din    <= 32'sd0;
        dvalid <= 1'b0;
        reset_stats();
        #500_000;     // 500 us
        report_phase("Phase 1 (din_held=0, no dvalid)");

        // ── Phase 2: din = +FS/1000 with dvalid pulsed ──
        $display("Phase 2: DC = +FS/1000 (MODE_DC equivalent)");
        phase  = 2;
        din    <= 32'sh7FFFFFFF / 1000;
        dvalid <= 1'b1;
        reset_stats();
        #500_000;
        report_phase("Phase 2 (DC = +FS/1000)");

        // ── Phase 3: 1 kHz sine at -6 dBFS ──
        $display("Phase 3: 1 kHz sine, -6 dBFS (MODE_DDS equivalent)");
        phase = 3;
        reset_stats();
        // hold a fresh stats window; the sine generator drives din/dvalid
        #2_000_000;   // 2 ms — 2 full cycles
        report_phase("Phase 3 (sine -6 dBFS)");

        // ── Phase 4: din = 0 again, but dvalid pulsing (true silence) ──
        $display("Phase 4: din = 0 with dvalid asserted (true silence)");
        phase = 4;
        din    <= 32'sd0;
        dvalid <= 1'b1;
        reset_stats();
        #500_000;
        report_phase("Phase 4 (true silence din=0, dvalid=1)");

        $display("dac_modulator_tb done");
        $finish;
    end

endmodule

`default_nettype wire
