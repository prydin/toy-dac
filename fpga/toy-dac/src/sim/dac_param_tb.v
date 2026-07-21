`timescale 1ns / 1ps
`default_nettype none

// dac_param_tb
// ────────────
// Self-contained sim of the parametric dac.v with synthesised CIFB
// coefficients pulled in from dsm_coeffs.vh. Exercises the order-3
// (or higher) path that the legacy Pascal-only dac.v couldn't handle.
//
// What this proves
// ----------------
// 1. dac.v elaborates with array-parameter coefficients (DSM_A,
//    DSM_C, DSM_G) and a non-default DSM_B1 / N_G.
// 2. The resonator-feedback `g` term wires up correctly (would
//    cause σ to slowly drift if the sign were inverted).
// 3. With a synthesised stable NTF, the modulator reaches steady
//    state and the bit stream tracks the DC input — the same
//    conditions under which Pascal-3 rails in dac_modulator_tb.
//
// Outputs
// -------
//   xsim_dac_param/dac_bits.bin : raw 1-bit-per-byte stream (one byte
//                                  per mclk cycle, 0x00 or 0x01) for
//                                  cross-checking against
//                                  scripts/dsm_model.py.
//   stdout                      : per-phase summary of bit ratio,
//                                  toggle rate, and peak |sigma_k|/FS.
//
// Run
// ---
//   xvlog -sv ../../src/rtl/dac.v dac_param_tb.v -i ../../src/rtl
//   xelab -debug typical dac_param_tb -s dac_param_sim
//   xsim dac_param_sim -R

module dac_param_tb;

    // Coefficient header from synthesize_dsm.py. The `include must be
    // inside a module / generate scope; that's where the localparams
    // become visible.
    `include "dsm_coeffs.vh"

    localparam integer WORDLENGTH = 32;
    localparam integer RATE_DIV   = 8;

    // 108 MHz mclk to match HW.
    localparam real T_CLK_NS = 9.259259;
    reg clk = 1'b0;
    always #(T_CLK_NS/2.0) clk = ~clk;

    reg rst = 1'b1;

    reg  signed [WORDLENGTH-1:0] din    = 0;
    reg                           dvalid = 1'b0;

    // Two independent Xorshift PRNGs for TPDF dither (same seeds as
    // the legacy modulator tb so results compare directly).
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

    // ── DUT: parametric dac with synthesised coefficients ───────────
    dac #(
        .WORDLENGTH    (WORDLENGTH),
        .ORDER         (DSM_ORDER),
        .COEFF_W       (DSM_COEFF_W),
        .COEFF_FRAC    (DSM_COEFF_FRAC),
        .STATE_INT_BITS(4),
        .N_G           (DSM_N_G),
        .RATE_DIV      (RATE_DIV),
        .DSM_A         (DSM_A),
        .DSM_C         (DSM_C),
        .DSM_B1        (DSM_B[0]),
        .DSM_G         (DSM_G)
    ) dut (
        .clk    (clk),
        .rst    (rst),
        .din    (din),
        .dvalid (dvalid),
        .dither1(dither1),
        .dither2(dither2),
        .dout   (dout)
    );

    // ── Stats ───────────────────────────────────────────────────────
    integer ones_cnt    = 0;
    integer toggle_cnt  = 0;
    integer samples_cnt = 0;
    reg     last_dout   = 1'b0;
    real    peak_sigma [0:DSM_ORDER-1];
    integer i;

    // Track per-integrator extrema. The dut state width is set by the
    // module's STATE_FRAC = WORDLENGTH-1 = 31 → ±1 FS = 2^31.
    real sigma_real;
    always @(posedge clk) begin
        if (!rst) begin
            for (i = 0; i < DSM_ORDER; i = i + 1) begin
                sigma_real = $itor($signed(dut.sigma[i]));
                if (sigma_real < 0.0) sigma_real = -sigma_real;
                if (sigma_real > peak_sigma[i]) peak_sigma[i] = sigma_real;
            end
            samples_cnt <= samples_cnt + 1;
            if (dout)            ones_cnt   <= ones_cnt   + 1;
            if (dout != last_dout) toggle_cnt <= toggle_cnt + 1;
            last_dout <= dout;
        end
    end

    initial begin
        for (i = 0; i < DSM_ORDER; i = i + 1) peak_sigma[i] = 0.0;
    end

    task reset_stats;
        integer kk;
        begin
            ones_cnt    = 0;
            toggle_cnt  = 0;
            samples_cnt = 0;
            for (kk = 0; kk < DSM_ORDER; kk = kk + 1) peak_sigma[kk] = 0.0;
        end
    endtask

    task report_phase;
        input [255:0] label;
        real frac_ones, toggle_rate;
        integer kk;
        begin
            frac_ones   = $itor(ones_cnt)   / $itor(samples_cnt);
            toggle_rate = $itor(toggle_cnt) / $itor(samples_cnt);
            $display("=== %0s ===", label);
            $display("  samples         : %0d", samples_cnt);
            $display("  fraction of 1's : %f", frac_ones);
            $display("  toggle rate     : %f", toggle_rate);
            for (kk = 0; kk < DSM_ORDER; kk = kk + 1) begin
                $display("  peak |sigma[%0d]|/FS: %f",
                         kk, peak_sigma[kk] / 2147483648.0);
            end
        end
    endtask

    task run_sine_phase;
        input [255:0] label;
        input real amp_fs;
        input integer n_mclk_cycles;
        integer nn;
        real phase;
        real tone;
        begin
            $display("%0s: sine amp = %f FS", label, amp_fs);
            reset_stats();
            dvalid <= 1'b1;
            for (nn = 0; nn < n_mclk_cycles; nn = nn + 1) begin
                phase = 6.283185307179586 * 1000.0 * ($itor(nn) / 108000000.0);
                tone = amp_fs * $sin(phase);
                din <= $rtoi(tone * 2147483647.0);
                @(posedge clk);
            end
            report_phase(label);
        end
    endtask

    // ── Bit-stream dump ─────────────────────────────────────────────
    integer bin_fd;
    reg     dump_active = 1'b0;
    always @(posedge clk) begin
        if (!rst && dump_active) begin
            $fwrite(bin_fd, "%c", dout ? 8'h01 : 8'h00);
        end
    end

    // ── Test sequencer ──────────────────────────────────────────────
    initial begin
        $display("dac_param_tb starting (ORDER=%0d, N_G=%0d, RATE_DIV=%0d)",
             DSM_ORDER, DSM_N_G, RATE_DIV);
        bin_fd = $fopen("dac_bits.bin", "wb");

        #200 rst = 1'b0;
        #50;

        // Phase 1: zero input.
        $display("Phase 1: din = 0");
        din    <= 32'sd0;
        dvalid <= 1'b1;
        reset_stats();
        #500_000;
        report_phase("Phase 1 (din=0)");

        // Phase 2: DC = +0.5 FS — well inside umax for any sensible NTF.
        $display("Phase 2: din = +0.5 FS");
        din    <= 32'sh40000000;   // 2^30 = 0.5 of 2^31
        dvalid <= 1'b1;
        reset_stats();
        // Dump the +0.5 FS bit stream for cross-checking with
        // scripts/dsm_model.py --dc 0.5.
        dump_active = 1'b1;
        #500_000;
        dump_active = 1'b0;
        report_phase("Phase 2 (DC=+0.5 FS)");

        // Phase 3: DC = -0.5 FS (sanity: bit ratio should mirror).
        $display("Phase 3: din = -0.5 FS");
        din    <= -32'sh40000000;
        dvalid <= 1'b1;
        reset_stats();
        #500_000;
        report_phase("Phase 3 (DC=-0.5 FS)");

        // Phase 4/5: high-level sine stress at the deployed divider.
        // 0 dBFS on the external I2S path should not send the modulator
        // numerically unstable; if this rails, the coefficient set is too
        // aggressive even after the Hinf reduction.
        run_sine_phase("Phase 4 (sine=-5 dBFS)", 0.562341, 2_000_000);
        run_sine_phase("Phase 5 (sine=0 dBFS)", 1.000000, 2_000_000);

        $fclose(bin_fd);
        $display("dac_param_tb done — wrote dac_bits.bin");
        $finish;
    end

endmodule

`default_nettype wire
