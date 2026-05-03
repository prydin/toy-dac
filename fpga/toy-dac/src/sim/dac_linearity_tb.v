`timescale 1ns / 1ps

// dac_linearity_tb
// ─────────────────
// Sanity check that the digital ΔΣ modulator is linear: feed a steady
// DC value, let everything settle, then count output marks vs spaces
// over a long measurement window. Mark/space ratio (= duty cycle) must
// be a perfect linear function of the input.
//
// For this DAC's encoding (q_fb = ±2^WORDLENGTH, din range
// ±2^(WORDLENGTH-1)-1), the relationship is:
//
//      duty = 0.5 + din / 2^(WORDLENGTH+1)
//
// So a full-scale positive input gives duty ≈ 0.75; full-scale
// negative gives duty ≈ 0.25; din=0 gives duty=0.5.
//
// Dither is forced to zero so we measure the raw modulator's behaviour.
// Several DC levels are tested, with extra coverage near zero where
// limit-cycle artefacts are most likely to bias the average.

module dac_linearity_tb;

    // Use a smaller WORDLENGTH so simulation runs in reasonable time.
    // Linearity claims scale with WORDLENGTH; 16 bits is plenty to see
    // any structural nonlinearity in the modulator.
    localparam integer WORDLENGTH = 16;
    localparam integer ORDER      = 3;

    // Settling and measurement window lengths (in mclk cycles).
    // Settle long enough to flush ORDER stages of integrator state
    // (this can take many million cycles for inputs near the rails
    // because the integrators wind up to large values). Measure long
    // enough that the duty resolution is well below the linearity
    // tolerance we want to claim.
    localparam integer SETTLE_CYCLES  = 1_000_000;
    localparam integer MEASURE_CYCLES = 4_000_000;

    // Pass criterion: residual after best-fit linear scale must be
    // smaller than this fraction of full scale. Loose for a smoke
    // test; tighten if the modulator looks well-behaved.
    real LIN_TOL_PPM = 100.0;   // 100 ppm of FS

    // ── Clock/reset ─────────────────────────────────────────────
    reg clk = 0;
    always #5 clk = ~clk;          // 100 MHz sim clock
    reg rst = 1;

    // ── DUT ─────────────────────────────────────────────────────
    reg signed [WORDLENGTH-1:0] din    = 0;
    reg                         dvalid = 1'b1;       // always update
    wire                        dout;
    wire signed [WORDLENGTH-1:0] din_held_debug;

    dac #(
        .WORDLENGTH(WORDLENGTH),
        .ORDER(ORDER)
    ) dut (
        .clk           (clk),
        .rst           (rst),
        .din           (din),
        .dvalid        (dvalid),
        .dither1       (32'd0),       // dither off
        .dither2       (32'd0),
        .dout          (dout),
        .din_held_debug(din_held_debug)
    );

    // ── Measurement helpers ─────────────────────────────────────
    integer i;
    integer mark_count;
    real    duty;
    real    expected;
    real    err_lsb;
    real    err_ppm;

    // Test vector: a spread of DC levels, with extra coverage near
    // zero. Values are normalized fractions of (signed) full scale;
    // they get scaled to WORDLENGTH bits at run time.
    real test_levels [0:14];
    initial begin
        test_levels[ 0] = -0.95;
        test_levels[ 1] = -0.50;
        test_levels[ 2] = -0.25;
        test_levels[ 3] = -0.10;
        test_levels[ 4] = -0.01;
        test_levels[ 5] = -0.001;
        test_levels[ 6] = -1.0/(2.0**(WORDLENGTH-1));   // -1 LSB
        test_levels[ 7] =  0.0;
        test_levels[ 8] =  1.0/(2.0**(WORDLENGTH-1));   // +1 LSB
        test_levels[ 9] =  0.001;
        test_levels[10] =  0.01;
        test_levels[11] =  0.10;
        test_levels[12] =  0.25;
        test_levels[13] =  0.50;
        test_levels[14] =  0.95;
    end

    // Convert a normalized level (-1..+1) to a signed WORDLENGTH int.
    function signed [WORDLENGTH-1:0] to_din;
        input real lvl;
        real scaled;
        begin
            scaled = lvl * (2.0**(WORDLENGTH-1));
            if (scaled >  ((2.0**(WORDLENGTH-1)) - 1.0)) scaled = (2.0**(WORDLENGTH-1)) - 1.0;
            if (scaled < -(2.0**(WORDLENGTH-1)))         scaled = -(2.0**(WORDLENGTH-1));
            to_din = $rtoi(scaled);
        end
    endfunction

    // Run one DC point: apply, settle, count marks, return duty.
    task run_point;
        input  real lvl;
        output real duty_out;
        integer j;
        integer marks;
        begin
            din = to_din(lvl);
            // Settle
            for (j = 0; j < SETTLE_CYCLES; j = j + 1) @(posedge clk);
            // Measure
            marks = 0;
            for (j = 0; j < MEASURE_CYCLES; j = j + 1) begin
                @(posedge clk);
                if (dout) marks = marks + 1;
            end
            duty_out = marks * 1.0 / MEASURE_CYCLES;
        end
    endtask

    // ── Main ────────────────────────────────────────────────────
    integer  errors;
    integer  npoints;
    real     measured [0:14];
    real     expected_arr [0:14];
    real     resid_max_ppm;

    initial begin
        $dumpfile("dac_linearity_tb.vcd");
        // Don't dump dout for the whole run — file would be huge.
        // Comment out the next line if waveforms aren't needed.
        // $dumpvars(0, dac_linearity_tb);

        // Reset
        repeat (10) @(posedge clk);
        rst <= 0;
        repeat (10) @(posedge clk);

        errors = 0;
        npoints = 15;

        $display("");
        $display("dac_linearity_tb: WORDLENGTH=%0d ORDER=%0d", WORDLENGTH, ORDER);
        $display("                  SETTLE=%0d MEASURE=%0d cycles",
                 SETTLE_CYCLES, MEASURE_CYCLES);
        $display("");
        $display("    level (FS)      din          duty       expected    err (LSB)   err (ppm)");
        $display("    ----------     -----        ------      --------    ---------   ---------");

        for (i = 0; i < npoints; i = i + 1) begin
            run_point(test_levels[i], duty);
            measured[i]      = duty;
            // Expected duty: 0.5 + din / 2^(WORDLENGTH+1)
            expected         = 0.5 + (to_din(test_levels[i]) * 1.0) / (2.0**(WORDLENGTH+1));
            expected_arr[i]  = expected;
            err_lsb          = (duty - expected) * (2.0**(WORDLENGTH+1));
            err_ppm          = (duty - expected) * 1.0e6 / 0.5;  // ppm of half-scale duty range
            $display("    %+8.5f     %6d     %8.6f    %8.6f    %+8.3f    %+8.2f",
                     test_levels[i], to_din(test_levels[i]),
                     duty, expected, err_lsb, err_ppm);
        end

        // ── Linearity check ─────────────────────────────────────
        // Fit no parameters; just compare each measured duty to its
        // ideal value. Any deviation is either:
        //   (a) finite measurement window (random), or
        //   (b) modulator nonlinearity (systematic, what we care about).
        // For ORDER=3 with no dither, near-DC inputs *will* sit on
        // limit cycles whose mean is exactly correct, so this should
        // pass to many ppm.
        resid_max_ppm = 0.0;
        for (i = 0; i < npoints; i = i + 1) begin
            err_ppm = (measured[i] - expected_arr[i]) * 1.0e6 / 0.5;
            if (err_ppm < 0) err_ppm = -err_ppm;
            if (err_ppm > resid_max_ppm) resid_max_ppm = err_ppm;
        end

        $display("");
        $display("Max residual: %0.2f ppm of FS  (tolerance %0.0f ppm)",
                 resid_max_ppm, LIN_TOL_PPM);

        if (resid_max_ppm > LIN_TOL_PPM) begin
            $display("=== FAIL: modulator nonlinearity exceeds tolerance ===");
            errors = errors + 1;
        end else begin
            $display("=== PASS: modulator linear within tolerance ===");
        end

        if (errors == 0)
            $display("\n=== dac_linearity_tb: PASS ===\n");
        else
            $display("\n=== dac_linearity_tb: %0d FAILURE(S) ===\n", errors);

        $finish;
    end

    // Global watchdog
    initial begin
        // Each point: ~5e6 cycles * 10 ns = 50 ms. 15 points → ~750 ms.
        // Add slack.
        #(2_000_000_000);
        $display("FATAL: testbench watchdog expired");
        $finish;
    end

endmodule
