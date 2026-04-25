`timescale 1ns / 1ps

// interp_dac_thd_tb
// ─────────────────
// Drives a discrete-time sine into interpolator100x → dac, captures
// the 1-bit dout stream to a file, and dumps simulation parameters in
// a header so a Python (or Octave/Matlab) script can FFT the captured
// stream and read off harmonic content.
//
// Goal: prove (or disprove) that the entire digital path from
// interpolator input through modulator output is harmonic-distortion
// free. dac.v alone has already been shown linear under DC excitation;
// this exercises both blocks under audio-band sine excitation.
//
// What the script needs to do
// ────────────────────────────
//   bits = np.loadtxt('dout.bits.txt', dtype=np.int8)
//   x    = bits * 2.0 - 1.0                # ±1 from 1/0
//   # Decimate-and-LP filter, then FFT, then read H1/H3/H5 in dB
//   # (decimation factor MCLK_HZ / (2*F_AUDIO_BANDWIDTH))
//
// Simulation budget
// ─────────────────
// CAPTURE_CYCLES = 2^20 ≈ 1M cycles. At 100 MHz nominal sim clock
// that's 10.5 ms of "real" time. Bit file is ~1 MiB. Sim runs in
// a few minutes on a typical box.

module interp_dac_thd_tb;

    // ── Knobs ───────────────────────────────────────────────────
    localparam integer WIDTH       = 32;          // matches real design
    localparam integer ORDER       = 3;
    localparam integer MCLK_HZ     = 100_000_000; // simulation clock
    localparam integer INPUT_PERIOD = 1224;       // mclk cycles per input sample
                                                  // (44.1 kHz @ 54 MHz → 1224;
                                                  // we re-use the same ratio so the
                                                  // FIR's frequency response is right)

    // Tone frequency: coherent with capture window so the FFT has no
    // leakage. F_TONE = K * MCLK_HZ / CAPTURE_CYCLES.
    // K = 11 → ~1049 Hz with the defaults below.
    localparam integer CAPTURE_CYCLES = 1 <<< 20; // 1,048,576
    localparam integer K_CYCLES       = 11;
    // Tone period in mclk cycles, as a real for accurate phase math.
    real TONE_PERIOD_MCLK;
    initial TONE_PERIOD_MCLK = (CAPTURE_CYCLES * 1.0) / K_CYCLES;

    // Tone amplitude as fraction of FS. Keep modest so the 3rd-order
    // modulator stays in its stable region (the design notes say
    // <~50% FS for ORDER=3).
    real TONE_AMP_FS;
    initial TONE_AMP_FS = 0.30;

    // Output file
    localparam OUT_FILE = "dout.bits.txt";

    // ── Clock/reset ─────────────────────────────────────────────
    reg clk = 0;
    always #5 clk = ~clk;            // 100 MHz
    reg rst = 1;

    // ── Sine generator ──────────────────────────────────────────
    // Free-running phase counter at the input sample rate. We tick
    // a new sample into the interpolator every INPUT_PERIOD mclks.
    integer  in_div     = 0;
    integer  in_count   = 0;
    real     phase_rad  = 0.0;
    real     phase_inc;
    initial  phase_inc  = 6.283185307179586 * INPUT_PERIOD / TONE_PERIOD_MCLK;

    reg                       in_valid = 1'b0;
    reg  signed [WIDTH-1:0]   in_data  = 0;
    wire                      in_ready;

    real    sine_real;
    real    fs_real;
    initial fs_real = (2.0**(WIDTH-1)) - 1.0;

    always @(posedge clk) begin
        if (rst) begin
            in_div    <= 0;
            in_count  <= 0;
            phase_rad <= 0.0;
            in_valid  <= 1'b0;
            in_data   <= 0;
        end else begin
            // Clear valid once accepted.
            if (in_valid && in_ready)
                in_valid <= 1'b0;

            if (in_div == INPUT_PERIOD - 1) begin
                in_div <= 0;
                // Compute next sample
                sine_real = $sin(phase_rad) * TONE_AMP_FS * fs_real;
                in_data   <= $rtoi(sine_real);
                in_valid  <= 1'b1;
                phase_rad <= phase_rad + phase_inc;
                in_count  <= in_count + 1;
            end else begin
                in_div <= in_div + 1;
            end
        end
    end

    // ── DUT 1: interpolator100x ─────────────────────────────────
    wire                      interp_valid;
    wire signed [WIDTH-1:0]   interp_data;

    interpolator100x #(
        .WIDTH(WIDTH)
    ) interp_inst (
        .aclk(clk),
        .s_axis_data_tvalid(in_valid),
        .s_axis_data_tready(in_ready),
        .s_axis_data_tdata (in_data),
        .m_axis_data_tvalid(interp_valid),
        .m_axis_data_tdata (interp_data)
    );

    // ── DUT 2: dac ──────────────────────────────────────────────
    // Note: this test feeds the interpolator's bursty output straight
    // into the dac without rate-leveling. Distortion from that uneven
    // pacing was the original reason the output FIFO was added in
    // root.v. For a clean digital-only THD measurement we DO want to
    // include the rate-leveling here too — otherwise we'll be measuring
    // a known artefact rather than residual modulator/interpolator
    // nonlinearity.
    //
    // Simplest substitute: same 100x NCO that asrc_tick uses. We don't
    // need the PI servo (no FIFO to track). Just generate a tick every
    // INPUT_PERIOD/100 mclk cycles, drained from a small holding FIFO.
    //
    // For brevity here we use a tiny in-line shim instead of pulling
    // in the whole asrc module.

    // Output rate-leveling tick: fires every INPUT_PERIOD/100 cycles.
    localparam integer TICK_X100_PERIOD = INPUT_PERIOD / 100;  // 12 for 1224
    integer            tick_div = 0;
    reg                tick_x100 = 1'b0;
    always @(posedge clk) begin
        if (rst) begin
            tick_div <= 0;
            tick_x100 <= 1'b0;
        end else begin
            if (tick_div == TICK_X100_PERIOD - 1) begin
                tick_div  <= 0;
                tick_x100 <= 1'b1;
            end else begin
                tick_div  <= tick_div + 1;
                tick_x100 <= 1'b0;
            end
        end
    end

    // Small synchronous FIFO between interpolator and DAC.
    localparam integer FIFO_DEPTH = 64;
    localparam integer FIFO_CW    = $clog2(FIFO_DEPTH+1);
    reg signed [WIDTH-1:0] fmem [0:FIFO_DEPTH-1];
    reg [FIFO_CW-1:0] fwr = 0, frd = 0, fcnt = 0;
    wire fifo_full  = (fcnt == FIFO_DEPTH);
    wire fifo_empty = (fcnt == 0);

    reg primed = 1'b0;
    always @(posedge clk) begin
        if (rst) primed <= 1'b0;
        else if (fcnt >= FIFO_DEPTH/2) primed <= 1'b1;
        else if (fifo_empty) primed <= 1'b0;
    end

    wire fpush = interp_valid & ~fifo_full;
    wire fpop  = tick_x100 & primed & ~fifo_empty;

    always @(posedge clk) begin
        if (rst) begin
            fwr  <= 0;
            frd  <= 0;
            fcnt <= 0;
        end else begin
            if (fpush) begin
                fmem[fwr] <= interp_data;
                fwr <= (fwr == FIFO_DEPTH-1) ? 0 : fwr + 1;
            end
            if (fpop) begin
                frd <= (frd == FIFO_DEPTH-1) ? 0 : frd + 1;
            end
            case ({fpush, fpop})
                2'b10: fcnt <= fcnt + 1;
                2'b01: fcnt <= fcnt - 1;
                default: ;
            endcase
        end
    end

    wire signed [WIDTH-1:0] dac_in = fmem[frd];
    wire                    dac_dv = fpop;
    wire                    dout;

    dac #(
        .WORDLENGTH(WIDTH),
        .ORDER(ORDER)
    ) dac_inst (
        .clk           (clk),
        .rst           (rst),
        .din           (dac_in),
        .dvalid        (dac_dv),
        .dither1       (32'd0),     // dither off — measure raw modulator
        .dither2       (32'd0),
        .dout          (dout),
        .din_held_debug()
    );

    // ── Capture ─────────────────────────────────────────────────
    integer fh;
    integer cap_cnt;
    reg     capturing;

    initial begin
        capturing = 1'b0;
        cap_cnt   = 0;

        $display("");
        $display("interp_dac_thd_tb");
        $display("  WIDTH         = %0d", WIDTH);
        $display("  ORDER         = %0d", ORDER);
        $display("  MCLK_HZ       = %0d", MCLK_HZ);
        $display("  INPUT_PERIOD  = %0d  (Fs = %0.3f kHz)",
                 INPUT_PERIOD, MCLK_HZ * 1.0 / INPUT_PERIOD / 1000.0);
        $display("  TICK_X100_PER = %0d", TICK_X100_PERIOD);
        $display("  CAPTURE_CYC   = %0d", CAPTURE_CYCLES);
        $display("  K_CYCLES      = %0d", K_CYCLES);
        $display("  TONE_HZ       = %0.3f", K_CYCLES * MCLK_HZ * 1.0 / CAPTURE_CYCLES);
        $display("  TONE_AMP_FS   = %0.4f", TONE_AMP_FS);
        $display("  OUT_FILE      = %s", OUT_FILE);
        $display("");

        // Reset
        rst = 1;
        repeat (32) @(posedge clk);
        rst = 0;

        // Let things settle: prime the interpolator pipeline and the
        // FIFO. ~5000 input samples = enough for the modulator to
        // forget initial transients.
        repeat (50_000) @(posedge clk);

        // Open output file and write header
        fh = $fopen(OUT_FILE, "w");
        if (fh == 0) begin
            $display("ERROR: cannot open %s", OUT_FILE);
            $finish;
        end
        $fdisplay(fh, "# interp_dac_thd_tb capture");
        $fdisplay(fh, "# mclk_hz=%0d", MCLK_HZ);
        $fdisplay(fh, "# tick_x100_period=%0d", TICK_X100_PERIOD);
        $fdisplay(fh, "# capture_cycles=%0d", CAPTURE_CYCLES);
        $fdisplay(fh, "# k_cycles=%0d", K_CYCLES);
        $fdisplay(fh, "# tone_hz=%0.6f", K_CYCLES * MCLK_HZ * 1.0 / CAPTURE_CYCLES);
        $fdisplay(fh, "# tone_amp_fs=%0.6f", TONE_AMP_FS);
        $fdisplay(fh, "# format: one bit per line, captured at every posedge clk");

        capturing = 1'b1;
        while (cap_cnt < CAPTURE_CYCLES) begin
            @(posedge clk);
            $fdisplay(fh, "%b", dout);
            cap_cnt = cap_cnt + 1;
        end
        capturing = 1'b0;
        $fclose(fh);

        $display("Capture complete: %0d bits written to %s",
                 CAPTURE_CYCLES, OUT_FILE);
        $display("");
        $display("Suggested Python post-processing:");
        $display("  import numpy as np");
        $display("  bits = np.loadtxt('%s', dtype=np.int8, comments='#')", OUT_FILE);
        $display("  x    = bits.astype(float)*2 - 1");
        $display("  # decimate by averaging tick_x100_period samples (no filter, just box)");
        $display("  # then window+FFT to read H1, H3, H5 in dB");
        $display("");

        $finish;
    end

    // Watchdog
    initial begin
        // Allow generous slack: capture window plus settle + I/O.
        // Step in 1 s chunks to dodge 32-bit decimal-literal limits.
        repeat (20) #(1_000_000_000);
        $display("FATAL: testbench watchdog expired");
        $finish;
    end

endmodule
