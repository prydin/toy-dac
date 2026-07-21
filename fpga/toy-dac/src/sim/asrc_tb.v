`timescale 1ns / 1ps
`default_nettype none

// asrc_tb
// ───────
// Stereo-level testbench for the rewritten `asrc` wrapper.
//
// Producer: emits I2S left+right samples at FS_IN_HZ (deliberately
// off-nominal so the PI servo has to track). Left channel is a sine
// at SIG_HZ_L, right at SIG_HZ_R, each at SIG_AMPL.
//
// Capture protocol
// ────────────────
// The servo's P+I dynamics dominate the first ~100 ms after enable,
// during which the sample window is dominated by transients. Drop
// SETTLE_MS of warm-up, then log:
//   - asrc_tb_audio.csv : <out_idx,time_ns,dac_left,dac_right>
//                         (CAPTURE_OUTS rows)
//   - asrc_tb_servo.csv : per-PI-update <time_ns,step,error,
//                                        samp_avail_l,samp_avail_r>
//
// What "good" looks like
// ──────────────────────
//   • Servo step converges within ~100 ms (KP=2048, KI=64, UPDATE_HZ=100).
//   • samp_avail_l/r oscillate gently around SAMP_SETPOINT (32).
//   • dac_left fits a 1 kHz sine with residual rms < 1e-4.
//   • dac_right fits a 1.5 kHz sine with similar residual.
//   • Outputs continue smoothly to end of capture; no underrun spikes.

module asrc_tb;

    // ── Parameters ──────────────────────────────────────────────────
    localparam integer WIDTH         = 32;
    localparam integer COEFF_W       = 18;
    localparam integer PHASES        = 256;
    localparam integer TAPS          = 64;
    localparam integer OUT_DIV       = 64;
    localparam integer MCLK_HZ       = 108_000_000;
    localparam real    MCLK_PERIOD_NS = 1000.0 / 108.0;          // ≈9.259 ns
    localparam [31:0]  STEP_44_1     = 32'd112_261_131;
    localparam [31:0]  STEP_48K      = 32'd122_175_407;

    // Stimulus — switch rate at compile time with +define+RATE_48K
`ifdef RATE_48K
    localparam real    FS_IN_HZ      = 48_022.0;   // +458 ppm above 48k nominal
    localparam [31:0]  STEP_NOM      = STEP_48K;
`else
    localparam real    FS_IN_HZ      = 44_120.0;   // +454 ppm above 44.1k nominal
    localparam [31:0]  STEP_NOM      = STEP_44_1;
`endif
    localparam real    SIG_HZ_L      = 1_000.0;
    localparam real    SIG_HZ_R      = 1_500.0;
    localparam integer SETTLE_MS     = 30;
    localparam integer CAPTURE_OUTS  = 8192;

    real sig_ampl = 1.0;
    initial begin
        if (!$value$plusargs("SIG_AMPL=%f", sig_ampl))
            sig_ampl = 1.0;
        $display("asrc_tb SIG_AMPL=%f FS", sig_ampl);
    end

    // ── Clock / reset ───────────────────────────────────────────────
    reg clk = 1'b0;
    always #(MCLK_PERIOD_NS/2.0) clk = ~clk;
    reg rst = 1'b1;

    // ── Producer NCO (real-rate -> mclk-aligned strobes) ────────────
    real    prod_inc_real = (FS_IN_HZ * 4294967296.0) / (1.0e9 / MCLK_PERIOD_NS);
    integer prod_inc      = 0;
    reg [31:0] prod_acc   = 32'd0;
    reg        sample_valid = 1'b0;

    initial prod_inc = $rtoi(prod_inc_real + 0.5);

    always @(posedge clk) begin
        if (rst) begin
            prod_acc     <= 32'd0;
            sample_valid <= 1'b0;
        end else begin
            {sample_valid, prod_acc} <= {1'b0, prod_acc} + {1'b0, prod_inc[31:0]};
        end
    end

    // Stereo sample values
    integer sample_count = 0;
    reg signed [WIDTH-1:0] sample_l = 0;
    reg signed [WIDTH-1:0] sample_r = 0;

    always @(posedge clk) begin
        if (rst) begin
            sample_count <= 0;
        end else if (sample_valid) begin
            sample_l <= $rtoi(sig_ampl * 32'sh7FFFFFFF
                              * $sin(2.0 * 3.141592653589793 * SIG_HZ_L
                                     * (sample_count / FS_IN_HZ)));
            sample_r <= $rtoi(sig_ampl * 32'sh7FFFFFFF
                              * $sin(2.0 * 3.141592653589793 * SIG_HZ_R
                                     * (sample_count / FS_IN_HZ)));
            sample_count <= sample_count + 1;
        end
    end

    // ── DUT ─────────────────────────────────────────────────────────
    wire signed [WIDTH-1:0] dac_left;
    wire signed [WIDTH-1:0] dac_right;
    wire                    dac_dv_left;
    wire                    dac_dv_right;
    wire        [15:0]      samp_avail_l;
    wire        [15:0]      samp_avail_r;
    wire                    in_consumed;
    wire        [31:0]      step_eff;
    wire signed [15:0]      servo_err;
    wire signed [31:0]      servo_step_adj;
    wire                    adjust;

    asrc #(
        .WIDTH        (WIDTH),
        .COEFF_W      (COEFF_W),
        .PHASES       (PHASES),
        .TAPS         (TAPS),
        .COEFF_FILE   ("src/rtl/frac_asrc.mem"),
        .MCLK_HZ      (MCLK_HZ),
        .OUT_DIV      (OUT_DIV),
        .SAMP_SETPOINT(TAPS),
        .SERVO_UPDATE_HZ(2000),
        .STEP_NOMINAL (STEP_NOM)
    ) dut (
        .clk                 (clk),
        .rst                 (rst),
        .enable              (1'b1),
        .step_nominal_in     (32'd0),               // use STEP_NOMINAL parameter
        .i2s_left            (sample_l),
        .i2s_right           (sample_r),
        .left_valid          (sample_valid),
        .right_valid         (sample_valid),
        .dac_left            (dac_left),
        .dac_right           (dac_right),
        .dac_dv_left         (dac_dv_left),
        .dac_dv_right        (dac_dv_right),
        .dbg_samples_avail_l (samp_avail_l),
        .dbg_samples_avail_r (samp_avail_r),
        .in_consumed         (in_consumed),
        .dbg_step            (step_eff),
        .dbg_servo_error     (servo_err),
        .dbg_servo_step_adj  (servo_step_adj),
        .adjust              (adjust)
    );

    // ── Logging ─────────────────────────────────────────────────────
    integer audio_csv = 0;
    integer servo_csv = 0;
    integer captured  = 0;
    integer total_outs = 0;
    integer sat_l = 0;
    integer sat_r = 0;
    reg signed [WIDTH-1:0] min_l = 32'sh7FFFFFFF;
    reg signed [WIDTH-1:0] max_l = -32'sh7FFFFFFF;
    reg signed [WIDTH-1:0] min_r = 32'sh7FFFFFFF;
    reg signed [WIDTH-1:0] max_r = -32'sh7FFFFFFF;
    real    settle_ns = SETTLE_MS * 1.0e6;
    reg     capturing = 1'b0;

    initial begin
        audio_csv = $fopen("asrc_tb_audio.csv", "w");
        servo_csv = $fopen("asrc_tb_servo.csv", "w");
        if (audio_csv == 0 || servo_csv == 0) begin
            $display("ERROR: could not open log CSVs");
            $finish;
        end
        $fwrite(audio_csv, "out_idx,time_ns,dac_left,dac_right\n");
        $fwrite(servo_csv, "time_ns,step,error,step_adj,samp_avail_l,samp_avail_r\n");
    end

    // Audio capture (windowed)
    always @(posedge clk) begin
        if (rst) begin
            captured   <= 0;
            total_outs <= 0;
            capturing  <= 1'b0;
            sat_l      <= 0;
            sat_r      <= 0;
            min_l      <= 32'sh7FFFFFFF;
            max_l      <= -32'sh7FFFFFFF;
            min_r      <= 32'sh7FFFFFFF;
            max_r      <= -32'sh7FFFFFFF;
        end else if (dac_dv_left) begin
            total_outs <= total_outs + 1;
            if (dac_left == 32'sh7FFFFFFF || dac_left == -32'sh80000000)
                sat_l <= sat_l + 1;
            if (dac_right == 32'sh7FFFFFFF || dac_right == -32'sh80000000)
                sat_r <= sat_r + 1;
            if (dac_left < min_l) min_l <= dac_left;
            if (dac_left > max_l) max_l <= dac_left;
            if (dac_right < min_r) min_r <= dac_right;
            if (dac_right > max_r) max_r <= dac_right;
            if (!capturing && $time >= settle_ns) capturing <= 1'b1;
            if (capturing) begin
                $fwrite(audio_csv, "%0d,%0t,%0d,%0d\n",
                        captured, $time, $signed(dac_left), $signed(dac_right));
                captured <= captured + 1;
                if (captured == CAPTURE_OUTS - 1) begin
                    $fclose(audio_csv);
                    $fclose(servo_csv);
                    $display("DONE: total_outs=%0d captured=%0d  step_eff=%0d  samp_avail=%0d/%0d",
                             total_outs + 1, captured + 1,
                             step_eff, samp_avail_l, samp_avail_r);
                    $display("ASRC range L=[%0d,%0d] R=[%0d,%0d] sat L/R=%0d/%0d",
                             min_l, max_l, min_r, max_r, sat_l, sat_r);
                    $finish;
                end
            end
        end
    end

    // Servo log on every PI update (10 ms apart)
    always @(posedge clk) begin
        if (!rst && adjust) begin
            $fwrite(servo_csv, "%0t,%0d,%0d,%0d,%0d,%0d\n",
                    $time, step_eff, servo_err, servo_step_adj,
                    samp_avail_l, samp_avail_r);
        end
    end

    // ── Reset / safety timeout ─────────────────────────────────────
    initial begin
        repeat (16) @(posedge clk);
        rst = 1'b0;
        // Hard timeout: 100 ms sim time.
        #(100.0e6);
        $display("ERROR: timeout (total_outs=%0d captured=%0d)",
                 total_outs, captured);
        $fclose(audio_csv);
        $fclose(servo_csv);
        $finish;
    end

endmodule

`default_nettype wire
