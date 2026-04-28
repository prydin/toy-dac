`timescale 1ns / 1ps
`default_nettype none

// fractional_asrc_tb
// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
// Bench for the fractional-phase polyphase ASRC core.
//
// What it does
// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
//   â€¢ Generates a 1 kHz sine input at "44.1 kHz" using a producer NCO
//     so the rate is bit-exact (push events on an mclk-aligned grid).
//   â€¢ Drives the DUT's fixed output strobe at mclk/64 = 1.6875 MHz
//     (corresponds to mclk = 108 MHz).
//   â€¢ Programs `step` for the 44.1 kHz â†’ 1.6875 MHz ratio:
//       step = round(44100/1687500 * 2**32) = 112_261_131
//   â€¢ Logs each output sample to fractional_asrc_tb.csv.
//
// Read with:
//
//     import pandas as pd, numpy as np, matplotlib.pyplot as plt
//     df = pd.read_csv("fractional_asrc_tb.csv")
//     y = df.data_out.values / (1<<31)        # normalise to Â±1
//     # Skip warm-up samples
//     y = y[1000:]
//     n = len(y); w = np.hanning(n)
//     S = 20*np.log10(np.maximum(np.abs(np.fft.rfft(y*w)), 1e-30))
//     S -= S.max()
//     f = np.fft.rfftfreq(n, 1/1.6875e6)
//     plt.semilogx(f, S); plt.ylim(-160, 5); plt.grid(); plt.show()
//
// What "good" looks like
// â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
//   â€¢ Fundamental at 1 kHz, no skirt.
//   â€¢ Image at 44.1k - 1k = 43.1 kHz suppressed below ~âˆ’95 dBc.
//   â€¢ No spurs above ~âˆ’90 dBc out to the Î”Î£ Nyquist (1.6875M / 2).
//
// Knobs
// â”€â”€â”€â”€â”€
//   FS_IN_HZ       â€” producer rate (try 44_100, 44_120 to test off-nom).
//   SIG_HZ         â€” input tone frequency.
//   N_OUT_SAMPLES  â€” number of resampled outputs to capture.

module fractional_asrc_tb;

    // â”€â”€ Parameters â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    localparam integer DATA_W       = 32;
    localparam integer COEFF_W      = 18;
    localparam integer PHASES       = 256;
    localparam integer TAPS         = 64;
    // Note: passed as a parameter override; xvlog (Verilog mode) accepts
    // string parameters declared without an explicit `string` type.

    // mclk: pick 108 MHz so mclk/64 = 1.6875 MHz output rate.
    localparam real    MCLK_PERIOD_NS = 1000.0 / 108.0;   // â‰ˆ9.259 ns
    localparam integer OUT_DIV       = 64;

    // Resampling step for 44.1 kHz â†’ 1.6875 MHz (Q0.32).
    localparam [31:0]  STEP_44_1     = 32'd112_261_131;

    // Stimulus
    // Producer is set ~0.2% above nominal so the input buffer always
    // has cushion (no servo in this isolation test). The DUT's `step`
    // is left at the nominal 44.1k value, so the spectrum reflects
    // only MAC/lerp quality, not buffer-depth dynamics.
    localparam real    FS_IN_HZ      = 44_200.0;
    localparam real    SIG_HZ        = 1_000.0;
    localparam real    SIG_AMPL      = 0.5;            // headroom-friendly
    localparam integer N_OUT_SAMPLES = 8192;

    // â”€â”€ Clock / reset â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    reg clk = 1'b0;
    always #(MCLK_PERIOD_NS/2.0) clk = ~clk;
    reg rst = 1'b1;

    // â”€â”€ Producer NCO: emits sample_valid at FS_IN_HZ â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    // Use real arithmetic at sim time to compute the increment so the
    // rate is exact regardless of MCLK_HZ.
    real    prod_inc_real = (FS_IN_HZ * 4294967296.0) / (1.0e9 / MCLK_PERIOD_NS);
    integer prod_inc      = 0;
    reg [31:0] prod_acc   = 32'd0;
    reg        sample_valid = 1'b0;

    initial begin
        prod_inc = $rtoi(prod_inc_real + 0.5);
    end

    always @(posedge clk) begin
        if (rst) begin
            prod_acc     <= 32'd0;
            sample_valid <= 1'b0;
        end else begin
            {sample_valid, prod_acc} <= {1'b0, prod_acc} + {1'b0, prod_inc[31:0]};
        end
    end

    // Sample value: 32-bit signed sine at SIG_HZ.
    real    t = 0.0;
    integer sample_count = 0;
    reg signed [DATA_W-1:0] sample_in = 0;

    always @(posedge clk) begin
        if (rst) begin
            t            <= 0.0;
            sample_count <= 0;
        end else if (sample_valid) begin
            sample_in    <= $rtoi(SIG_AMPL * 32'sh7FFFFFFF
                                  * $sin(2.0 * 3.141592653589793 * SIG_HZ * (sample_count / FS_IN_HZ)));
            sample_count <= sample_count + 1;
        end
    end

    // â”€â”€ Output strobe: every OUT_DIV mclks â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    reg [$clog2(OUT_DIV)-1:0] out_div_cnt = 0;
    reg out_strobe = 1'b0;
    always @(posedge clk) begin
        if (rst) begin
            out_div_cnt <= 0;
            out_strobe  <= 1'b0;
        end else begin
            if (out_div_cnt == OUT_DIV - 1) begin
                out_div_cnt <= 0;
                out_strobe  <= 1'b1;
            end else begin
                out_div_cnt <= out_div_cnt + 1;
                out_strobe  <= 1'b0;
            end
        end
    end

    // â”€â”€ DUT â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    wire signed [DATA_W-1:0] data_out;
    wire                     dvalid_out;
    wire                     in_consumed;
    wire        [31:0]       dbg_phase_acc;

    fractional_asrc #(
        .DATA_W    (DATA_W),
        .COEFF_W   (COEFF_W),
        .PHASES    (PHASES),
        .TAPS      (TAPS)
    ) dut (
        .clk          (clk),
        .rst          (rst),
        .enable       (1'b1),
        .sample_in    (sample_in),
        .sample_valid (sample_valid),
        .out_strobe   (out_strobe),
        .step         (STEP_44_1),
        .data_out     (data_out),
        .dvalid_out   (dvalid_out),
        .in_consumed  (in_consumed),
        .dbg_phase_acc(dbg_phase_acc),
        .dbg_mac_cyc  ()
    );

    // â”€â”€ Logging â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    integer csv;
    integer out_count = 0;
    integer in_count  = 0;

    initial begin
        csv = $fopen("fractional_asrc_tb.csv", "w");
        if (csv == 0) begin
            $display("ERROR: could not open fractional_asrc_tb.csv");
            $finish;
        end
        $fwrite(csv, "out_idx,time_ns,data_out,phase_acc,sample_in_now,in_count,wptr_now\n");
    end

    always @(posedge clk) begin
        if (sample_valid) in_count <= in_count + 1;
        if (dvalid_out) begin
            $fwrite(csv, "%0d,%0t,%0d,%0d,%0d,%0d,%0d\n",
                    out_count, $time, $signed(data_out),
                    dbg_phase_acc, $signed(sample_in), in_count,
                    dut.wptr);
            out_count <= out_count + 1;
            if (out_count == N_OUT_SAMPLES - 1) begin
                $fclose(csv);
                $display("DONE: in=%0d out=%0d  (ratio %.4f, expected %.4f)",
                         in_count, out_count + 1,
                         (out_count + 1) * 1.0 / in_count,
                         1687500.0 / FS_IN_HZ);
                $finish;
            end
        end
    end

    // â”€â”€ Reset / safety â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
    initial begin
        repeat (16) @(posedge clk);
        rst = 1'b0;
        // Hard timeout: ~10 ms sim time.
        #(10_000_000.0);
        $display("ERROR: timeout (in=%0d out=%0d)", in_count, out_count);
        $fclose(csv);
        $finish;
    end

endmodule

`default_nettype wire
