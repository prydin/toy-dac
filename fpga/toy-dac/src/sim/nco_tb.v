`timescale 1ns / 1ps
`default_nettype none

// nco_tb
// ──────
// Demonstrates the NCO + PI servo locking onto an input rate that
// differs from INC_NOMINAL.
//
// What it does
// ────────────
//   • Models a producer that pushes one sample into a counter (the
//     "FIFO depth") at a *different* rate than the NCO's nominal.
//   • Each NCO `tick` decrements that counter (a sample being popped).
//   • Every UPDATE_HZ window, dumps a CSV row:
//
//       time_ms, fifo_count, err, pi_out, inc_eff, meas_tick_hz
//
//   • Shuts down once we've watched enough updates.
//
// Read this output with e.g.:
//
//       import pandas as pd, matplotlib.pyplot as plt
//       df = pd.read_csv('nco_tb.csv')
//       fig, ax = plt.subplots(3, 1, sharex=True, figsize=(10,6))
//       ax[0].plot(df.time_ms, df.fifo_count); ax[0].set_ylabel('fifo')
//       ax[1].plot(df.time_ms, df.meas_tick_hz); ax[1].set_ylabel('Hz')
//       ax[2].plot(df.time_ms, df.pi_out);    ax[2].set_ylabel('pi_out')
//       plt.show()
//
// Knobs you'll typically twist:
//   INPUT_HZ_ACTUAL  — what the producer is really doing
//   UPDATE_HZ        — how fast the servo reacts (large = fast sim)
//   N_UPDATES        — how many servo updates to capture

module nco_tb;

    // ── Knobs ──────────────────────────────────────────────────────
    localparam integer MCLK_HZ           = 108_000_000;
    localparam integer FS_NOMINAL_HZ     = 44_100;
    // Producer is +0.5 % off nominal — well outside the servo
    // deadband, easily within the INC_ADJ clamp.
    localparam real    INPUT_HZ_ACTUAL   = 44_320.5;
    localparam integer FIFO_DEPTH        = 64;
    localparam integer SETPOINT          = FIFO_DEPTH/2;

    // Crank UPDATE_HZ way up so we converge in milliseconds of sim
    // time, not seconds. Real design uses 10 Hz; here 1 kHz keeps the
    // dynamics legible without taking forever.
    localparam integer UPDATE_HZ         = 1_000;
    localparam integer N_UPDATES         = 600;

    // Production NCO defaults are KP=32, KI=4 at UPDATE_HZ=10. The I
    // term integrates per update, so increasing UPDATE_HZ by R must
    // come with KI/R to keep the loop bandwidth (and stability) the
    // same. KP doesn't scale — it's a per-update proportional kick.
    // Without this, the integrator winds up 100× too fast, overshoots,
    // drains the FIFO past LOW_MARK, the prime gate deasserts, the PI
    // state is zeroed, and we limit-cycle at ~1.5 Hz instead of
    // converging.
    localparam integer KP_TB             = 32;
    localparam integer KI_TB             = 1;     // = 4 × (10 / 1000), rounded up

    // Derived
    localparam [31:0]  INC_NOMINAL =
        (((64'd1 << 32) * FS_NOMINAL_HZ) + (MCLK_HZ/2)) / MCLK_HZ;
    localparam integer FIFO_CW     = $clog2(FIFO_DEPTH+1);

    // ── Clock / reset ──────────────────────────────────────────────
    reg clk = 1'b0;
    always #(500.0 / (MCLK_HZ/1_000_000.0)) clk = ~clk;   // ≈18.5 ns half-period at 54 MHz
    reg rst = 1'b1;

    // ── Producer: push samples at INPUT_HZ_ACTUAL ─────────────────
    // Implemented with the same NCO trick: add a fixed increment per
    // mclk and use the carry-out as the push event. Increment is
    // computed at sim time using real arithmetic so we get any
    // requested fractional rate.
    reg [31:0] prod_acc  = 32'd0;
    reg [31:0] prod_inc  = 32'd0;
    reg        push      = 1'b0;
    initial begin
        // round(2^32 × INPUT_HZ_ACTUAL / MCLK_HZ)
        prod_inc = $rtoi((INPUT_HZ_ACTUAL * 4294967296.0) / MCLK_HZ + 0.5);
    end
    always @(posedge clk) begin
        if (rst) begin
            prod_acc <= 32'd0;
            push     <= 1'b0;
        end else begin
            {push, prod_acc} <= {1'b0, prod_acc} + {1'b0, prod_inc};
        end
    end

    // ── Tiny "FIFO": just a saturating fill counter ────────────────
    // The DUT only reads fifo_count, so this is the entire model.
    reg [FIFO_CW-1:0] fifo_count = 0;
    wire              tick;
    wire              tick_x100;
    always @(posedge clk) begin
        if (rst) begin
            fifo_count <= 0;
        end else begin
            case ({push & (fifo_count != FIFO_DEPTH[FIFO_CW-1:0]),
                   tick & (fifo_count != 0)})
                2'b10: fifo_count <= fifo_count + 1'b1;
                2'b01: fifo_count <= fifo_count - 1'b1;
                default: ; // both or neither → no change
            endcase
        end
    end

    // ── DUT ────────────────────────────────────────────────────────
    wire signed [15:0] dbg_error;
    wire signed [31:0] dbg_inc_adj;
    wire        [31:0] dbg_inc_eff;
    wire               adjust;

    nco #(
        .FIFO_DEPTH (FIFO_DEPTH),
        .SETPOINT   (SETPOINT),
        .MCLK_HZ    (MCLK_HZ),
        .UPDATE_HZ  (UPDATE_HZ),
        .KP         (KP_TB),
        .KI         (KI_TB)
    ) dut (
        .clk        (clk),
        .rst        (rst),
        .enable     (1'b1),
        .inc_nominal_in(INC_NOMINAL),
        .fifo_count (fifo_count),
        .tick       (tick),
        .tick_x100  (tick_x100),
        .dbg_error  (dbg_error),
        .dbg_inc_adj(dbg_inc_adj),
        .dbg_inc_eff(dbg_inc_eff),
        .adjust     (adjust)
    );

    // ── Measure realised tick rate over each UPDATE window ────────
    // Count `tick` pulses between log rows; convert to Hz using
    // sim-time deltas so we don't have to re-derive from inc_eff.
    //
    // Note on quantization: with UPDATE_HZ = 1000, each window is 1 ms,
    // so we count ~44 ticks. Integer-tick rounding gives ±1 kHz of
    // measurement noise. To beat that down without slowing the log
    // rate, accumulate tick counts over MEAS_AVG windows (sliding) and
    // report the moving average.
    localparam integer MEAS_AVG = 64;   // ~64 ms moving window  → ~16 Hz quantization
    integer tick_cnt   = 0;
    real    last_log_t = 0.0;
    integer tick_hist [0:MEAS_AVG-1];
    real    time_hist [0:MEAS_AVG-1];
    integer hist_idx   = 0;
    integer hist_fill  = 0;
    integer hist_sum   = 0;
    real    hist_dt    = 0.0;

    always @(posedge clk) if (!rst && tick) tick_cnt = tick_cnt + 1;

    // ── Logging ────────────────────────────────────────────────────
    integer fh;
    integer logged;
    integer update_div;
    integer wait_cnt;
    real    now_ms;
    real    dt_s;
    real    meas_hz;

    initial begin
        update_div = MCLK_HZ / UPDATE_HZ;

        $display("");
        $display("nco_tb");
        $display("  MCLK_HZ          = %0d", MCLK_HZ);
        $display("  FS_NOMINAL_HZ    = %0d", FS_NOMINAL_HZ);
        $display("  INC_NOMINAL      = %0d", INC_NOMINAL);
        $display("  INPUT_HZ_ACTUAL  = %0.3f  (%+0.3f Hz, %+0.3f ppm)",
                 INPUT_HZ_ACTUAL,
                 INPUT_HZ_ACTUAL - FS_NOMINAL_HZ,
                 (INPUT_HZ_ACTUAL - FS_NOMINAL_HZ) * 1e6 / FS_NOMINAL_HZ);
        $display("  UPDATE_HZ        = %0d  (window = %0d cycles)",
                 UPDATE_HZ, update_div);
        $display("  N_UPDATES        = %0d  (~%0.2f ms sim)",
                 N_UPDATES, N_UPDATES * 1000.0 / UPDATE_HZ);
        $display("");

        fh = $fopen("nco_tb.csv", "w");
        if (fh == 0) begin
            $display("ERROR: cannot open nco_tb.csv");
            $finish;
        end
        $fdisplay(fh, "time_ms,fifo_count,err,pi_out,inc_eff,meas_tick_hz");

        // Reset
        repeat (32) @(posedge clk);
        rst = 1'b0;

        // Wait for the NCO to be primed (depth climbs to LOW_MARK).
        // With INPUT_HZ ≈ 44 kHz and a 54 MHz clk, that's <1 ms.
        wait_cnt = 0;
        while (fifo_count < SETPOINT/2 && wait_cnt < (MCLK_HZ/100)) begin
            @(posedge clk);
            wait_cnt = wait_cnt + 1;
        end

        last_log_t = $realtime;
        tick_cnt   = 0;

        for (logged = 0; logged < N_UPDATES; logged = logged + 1) begin
            // Wait one UPDATE window
            repeat (update_div) @(posedge clk);

            now_ms  = $realtime / 1.0e6;            // ns → ms
            dt_s    = ($realtime - last_log_t) / 1.0e9;

            // Slide tick_cnt / dt_s into the history ring and recompute
            // the moving sum. Plain O(N) rebuild — N=64 is trivial.
            tick_hist[hist_idx] = tick_cnt;
            time_hist[hist_idx] = dt_s;
            hist_idx = (hist_idx + 1) % MEAS_AVG;
            if (hist_fill < MEAS_AVG) hist_fill = hist_fill + 1;
            begin : avg_block
                integer i;
                hist_sum = 0;
                hist_dt  = 0.0;
                for (i = 0; i < hist_fill; i = i + 1) begin
                    hist_sum = hist_sum + tick_hist[i];
                    hist_dt  = hist_dt  + time_hist[i];
                end
            end
            meas_hz = (hist_dt > 0.0) ? (hist_sum / hist_dt) : 0.0;

            $fdisplay(fh,
                "%0.4f,%0d,%0d,%0d,%0d,%0.3f",
                now_ms,
                fifo_count,
                dbg_error,
                $signed(dbg_inc_adj),   // == integ; pi_out = integ+prop, but integ is the persistent state
                dbg_inc_eff,
                meas_hz);
            $fflush(fh);

            last_log_t = $realtime;
            tick_cnt   = 0;
        end

        $fclose(fh);
        $display("Wrote %0d rows to nco_tb.csv", N_UPDATES);
        $display("Final fifo_count=%0d, err=%0d, inc_eff=%0d (vs nominal %0d, delta=%+0d)",
                 fifo_count, $signed(dbg_error), dbg_inc_eff, INC_NOMINAL,
                 $signed({1'b0, dbg_inc_eff}) - $signed({1'b0, INC_NOMINAL}));
        $finish;
    end

    // Watchdog
    initial begin
        repeat (10) #1_000_000_000;   // 10 s sim time
        $display("FATAL: nco_tb watchdog expired");
        $finish;
    end

endmodule

`default_nettype wire
