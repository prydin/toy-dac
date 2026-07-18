`timescale 1ns / 1ps
`default_nettype none

// frac_servo_tb
// ─────────────
// Demonstrates frac_servo adapting to a slightly "off" input sample
// rate.
//
// Plant model
// ───────────
// Two 32-bit phase accumulators (producer and consumer) are stepped
// every mclk.  Each time an accumulator wraps past 2^32 one
// sample is produced or consumed, updating fifo_count.
//
//   producer step = STEP_NOM + STEP_OFFSET  (slightly fast input)
//   consumer step = DUT output `step`       (adjusted by servo)
//
// Parameters are scaled for fast simulation (1 MHz mclk, 3 Hz
// update rate) rather than realistic audio rates.
//
// Expected behaviour
// ──────────────────
// 1. With the servo disabled, fifo fills at ~5 samples / update tick.
// 2. Once enabled, frac_servo raises `step` above STEP_NOM,
//    increasing the consumer rate until the fill stabilises.
// 3. The KI term accumulates until the step adjustment fully cancels
//    the frequency offset; fifo_count settles back near SETPOINT.

module frac_servo_tb;

    // ── Parameters ────────────────────────────────────────────────
    localparam integer MCLK_HZ        = 1_000_000;
    localparam integer FIFO_DEPTH     = 128;
    localparam integer SETPOINT       = FIFO_DEPTH / 2;         // 64
    localparam integer KP             = 256;
    localparam integer KI             = 512;
    localparam integer ERR_DEADBAND   = 0;
    localparam integer STEP_SLEW      = 10;
    localparam integer UPDATE_HZ      = 3;
    localparam integer UPDATE_DIV     = MCLK_HZ / UPDATE_HZ;   // 333 333

    // Q0.32 nominal step → nominal consumer rate = MCLK * 2^31 / 2^32 = 500 kHz
    localparam [31:0] STEP_NOM    = 32'h80000000;
    // +65536 ≈ +30.5 ppm (producer is this much faster than nominal)
    localparam [31:0] STEP_OFFSET = 32'h00010000;
    localparam [31:0] PROD_STEP   = STEP_NOM + STEP_OFFSET;

    // Per-update FIFO drift at nominal (no servo):
    //   UPDATE_DIV * STEP_OFFSET / 2^32 ≈ 333333 * 65536 / 2^32 ≈ 5.09 samples
    localparam integer CLK_HALF = 500;   // ns, → 1 MHz clock

    // ── DUT wiring ────────────────────────────────────────────────
    localparam integer FIFO_CNT_W = $clog2(FIFO_DEPTH + 1);  // 8 bits

    reg  clk    = 1'b0;
    reg  rst    = 1'b1;
    reg  enable = 1'b0;
    reg  [31:0]            step_nominal_in = STEP_NOM;
    reg  [FIFO_CNT_W-1:0]  fifo_count      = SETPOINT[FIFO_CNT_W-1:0];

    wire [31:0]            step;
    wire signed [15:0]     dbg_error;
    wire signed [31:0]     dbg_step_adj;
    wire                   adjust;

    frac_servo #(
        .MCLK_HZ            (MCLK_HZ),
        .FIFO_DEPTH         (FIFO_DEPTH),
        .KP                 (KP),
        .KI                 (KI),
        .ERR_DEADBAND       (ERR_DEADBAND),
        .UPDATE_HZ          (UPDATE_HZ),
        .STEP_SLEW_PERIOD   (STEP_SLEW),
        .ADJUST_QUANTUM_LOG2(8)
    ) dut (
        .clk            (clk),
        .rst            (rst),
        .enable         (enable),
        .step_nominal_in(step_nominal_in),
        .fifo_count     (fifo_count),
        .step           (step),
        .dbg_error      (dbg_error),
        .dbg_step_adj   (dbg_step_adj),
        .adjust         (adjust)
    );

    always #(CLK_HALF) clk = ~clk;

    // ── Plant model ───────────────────────────────────────────────
    // Fractional-phase accumulators; a wrap-around counts as one
    // sample produced (input side) or consumed (output side).
    reg [31:0] prod_acc = 32'h0;
    reg [31:0] cons_acc = 32'h0;

    // Overflow detection: next value is less than the addend
    // (unsigned wrap-around) iff the addition crossed 2^32.
    wire prod_ovf = (prod_acc + PROD_STEP < prod_acc);
    wire cons_ovf = (cons_acc + step      < cons_acc);

    always @(posedge clk) begin
        prod_acc <= prod_acc + PROD_STEP;
        cons_acc <= cons_acc + step;

        // Update FIFO fill — only one sample changes per cycle since
        // simultaneous overflow in both directions is a no-op.
        if (prod_ovf && !cons_ovf) begin
            if (fifo_count < FIFO_DEPTH[FIFO_CNT_W-1:0])
                fifo_count <= fifo_count + 1'b1;
        end else if (!prod_ovf && cons_ovf) begin
            if (fifo_count > {FIFO_CNT_W{1'b0}})
                fifo_count <= fifo_count - 1'b1;
        end
    end

    // ── Per-update logging ────────────────────────────────────────
    localparam integer DIV_W = $clog2(UPDATE_DIV + 1);
    reg [DIV_W-1:0] log_cnt      = 0;
    integer         update_index = 0;

    wire log_tick = (log_cnt == UPDATE_DIV - 1);

    always @(posedge clk) begin
        if (rst) begin
            log_cnt <= 0;
        end else begin
            log_cnt <= log_tick ? {DIV_W{1'b0}} : log_cnt + 1'b1;
            if (log_tick && enable) begin
                $display("upd %3d | fifo %3d | err %4d | step_adj %9d | step 0x%08X",
                         update_index,
                         fifo_count,
                         dbg_error,
                         dbg_step_adj,
                         step);
                update_index <= update_index + 1;
            end
        end
    end

    // ── Checks ────────────────────────────────────────────────────
    integer errors = 0;

    task check;
        input         cond;
        input [255:0] msg;
        begin
            if (!cond) begin
                $display("FAIL @%0t: %0s", $time, msg);
                errors = errors + 1;
            end else
                $display("PASS: %0s", msg);
        end
    endtask

    // ── Stimulus ──────────────────────────────────────────────────
    initial begin
        $dumpfile("frac_servo_tb.vcd");
        $dumpvars(0, frac_servo_tb);

        // Reset
        repeat (8) @(posedge clk);
        rst    <= 1'b0;
        repeat (4) @(posedge clk);

        // While disabled, step must equal step_nominal.
        check(step === STEP_NOM, "disabled: step == step_nominal");

        // Enable servo — producer is already running faster so fifo
        // will start drifting upward immediately.
        enable <= 1'b1;

        // ── S1: drift detected ───────────────────────────────────
        // After 5 update ticks the FIFO should have risen noticeably
        // above SETPOINT (~5 samples/tick × 5 ticks ≈ +25).
        repeat (5 * UPDATE_DIV) @(posedge clk);
        check(fifo_count > SETPOINT[FIFO_CNT_W-1:0],
              "S1: fifo drifted above setpoint (producer faster than consumer)");
        check($signed(dbg_step_adj) > 0,
              "S1: servo raised step_adj above zero");

        // ── S2: convergence ──────────────────────────────────────
        // Allow up to 50 more update ticks for the PI loop to drive
        // fifo_count back to within ±3 of SETPOINT.
        begin : wait_converge
            integer i;
            for (i = 0; i < 50; i = i + 1) begin
                repeat (UPDATE_DIV) @(posedge clk);
                if ($signed(fifo_count) >= SETPOINT - 3 &&
                    $signed(fifo_count) <= SETPOINT + 3)
                begin
                    $display("Converged at update %0d, fifo_count=%0d",
                             5 + i + 1, fifo_count);
                    disable wait_converge;
                end
            end
        end

        check($signed(fifo_count) >= SETPOINT - 3 &&
              $signed(fifo_count) <= SETPOINT + 3,
              "S2: fifo settled within +/-3 of setpoint");
        check(step > STEP_NOM,
              "S2: step > step_nominal (offset absorbed by servo)");

        // ── S3: hold ─────────────────────────────────────────────
        // After convergence, allow a short ring-down then require the
        // fill to stay inside a wider band for 10 ticks.
        //
        // Why wider than S2?
        //   S2 triggers on first entry into +/-3. With PI + slew limiting,
        //   it is normal to continue past that point before settling.
        //   This check validates bounded, stable behaviour instead of
        //   forcing immediate no-overshoot lock.
        repeat (3 * UPDATE_DIV) @(posedge clk);
        begin : hold_check
            integer i;
            reg s3_ok;
            integer lo;
            integer hi;
            s3_ok = 1'b1;
            lo = fifo_count;
            hi = fifo_count;
            for (i = 0; i < 10; i = i + 1) begin
                repeat (UPDATE_DIV) @(posedge clk);
                if (fifo_count < lo) lo = fifo_count;
                if (fifo_count > hi) hi = fifo_count;
                if ($signed(fifo_count) < SETPOINT - 14 ||
                    $signed(fifo_count) > SETPOINT + 14)
                begin
                    $display("FAIL S3 at hold tick %0d: fifo_count=%0d",
                             i, fifo_count);
                    errors = errors + 1;
                    s3_ok = 1'b0;
                    disable hold_check;
                end
            end
            if (s3_ok)
                check(1'b1, "S3: fifo stayed within +/-14 of setpoint for 10 ticks");
            $display("S3 window: min=%0d max=%0d setpoint=%0d", lo, hi, SETPOINT);
        end

        repeat (10) @(posedge clk);

        if (errors == 0)
            $display("\n=== frac_servo_tb: ALL CHECKS PASSED ===");
        else
            $display("\n=== frac_servo_tb: %0d FAILURE(S) ===", errors);

        $finish;
    end

    // Global watchdog — 70 update ticks.
    // Use realtime math to avoid 32-bit integer overflow in the delay
    // expression (70*UPDATE_DIV*CLK_HALF*2 exceeds 2^32 for this TB).
    localparam realtime WATCHDOG_NS = 70.0 * UPDATE_DIV * (2.0 * CLK_HALF);
    initial begin
        #(WATCHDOG_NS);
        $display("FATAL: watchdog expired");
        $finish;
    end

endmodule

`default_nettype wire
