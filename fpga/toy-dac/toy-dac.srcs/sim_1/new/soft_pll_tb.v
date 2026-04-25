`timescale 1ns / 1ps

// Testbench for soft_pll.v.
//
// We override MCLK_HZ to a tiny value so UPDATE_DIV is small and a few
// update windows fit in a reasonable simulation. The control behaviour
// is independent of the absolute time scaling.
//
// Cases:
//   1) fifo_count == SETPOINT          → no psen pulses
//   2) fifo_count >  SETPOINT (full)   → pulses in one direction
//   3) fifo_count <  SETPOINT (empty)  → pulses in the opposite direction
//   4) psdone gating: psen never re-asserts until psdone has pulsed
//
// A simple psdone model: 5 mclk cycles after each psen pulse.

module soft_pll_tb;

    localparam integer FIFO_DEPTH = 64;
    localparam integer SETPOINT   = 32;
    // Tiny "MCLK_HZ" so UPDATE_DIV = 1000 cycles. Each update window
    // is then 1000 cycles, easy to simulate.
    localparam integer MCLK_HZ    = 1000;
    localparam integer UPDATE_HZ  = 1;
    localparam integer KP         = 4;
    localparam integer MAX_PULSES = 200;

    localparam CLK_PERIOD_NS = 10;

    reg clk = 0;
    reg rst = 1;
    reg [$clog2(FIFO_DEPTH+1)-1:0] fifo_count = 0;

    wire psen;
    wire psincdec;
    reg  psdone = 0;

    wire signed [15:0] dbg_error;
    wire signed [15:0] dbg_pulses_remaining;

    soft_pll #(
        .FIFO_DEPTH (FIFO_DEPTH),
        .SETPOINT   (SETPOINT),
        .MCLK_HZ    (MCLK_HZ),
        .UPDATE_HZ  (UPDATE_HZ),
        .KP         (KP),
        .MAX_PULSES (MAX_PULSES),
        .INVERT_DIR (1'b0)
    ) uut (
        .mclk                 (clk),
        .rst                  (rst),
        .fifo_count           (fifo_count),
        .psen                 (psen),
        .psincdec             (psincdec),
        .psdone               (psdone),
        .dbg_error            (dbg_error),
        .dbg_pulses_remaining (dbg_pulses_remaining)
    );

    // Clock
    always #(CLK_PERIOD_NS/2) clk = ~clk;

    // ── psdone model: pulse psdone 5 cycles after each psen ──
    integer psdone_delay = 0;
    always @(posedge clk) begin
        psdone <= 1'b0;
        if (psen)
            psdone_delay <= 5;
        else if (psdone_delay > 0) begin
            psdone_delay <= psdone_delay - 1;
            if (psdone_delay == 1)
                psdone <= 1'b1;
        end
    end

    // ── Counters per scenario ──
    integer psen_pulses_inc;   // psincdec=1
    integer psen_pulses_dec;   // psincdec=0
    integer psen_while_busy;   // psen asserted while psdone hasn't completed
    integer outstanding;        // 1 if waiting for psdone

    always @(posedge clk) begin
        if (rst) begin
            outstanding <= 0;
        end else begin
            if (psen) begin
                if (outstanding) psen_while_busy = psen_while_busy + 1;
                outstanding <= 1;
                if (psincdec) psen_pulses_inc = psen_pulses_inc + 1;
                else          psen_pulses_dec = psen_pulses_dec + 1;
            end
            if (psdone)
                outstanding <= 0;
        end
    end

    task reset_counters;
        begin
            psen_pulses_inc = 0;
            psen_pulses_dec = 0;
            psen_while_busy = 0;
        end
    endtask

    task do_reset;
        begin
            rst = 1;
            fifo_count = SETPOINT;
            repeat (4) @(posedge clk);
            @(negedge clk); rst = 0;
        end
    endtask

    // Run for `n_windows` update windows
    task run_windows(input integer n_windows);
        begin
            // Each window is UPDATE_DIV = MCLK_HZ/UPDATE_HZ cycles.
            repeat (n_windows * (MCLK_HZ / UPDATE_HZ) + 50) @(posedge clk);
        end
    endtask

    integer fails = 0;
    task check(input [255:0] name, input cond);
        begin
            if (cond)
                $display("  PASS: %0s", name);
            else begin
                $display("  FAIL: %0s", name);
                fails = fails + 1;
            end
        end
    endtask

    initial begin
        $display("=== soft_pll_tb ===");

        // ─── 1) At setpoint: no pulses ──────────────────────────────
        $display("\n--- Scenario 1: fifo_count == SETPOINT (%0d) ---", SETPOINT);
        do_reset; reset_counters;
        fifo_count = SETPOINT;
        run_windows(3);
        $display("  inc=%0d dec=%0d busy_violations=%0d dbg_error=%0d",
                 psen_pulses_inc, psen_pulses_dec, psen_while_busy, dbg_error);
        check("no INC pulses",                    psen_pulses_inc == 0);
        check("no DEC pulses",                    psen_pulses_dec == 0);
        check("dbg_error == 0",                   dbg_error == 0);

        // ─── 2) Above setpoint: speed up (one direction) ───────────
        $display("\n--- Scenario 2: fifo_count > SETPOINT (overfull, want speed-up) ---");
        do_reset; reset_counters;
        fifo_count = SETPOINT + 8;     // error = +8, KP*err = +32 pulses/window
        run_windows(3);
        $display("  inc=%0d dec=%0d busy_violations=%0d dbg_error=%0d",
                 psen_pulses_inc, psen_pulses_dec, psen_while_busy, dbg_error);
        check("got pulses",                       (psen_pulses_inc + psen_pulses_dec) > 0);
        check("only one direction",               (psen_pulses_inc == 0) ^ (psen_pulses_dec == 0));
        check("dbg_error == +8",                  dbg_error == 8);
        check("no psen while psdone outstanding", psen_while_busy == 0);

        // ─── 3) Below setpoint: opposite direction ─────────────────
        $display("\n--- Scenario 3: fifo_count < SETPOINT (underfull, want slow-down) ---");
        do_reset; reset_counters;
        fifo_count = SETPOINT - 8;
        run_windows(3);
        $display("  inc=%0d dec=%0d busy_violations=%0d dbg_error=%0d",
                 psen_pulses_inc, psen_pulses_dec, psen_while_busy, dbg_error);
        check("got pulses",                       (psen_pulses_inc + psen_pulses_dec) > 0);
        check("only one direction",               (psen_pulses_inc == 0) ^ (psen_pulses_dec == 0));
        check("dbg_error == -8",                  dbg_error == -8);
        check("no psen while psdone outstanding", psen_while_busy == 0);

        // ─── 4) Big error: clamped to MAX_PULSES ───────────────────
        $display("\n--- Scenario 4: very large error → clamped at MAX_PULSES (%0d) ---", MAX_PULSES);
        do_reset; reset_counters;
        fifo_count = FIFO_DEPTH;        // max possible
        run_windows(2);
        $display("  inc=%0d dec=%0d total=%0d (per window)",
                 psen_pulses_inc, psen_pulses_dec,
                 (psen_pulses_inc + psen_pulses_dec) / 2);
        // Note: pulse engine is rate-limited by psdone latency, so the
        // count we see in 2 windows is min(MAX_PULSES, achievable) per
        // window. We just check it's nonzero and within the cap.
        check("pulses delivered",                 (psen_pulses_inc + psen_pulses_dec) > 0);
        check("per-window count <= MAX_PULSES",   ((psen_pulses_inc + psen_pulses_dec) / 2) <= MAX_PULSES);

        $display("\n=== Summary: %0s (%0d fails) ===",
                 (fails == 0) ? "ALL PASS" : "FAILURES", fails);
        $finish;
    end

endmodule
