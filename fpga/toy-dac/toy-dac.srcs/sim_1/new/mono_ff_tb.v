`timescale 1ns / 1ps

// Testbench for mono_ff.v
//
// Spec (as understood from the source):
//   - On a rising edge of d, q goes high and stays high for DELAY ns.
//   - While q is high, further rising edges of d are ignored unless
//     RESETTABLE=1, in which case each new rising edge restarts the
//     timer (q stays high, full DELAY from the new edge).
//   - rst is asynchronous active-high and clears q + counter.
//
// Scenarios:
//   1) Single trigger: q rises within 1 cycle of d's rising edge,
//      stays high for ~DELAY ns, then falls.
//   2) Re-trigger while busy, RESETTABLE=0: second pulse on d during
//      the busy window is ignored; q falls DELAY ns after the first
//      edge.
//   3) Re-trigger while busy, RESETTABLE=1: second pulse extends the
//      busy window; q falls DELAY ns after the second edge.
//   4) Back-to-back triggers after q has fallen (both modes): both
//      should fire.  This catches the latching-last_d bug.
//   5) Async reset while busy: q drops immediately, internal state
//      cleared, next d-edge triggers normally.

module mono_ff_tb;

    // Use a small DELAY so the simulation runs fast.
    localparam integer FCLK     = 54_000_000;
    localparam integer DELAY_NS = 1_000;                 // 1 µs
    localparam integer EXP_CYC  = (FCLK * DELAY_NS) / 1_000_000_000;  // = 54

    localparam integer CLK_PERIOD_NS = 1_000_000_000 / FCLK;          // 18 ns nominal

    // Two DUTs so we can exercise both modes side by side.
    reg clk = 0;
    reg rst = 0;
    reg d_a = 0;   // RESETTABLE = 0
    reg d_b = 0;   // RESETTABLE = 1
    wire q_a, q_b;

    always #(CLK_PERIOD_NS/2) clk = ~clk;

    mono_ff #(
        .FCLK(FCLK),
        .DELAY(DELAY_NS),
        .RESETTABLE(1'b0)
    ) dut_a (
        .clk(clk), .rst(rst), .d(d_a), .q(q_a)
    );

    mono_ff #(
        .FCLK(FCLK),
        .DELAY(DELAY_NS),
        .RESETTABLE(1'b1)
    ) dut_b (
        .clk(clk), .rst(rst), .d(d_b), .q(q_b)
    );

    // ── Bookkeeping ─────────────────────────────────────────────
    integer errors = 0;

    task check;
        input        cond;
        input [255:0] msg;
        begin
            if (!cond) begin
                $display("FAIL @%0t: %0s", $time, msg);
                errors = errors + 1;
            end else begin
                $display("PASS @%0t: %0s", $time, msg);
            end
        end
    endtask

    // Measure how long q stays high after we asked. Returns ns.
    task measure_pulse;
        input        which;       // 0 = q_a, 1 = q_b
        output integer width_ns;
        time t_lo, t_hi;
        begin
            // wait for q to rise
            if (which == 0) wait (q_a == 1'b1); else wait (q_b == 1'b1);
            t_hi = $time;
            // wait for q to fall
            if (which == 0) wait (q_a == 1'b0); else wait (q_b == 1'b0);
            t_lo = $time;
            width_ns = t_lo - t_hi;
        end
    endtask

    // Drive d high for a single clock cycle (one rising edge).
    task pulse_d_a;
        begin
            @(posedge clk); d_a <= 1'b1;
            @(posedge clk); d_a <= 1'b0;
        end
    endtask
    task pulse_d_b;
        begin
            @(posedge clk); d_b <= 1'b1;
            @(posedge clk); d_b <= 1'b0;
        end
    endtask

    integer w;

    initial begin
        $dumpfile("mono_ff_tb.vcd");
        $dumpvars(0, mono_ff_tb);

        // Reset
        rst = 1;
        repeat (4) @(posedge clk);
        rst <= 0;
        @(posedge clk);

        check(q_a === 1'b0, "S0: q_a low after reset");
        check(q_b === 1'b0, "S0: q_b low after reset");

        // ── Scenario 1: single trigger, both modes ──
        fork
            begin pulse_d_a; measure_pulse(0, w);
                  // Allow ±2 cycles tolerance on the measured width
                  check(w >= DELAY_NS - 2*CLK_PERIOD_NS &&
                        w <= DELAY_NS + 2*CLK_PERIOD_NS,
                        "S1: q_a width ~= DELAY"); end
            begin pulse_d_b; measure_pulse(1, w);
                  check(w >= DELAY_NS - 2*CLK_PERIOD_NS &&
                        w <= DELAY_NS + 2*CLK_PERIOD_NS,
                        "S1: q_b width ~= DELAY"); end
        join

        // Settle
        repeat (10) @(posedge clk);

        // ── Scenario 2: retrigger while busy, RESETTABLE=0 ──
        // Fire d_a, then fire it again at ~half the window.  q_a
        // should fall DELAY ns after the FIRST edge.
        begin : s2
            time t0, t1;
            @(posedge clk); d_a <= 1'b1;
            @(posedge clk); d_a <= 1'b0;
            t0 = $time;
            // Wait ~DELAY/2, pulse again
            #(DELAY_NS/2);
            @(posedge clk); d_a <= 1'b1;
            @(posedge clk); d_a <= 1'b0;
            // Now wait for q_a to fall
            wait (q_a == 1'b0);
            t1 = $time;
            // Expect t1 - t0 ≈ DELAY (NOT 1.5×DELAY)
            check((t1 - t0) <= DELAY_NS + 2*CLK_PERIOD_NS,
                  "S2: RESETTABLE=0 ignores retrigger");
        end

        repeat (10) @(posedge clk);

        // ── Scenario 3: retrigger while busy, RESETTABLE=1 ──
        // Same as S2 but expect q_b to fall ≈ DELAY ns after the
        // SECOND edge.
        begin : s3
            time t1edge, tfall;
            @(posedge clk); d_b <= 1'b1;
            @(posedge clk); d_b <= 1'b0;
            #(DELAY_NS/2);
            @(posedge clk); d_b <= 1'b1;
            t1edge = $time;
            @(posedge clk); d_b <= 1'b0;
            wait (q_b == 1'b0);
            tfall = $time;
            check((tfall - t1edge) >= DELAY_NS - 2*CLK_PERIOD_NS,
                  "S3: RESETTABLE=1 extends pulse on retrigger");
        end

        repeat (10) @(posedge clk);

        // ── Scenario 4: back-to-back after q has fallen ──
        // Both modes: trigger, wait for q low, trigger again, expect
        // a second pulse.  This exposes the last_d-latch bug.
        check(q_a === 1'b0, "S4 pre: q_a idle");
        pulse_d_a;
        wait (q_a == 1'b1);   // first pulse seen
        wait (q_a == 1'b0);   // first pulse done
        repeat (5) @(posedge clk);
        fork : s4_a_watchdog
            begin
                pulse_d_a;
                wait (q_a == 1'b1);
                check(1'b1, "S4: q_a re-triggers after idle");
                disable s4_a_watchdog;
            end
            begin
                #(4*DELAY_NS);
                check(1'b0, "S4: q_a re-trigger TIMEOUT (last_d latched bug?)");
                disable s4_a_watchdog;
            end
        join
        wait (q_a == 1'b0);

        repeat (10) @(posedge clk);

        check(q_b === 1'b0, "S4 pre: q_b idle");
        fork : s4_b_watchdog
            begin
                pulse_d_b;
                wait (q_b == 1'b1);
                check(1'b1, "S4: q_b re-triggers after idle");
                disable s4_b_watchdog;
            end
            begin
                #(4*DELAY_NS);
                check(1'b0, "S4: q_b re-trigger TIMEOUT");
                disable s4_b_watchdog;
            end
        join
        wait (q_b == 1'b0);

        repeat (10) @(posedge clk);

        // ── Scenario 5: async reset while busy ──
        pulse_d_a;
        wait (q_a == 1'b1);
        #(DELAY_NS/4);
        rst = 1;
        #(2*CLK_PERIOD_NS);
        check(q_a === 1'b0, "S5: rst clears q_a immediately");
        rst = 0;
        repeat (4) @(posedge clk);
        // Should be able to trigger again
        fork : s5_watchdog
            begin
                pulse_d_a;
                wait (q_a == 1'b1);
                check(1'b1, "S5: triggers normally after rst");
                disable s5_watchdog;
            end
            begin
                #(4*DELAY_NS);
                check(1'b0, "S5: post-rst trigger TIMEOUT");
                disable s5_watchdog;
            end
        join
        wait (q_a == 1'b0);

        repeat (20) @(posedge clk);

        if (errors == 0)
            $display("\n=== mono_ff_tb: ALL CHECKS PASSED ===");
        else
            $display("\n=== mono_ff_tb: %0d FAILURE(S) ===", errors);

        $finish;
    end

    // Global watchdog
    initial begin
        #(50 * DELAY_NS);
        $display("FATAL: testbench watchdog expired");
        $finish;
    end

endmodule
