`timescale 1ns / 1ps
`default_nettype none

// Quick smoke test for mono_ff.
//
// Covers:
//   1) Single trigger produces a pulse of ~DELAY cycles wide.
//   2) RESETTABLE=0 ignores a retrigger while q is high.
//   3) RESETTABLE=1 extends the pulse on a retrigger.
//   4) Async reset clears q immediately.
//   5) Module re-arms after q falls (last_d is not stuck high).

module mono_ff_quick_tb;

    localparam integer FCLK          = 100_000_000;        // 10 ns period (clean math)
    localparam integer CLK_PERIOD_NS = 1_000_000_000 / FCLK;
    localparam integer DELAY_NS      = 200;                // 20 cycles
    localparam integer EXP_CYC       = (FCLK / 1_000_000_000.0) * DELAY_NS; // 20

    reg  clk = 0;
    reg  rst = 1;
    reg  d_a = 0;   // non-retriggerable
    reg  d_b = 0;   // retriggerable
    wire q_a, q_b;

    always #(CLK_PERIOD_NS/2) clk = ~clk;

    mono_ff #(
        .FCLK(FCLK), .DELAY_NS(DELAY_NS), .RESETTABLE(1'b0)
    ) dut_a (.clk(clk), .rst(rst), .d(d_a), .q(q_a));

    mono_ff #(
        .FCLK(FCLK), .DELAY_NS(DELAY_NS), .RESETTABLE(1'b1)
    ) dut_b (.clk(clk), .rst(rst), .d(d_b), .q(q_b));

    integer errors = 0;
    task check(input cond, input [255:0] msg);
        begin
            if (!cond) begin
                $display("FAIL @%0t ns: %0s", $time, msg);
                errors = errors + 1;
            end else begin
                $display("PASS @%0t ns: %0s", $time, msg);
            end
        end
    endtask

    // Pulse a signal for one clock cycle.
    task pulse_a; begin @(posedge clk); d_a <= 1'b1; @(posedge clk); d_a <= 1'b0; end endtask
    task pulse_b; begin @(posedge clk); d_b <= 1'b1; @(posedge clk); d_b <= 1'b0; end endtask

    // Count cycles q stays high after a known rising edge.
    task automatic count_high(input which, output integer cycles);
        integer n;
        begin
            n = 0;
            // Wait until q goes high
            if (which == 0) wait (q_a == 1'b1); else wait (q_b == 1'b1);
            // Count cycles while q stays high
            while ((which == 0 ? q_a : q_b) === 1'b1) begin
                @(posedge clk);
                n = n + 1;
            end
            cycles = n;
        end
    endtask

    integer w;

    initial begin
        $dumpfile("mono_ff_quick_tb.vcd");
        $dumpvars(0, mono_ff_quick_tb);

        // Reset
        repeat (4) @(posedge clk);
        rst <= 0;
        @(posedge clk);
        check(q_a === 1'b0 && q_b === 1'b0, "S0: q low after reset");

        // ── S1: single trigger, both modes ──
        fork
            begin pulse_a; count_high(0, w);
                  check(w >= EXP_CYC - 2 && w <= EXP_CYC + 2,
                        "S1a: q_a width ~= EXP_CYC"); end
            begin pulse_b; count_high(1, w);
                  check(w >= EXP_CYC - 2 && w <= EXP_CYC + 2,
                        "S1b: q_b width ~= EXP_CYC"); end
        join

        repeat (10) @(posedge clk);

        // ── S2: retrigger ignored, RESETTABLE=0 ──
        begin : s2
            time t0, t1;
            @(posedge clk); d_a <= 1'b1;
            t0 = $time;
            @(posedge clk); d_a <= 1'b0;
            // Retrigger half-way through the pulse
            #(DELAY_NS/2);
            @(posedge clk); d_a <= 1'b1;
            @(posedge clk); d_a <= 1'b0;
            wait (q_a == 1'b0);
            t1 = $time;
            check((t1 - t0) <= DELAY_NS + 3*CLK_PERIOD_NS,
                  "S2: RESETTABLE=0 ignores retrigger");
        end

        repeat (10) @(posedge clk);

        // ── S3: retrigger extends pulse, RESETTABLE=1 ──
        begin : s3
            time t_second, t_fall;
            @(posedge clk); d_b <= 1'b1;
            @(posedge clk); d_b <= 1'b0;
            #(DELAY_NS/2);
            @(posedge clk); d_b <= 1'b1;
            t_second = $time;
            @(posedge clk); d_b <= 1'b0;
            wait (q_b == 1'b0);
            t_fall = $time;
            check((t_fall - t_second) >= DELAY_NS - 2*CLK_PERIOD_NS,
                  "S3: RESETTABLE=1 extends pulse");
        end

        repeat (10) @(posedge clk);

        // ── S4: async reset while busy ──
        pulse_a;
        wait (q_a == 1'b1);
        #(DELAY_NS/4);
        rst = 1'b1;
        #(2*CLK_PERIOD_NS);
        check(q_a === 1'b0, "S4: rst clears q_a immediately");
        rst = 1'b0;
        repeat (4) @(posedge clk);

        // ── S5: re-arms after pulse, both modes ──
        pulse_a;
        wait (q_a == 1'b1);
        wait (q_a == 1'b0);
        repeat (3) @(posedge clk);
        fork : s5a
            begin pulse_a; wait (q_a == 1'b1);
                  check(1'b1, "S5a: q_a re-triggers after idle");
                  disable s5a; end
            begin #(4*DELAY_NS);
                  check(1'b0, "S5a: q_a re-trigger TIMEOUT (last_d stuck?)");
                  disable s5a; end
        join
        wait (q_a == 1'b0);

        if (errors == 0)
            $display("\n=== mono_ff_quick_tb: ALL %0d CHECKS PASSED ===", 7);
        else
            $display("\n=== mono_ff_quick_tb: %0d FAILURE(S) ===", errors);
        $finish;
    end

    // Global watchdog
    initial begin
        #(200 * DELAY_NS);
        $display("FATAL: testbench watchdog expired");
        $finish;
    end

endmodule

`default_nettype wire
