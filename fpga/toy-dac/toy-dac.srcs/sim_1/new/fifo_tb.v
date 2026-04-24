`timescale 1ns / 1ps

// Testbench for fifo.v.
//
// Five scenarios, each run from a clean reset:
//   1) Balanced: writes and reads happen at the same rate.
//   2) Slow fill: writes a bit faster than reads, but stops before full.
//   3) Slow drain: reads a bit faster than writes, but stops before empty.
//   4) Overrun:   writes much faster than reads until `full` asserts and
//                 wr_en is held high — confirms the FIFO refuses to accept
//                 more data and `count` stays at DEPTH.
//   5) Underrun:  reads from an empty FIFO — confirms `empty` is asserted
//                 and `count` stays at 0.
//
// At the end the TB prints PASS / FAIL based on simple invariants:
//  - No data loss/duplication in scenarios 1..3 (read sequence matches
//    the head of the write sequence).
//  - In scenario 4, `full` was asserted and writes attempted while full
//    were dropped (count never exceeded DEPTH).
//  - In scenario 5, `empty` was asserted and read attempts produced no
//    forward progress in rd_ptr (count stayed at 0).

module fifo_tb;

    localparam WIDTH = 32;
    localparam DEPTH = 16;          // small for fast sim & easy to overrun
    localparam CLK_PERIOD_NS = 10;  // 100 MHz

    reg clk = 0;
    reg rst = 1;

    reg                     wr_en   = 0;
    reg  signed [WIDTH-1:0] wr_data = 0;
    wire                    full;

    reg                     rd_en = 0;
    wire signed [WIDTH-1:0] rd_data;
    wire                    empty;

    wire [$clog2(DEPTH+1)-1:0] count;

    fifo #(
        .WIDTH(WIDTH),
        .DEPTH(DEPTH)
    ) uut (
        .clk(clk),
        .rst(rst),
        .wr_data(wr_data),
        .wr_en(wr_en),
        .full(full),
        .rd_data(rd_data),
        .rd_en(rd_en),
        .empty(empty),
        .count(count)
    );

    // Clock
    always #(CLK_PERIOD_NS / 2) clk = ~clk;

    // ── Scoreboard ────────────────────────────────────────────────────
    // Independent expectation queue: every accepted write pushes its
    // value, every accepted read pops the head and compares to rd_data.
    integer expect_q [0:1023];
    integer expect_head = 0;
    integer expect_tail = 0;
    integer mismatches  = 0;

    task push_expected(input integer v);
        begin
            expect_q[expect_tail] = v;
            expect_tail = expect_tail + 1;
        end
    endtask

    // Sample read data on the same cycle as rd_en (FIFO is read-first /
    // combinational rd_data = mem[rd_addr]).
    always @(posedge clk) begin
        if (!rst && rd_en && !empty) begin
            if (rd_data !== expect_q[expect_head]) begin
                $display("t=%0t  MISMATCH  got=%0d  expected=%0d  (count_before=%0d)",
                         $time, rd_data, expect_q[expect_head], count);
                mismatches = mismatches + 1;
            end
            expect_head = expect_head + 1;
        end
    end

    // ── Helpers ───────────────────────────────────────────────────────
    integer next_data = 1;

    task do_reset;
        begin
            wr_en = 0; rd_en = 0; wr_data = 0;
            rst = 1;
            repeat (4) @(posedge clk);
            @(negedge clk); rst = 0;
            @(posedge clk);
            expect_head = 0;
            expect_tail = 0;
            mismatches  = 0;
            next_data   = 1;
        end
    endtask

    // Run `cycles` clk cycles; on each cycle assert wr_en with prob
    // wr_rate/100 and rd_en with prob rd_rate/100. Uses $random for
    // pseudo-random pacing so the rates aren't synchronised.
    //
    // wr_en / rd_en are NOT gated by full / empty here — we want the
    // FIFO itself to reject excess attempts so the scenario monitors
    // can count dropped writes and no-op reads. The scoreboard only
    // pushes the expected value when the write will actually land
    // (wr_en && !full); same for reads (sampled by the always block
    // that compares rd_data, gated by !empty).
    task run_traffic(input integer cycles,
                     input integer wr_rate,
                     input integer rd_rate);
        integer i;
        integer r;
        begin
            for (i = 0; i < cycles; i = i + 1) begin
                @(negedge clk);
                r = {$random} % 100;
                wr_en = (r < wr_rate);
                if (wr_en) begin
                    wr_data = next_data;
                    if (!full) begin
                        push_expected(next_data);
                        next_data = next_data + 1;
                    end
                end else begin
                    wr_data = 0;
                end
                r = {$random} % 100;
                rd_en = (r < rd_rate);
            end
            @(negedge clk);
            wr_en = 0;
            rd_en = 0;
        end
    endtask

    // ── Scenario monitors ─────────────────────────────────────────────
    integer max_count_seen;
    integer full_seen;
    integer empty_seen;
    integer dropped_writes;
    integer noop_reads;

    always @(posedge clk) begin
        if (!rst) begin
            if (count > max_count_seen) max_count_seen = count;
            if (full)  full_seen  = full_seen  + 1;
            if (empty) empty_seen = empty_seen + 1;
            // wr_en held high while full → write would be dropped
            if (wr_en && full)  dropped_writes = dropped_writes + 1;
            // rd_en held high while empty → read would be a no-op
            if (rd_en && empty) noop_reads     = noop_reads     + 1;
        end
    end

    task reset_monitors;
        begin
            max_count_seen = 0;
            full_seen      = 0;
            empty_seen     = 0;
            dropped_writes = 0;
            noop_reads     = 0;
        end
    endtask

    // ── Main ──────────────────────────────────────────────────────────
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
        $display("=== fifo_tb (DEPTH=%0d, WIDTH=%0d) ===", DEPTH, WIDTH);

        // -----------------------------------------------------------------
        // 1) Balanced: equal write and read rates, never fills, never empties for long.
        // -----------------------------------------------------------------
        $display("\n--- Scenario 1: balanced rates (50%% / 50%%) ---");
        do_reset; reset_monitors;
        run_traffic(2000, 50, 50);
        $display("  final count=%0d  max_count=%0d  full_cyc=%0d  empty_cyc=%0d  mismatches=%0d",
                 count, max_count_seen, full_seen, empty_seen, mismatches);
        check("no data mismatches",                     mismatches == 0);
        check("never went full",                        full_seen  == 0);
        check("max count well below DEPTH",             max_count_seen < DEPTH);

        // -----------------------------------------------------------------
        // 2) Slow fill: writes faster than reads, but stops before full.
        // -----------------------------------------------------------------
        $display("\n--- Scenario 2: writes 60%%, reads 40%%, short run ---");
        do_reset; reset_monitors;
        // Pick cycle count so expected fill ≈ 0.2 * cycles stays < DEPTH
        run_traffic(60, 60, 40);
        $display("  final count=%0d  max_count=%0d  full_cyc=%0d  empty_cyc=%0d  mismatches=%0d",
                 count, max_count_seen, full_seen, empty_seen, mismatches);
        check("no data mismatches",                     mismatches == 0);
        check("never reached full",                     full_seen  == 0);
        check("count grew above zero",                  max_count_seen > 0);

        // -----------------------------------------------------------------
        // 3) Slow drain: reads faster than writes, but stops before empty.
        // -----------------------------------------------------------------
        $display("\n--- Scenario 3: pre-fill then writes 40%%, reads 60%% ---");
        do_reset; reset_monitors;
        // Pre-fill the FIFO halfway so the drain phase has something to do.
        run_traffic(40, 100, 0);   // fill rapidly to ~DEPTH (will cap at full)
        // Now drain with reads slightly faster than writes, but not long
        // enough to fully empty.
        reset_monitors;
        run_traffic(40, 40, 60);
        $display("  final count=%0d  max_count=%0d  full_cyc=%0d  empty_cyc=%0d  mismatches=%0d",
                 count, max_count_seen, full_seen, empty_seen, mismatches);
        check("no data mismatches",                     mismatches == 0);
        check("never went empty during drain phase",    empty_seen == 0);
        check("count decreased (final < max during drain)", count < max_count_seen || count == 0);

        // -----------------------------------------------------------------
        // 4) Overrun: hold wr_en high, no reads. Confirm full asserts and
        //    excess writes are dropped (count caps at DEPTH).
        // -----------------------------------------------------------------
        $display("\n--- Scenario 4: overrun (writes 100%%, reads 0%%) ---");
        do_reset; reset_monitors;
        run_traffic(DEPTH * 4, 100, 0);
        $display("  final count=%0d  max_count=%0d  full_cyc=%0d  dropped_writes=%0d",
                 count, max_count_seen, full_seen, dropped_writes);
        check("full was asserted",                      full_seen  > 0);
        check("max_count never exceeded DEPTH",         max_count_seen <= DEPTH);
        check("max_count reached DEPTH",                max_count_seen == DEPTH);
        check("at least one write was dropped",         dropped_writes > 0);
        check("no data mismatches on the writes that were accepted",
                                                        mismatches == 0);

        // -----------------------------------------------------------------
        // 5) Underrun: read from empty FIFO. Confirm empty asserts and rd_ptr
        //    doesn't advance (count stays 0, no false data appears).
        // -----------------------------------------------------------------
        $display("\n--- Scenario 5: underrun (writes 0%%, reads 100%%) ---");
        do_reset; reset_monitors;
        run_traffic(50, 0, 100);
        $display("  final count=%0d  empty_cyc=%0d  noop_reads=%0d  mismatches=%0d",
                 count, empty_seen, noop_reads, mismatches);
        check("empty was asserted",                     empty_seen > 0);
        check("count stayed at 0",                      max_count_seen == 0);
        check("at least one read no-op (rd_en && empty)", noop_reads > 0);
        check("no spurious data delivered",             mismatches == 0);

        // -----------------------------------------------------------------
        $display("\n=== Summary: %0s (%0d fails) ===",
                 (fails == 0) ? "ALL PASS" : "FAILURES", fails);
        $finish;
    end

endmodule
