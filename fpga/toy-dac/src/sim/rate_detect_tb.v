`timescale 1ns / 1ps

// Testbench for rate_detect.
// Drives lrclk_pos_edge as one-cycle pulses at three different rates,
// each rate held long enough for several windows to complete, and prints
// the period / window_period outputs whenever rate_valid pulses high.

module rate_detect_tb;

    // Use a small window so a window completes after only 16 lrclk edges,
    // keeping simulation time short while still exercising the same logic.
    localparam WINDOW_SIZE = 16;

    localparam CLK_PERIOD_NS = 10; // 100 MHz system clock

    reg clk = 0;
    reg rst = 1;
    reg lrclk_pos_edge = 0;
    reg dvalid = 0;

    wire        rate_valid;
    wire [31:0] window_period;
    wire [15:0] period;

    rate_detect #(
        .WINDOW_SIZE(WINDOW_SIZE)
    ) uut (
        .clk(clk),
        .rst(rst),
        .lrclk_pos_edge(lrclk_pos_edge),
        .dvalid(dvalid),
        .rate_valid(rate_valid),
        .window_period(window_period),
        .period(period)
    );

    // Clock
    always #(CLK_PERIOD_NS / 2) clk = ~clk;

    // Print whenever the rate output is asserted, plus context.
    always @(posedge clk) begin
        if (rate_valid) begin
            $display("t=%0t ns  rate_valid=1  window_period=%0d clks  period=%0d (period/WINDOW_SIZE=%0d clks/lrclk)",
                     $time, window_period, period, period / WINDOW_SIZE);
        end
    end

    // Drive `count` one-cycle pulses on lrclk_pos_edge, spaced
    // `cycles_between` clk cycles apart (edge-to-edge period).
    task drive_lrclk_burst(input integer count, input integer cycles_between);
        integer i;
        begin
            $display("t=%0t ns  --- driving %0d lrclk pulses, period = %0d clk cycles ---",
                     $time, count, cycles_between);
            for (i = 0; i < count; i = i + 1) begin
                // One-cycle pulse, aligned to clk
                @(posedge clk);
                lrclk_pos_edge <= 1'b1;
                @(posedge clk);
                lrclk_pos_edge <= 1'b0;
                // Hold low for the remainder of the requested period.
                // Subtract the 1 cycle the pulse already took.
                repeat (cycles_between - 1) @(posedge clk);
            end
        end
    endtask

    initial begin
        // Reset
        lrclk_pos_edge = 0;
        dvalid         = 0;
        rst            = 1;
        repeat (5) @(posedge clk);
        rst <= 0;
        @(posedge clk);

        // Three rates, each held for several full windows so we get
        // multiple rate_valid pulses per rate. With WINDOW_SIZE=16 we
        // need at least 16 lrclk pulses per window; use 3 windows = 48
        // pulses per rate.

        // ~390 kHz lrclk equivalent (clk/256)
        drive_lrclk_burst(48, 256);

        // ~195 kHz lrclk equivalent (clk/512)
        drive_lrclk_burst(48, 512);

        // ~100 kHz lrclk equivalent (clk/1000)
        drive_lrclk_burst(48, 1000);

        // Drain so any final outputs print before $finish
        repeat (20000) @(posedge clk);

        $display("t=%0t ns  Simulation complete.", $time);
        $finish;
    end

endmodule
