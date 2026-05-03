`timescale 1ns / 1ps
`default_nettype none

// dac_idle_tone_tb
// ────────────────
// Focused idle-tone hunt. Drives a small DC value into dac.v alone
// (no ASRC, no I2S, no analog) and dumps the raw 1-bit modulator
// output to a binary file. Repeat for several DC levels and run
// scripts/analyze_idle_tones.py over the dumps to look for discrete
// spurs in the audio band.
//
// Idle tones in a 2nd-order ΔΣ modulator with small/rational DC
// inputs are well documented: the loop locks into a short limit
// cycle whose period puts a discrete tone in the audio band. They
// are particularly visible at:
//   din = 0
//   din = ±FS / N   for small integer N
// and are sensitive to the dither amplitude / placement (which is
// why we want to test this in sim with an idealised noise source —
// no analog confounders).
//
// File format
// ───────────
// Each output file is the raw bitstream: one byte per output cycle
// (0x00 or 0x01). We dump SAMPLES_PER_RUN cycles after a settling
// window. At 108 MHz mclk, 4 M samples = ~37 ms = plenty for 20 Hz
// FFT bin resolution after offline LP-filter and decimation.
//
// Run
// ───
//   xvlog ../../src/rtl/dac.v dac_idle_tone_tb.v
//   xelab -debug typical dac_idle_tone_tb -s idle_tone_sim
//   xsim idle_tone_sim -R
// Output files (in the working dir):
//   idle_din_+0000000.bin
//   idle_din_+0000001.bin
//   idle_din_+0000010.bin
//   ...

module dac_idle_tone_tb;

    localparam integer WORDLENGTH = 32;
    localparam integer ORDER      = 2;

    // 4 M cycles ≈ 37 ms at 108 MHz; tweak down for quicker turns.
    localparam integer SAMPLES_PER_RUN = 4_000_000;
    localparam integer SETTLE_CYCLES   = 100_000;

    // 108 MHz mclk
    localparam real T_CLK_NS = 9.259259;
    reg clk = 1'b0;
    always #(T_CLK_NS/2.0) clk = ~clk;

    reg rst = 1'b1;
    reg signed [WORDLENGTH-1:0] din    = 0;
    reg                          dvalid = 1'b1;

    // Two independent xorshift PRNGs for TPDF dither.
    reg [31:0] dither1 = 32'hCAFEBABE;
    reg [31:0] dither2 = 32'hDEADBEEF;
    reg [31:0] x1, x2;
    always @(posedge clk) begin
        x1       = dither1 ^ (dither1 << 13);
        x1       = x1      ^ (x1      >> 17);
        dither1 <= x1      ^ (x1      << 5);
        x2       = dither2 ^ (dither2 << 17);
        x2       = x2      ^ (x2      >> 7);
        dither2 <= x2      ^ (x2      << 11);
    end

    wire dout;

    dac #(
        .WORDLENGTH(WORDLENGTH),
        .ORDER     (ORDER)
    ) dut (
        .clk    (clk),
        .rst    (rst),
        .din    (din),
        .dvalid (dvalid),
        .dither1(dither1),
        .dither2(dither2),
        .dout   (dout)
    );

    // ── Dump machinery ────────────────────────────────────────────
    integer fd;
    integer dump_active = 0;
    integer dumped_cnt  = 0;
    reg [7:0] byte_buf;

    always @(posedge clk) begin
        if (dump_active && !rst) begin
            byte_buf = {7'd0, dout};
            $fwrite(fd, "%c", byte_buf);
            dumped_cnt <= dumped_cnt + 1;
        end
    end

    // ── Test sequence: sweep small DC values ──────────────────────
    // Values are in raw 32-bit signed counts (FS = 2^31 = 2_147_483_648).
    // We deliberately include 0 (the worst case for limit cycles), a few
    // small powers of two, and a few "rational FS/N" values that are
    // historically prone to idle tones.
    integer test_idx;
    integer total_tests;
    reg signed [31:0] test_levels [0:15];
    reg [255:0]       test_names  [0:15];   // text label per file

    task run_one(input integer idx);
        reg [255:0] fname;
        begin
            din    <= test_levels[idx];
            dvalid <= 1'b1;
            // settle
            dumped_cnt = 0;
            #(SETTLE_CYCLES * T_CLK_NS);
            // open file
            $sformat(fname, "idle_%0s.bin", test_names[idx]);
            fd = $fopen(fname, "wb");
            if (fd == 0) begin
                $display("ERROR: could not open %0s", fname);
                $finish;
            end
            $display("[%0d/%0d] din=%0d  ->  %0s",
                     idx + 1, total_tests, test_levels[idx], fname);
            dump_active = 1;
            // dump SAMPLES_PER_RUN cycles
            wait (dumped_cnt >= SAMPLES_PER_RUN);
            dump_active = 0;
            $fclose(fd);
        end
    endtask

    initial begin
        // build the table
        test_levels[ 0] =  32'sd0;          test_names[ 0] = "din_zero";
        test_levels[ 1] =  32'sd1;          test_names[ 1] = "din_plus1lsb";
        test_levels[ 2] = -32'sd1;          test_names[ 2] = "din_minus1lsb";
        test_levels[ 3] =  32'sh00010000;   test_names[ 3] = "din_plus_2pow16";
        test_levels[ 4] =  32'sh00100000;   test_names[ 4] = "din_plus_2pow20";
        test_levels[ 5] =  32'sh01000000;   test_names[ 5] = "din_plus_2pow24";
        // FS/1000  (current MODE_DC value in HW)
        test_levels[ 6] =  32'sh7FFFFFFF / 32'sd1000;  test_names[ 6] = "din_plus_FS_div1000";
        // FS/512  - a "round" rational bound to lock
        test_levels[ 7] =  32'sh7FFFFFFF / 32'sd512;   test_names[ 7] = "din_plus_FS_div512";
        // FS/3   - classic idle-tone trap
        test_levels[ 8] =  32'sh7FFFFFFF / 32'sd3;     test_names[ 8] = "din_plus_FS_div3";
        // FS/100
        test_levels[ 9] =  32'sh7FFFFFFF / 32'sd100;   test_names[ 9] = "din_plus_FS_div100";
        // negative versions of the more interesting ones
        test_levels[10] = -32'sh00100000;              test_names[10] = "din_minus_2pow20";
        test_levels[11] = -32'sh7FFFFFFF / 32'sd1000;  test_names[11] = "din_minus_FS_div1000";

        total_tests = 12;

        $display("dac_idle_tone_tb starting (ORDER=%0d, %0d tests, %0d samples each)",
                 ORDER, total_tests, SAMPLES_PER_RUN);

        // global reset
        #200 rst = 1'b0;
        #200;

        for (test_idx = 0; test_idx < total_tests; test_idx = test_idx + 1)
            run_one(test_idx);

        $display("dac_idle_tone_tb done");
        $finish;
    end

endmodule

`default_nettype wire
