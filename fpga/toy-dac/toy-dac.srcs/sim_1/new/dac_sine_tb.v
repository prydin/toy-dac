`timescale 1ns / 1ps
`default_nettype none

// dac_sine_tb
// ───────────
// Drive a 1 kHz sine into dac.v at hardware-realistic sample rate
// and amplitude, dump the raw bitstream for offline FFT.
//
// Sample rate model
//   Modulator clock                : 108 MHz
//   Hardware input rate to dac.v   : Fs_in = 108 MHz / 64 = 1.6875 MHz
//   So we pulse dvalid for 1 cycle every 64 mclks, matching HW.
//
// Sine amplitude
//   default -50 dBFS, 1 kHz
//   amp = 10^(-50/20) * 2^31 ≈ 6_793_001 counts
//
// File format identical to dac_idle_tone_tb: one byte per mclk cycle,
// 0x00 or 0x01. Output:
//   sine_1kHz_-50dBFS.bin

module dac_sine_tb;

    localparam integer WORDLENGTH = 32;
    localparam integer ORDER      = 2;

    // 8 M cycles ≈ 74 ms at 108 MHz → ~74 periods of 1 kHz.
    localparam integer SAMPLES_PER_RUN = 8_000_000;
    localparam integer SETTLE_CYCLES   = 200_000;

    // 108 MHz mclk
    localparam real T_CLK_NS = 9.259259;

    // Input sample rate divider
    localparam integer FS_DIV = 64;          // 108 MHz / 64 = 1.6875 MHz

    // Sine parameters
    localparam real PI         = 3.14159265358979;
    localparam real FS_IN_HZ   = 108.0e6 / FS_DIV;
    localparam real F_TONE_HZ  = 1000.0;
    // -10 dBFS  (10^(-10/20) ≈ 0.31623)
    localparam real AMP_DBFS   = -10.0;
    localparam real AMP_FRAC   = 0.31622777;  // = 10^(-10/20)
    // FS = 2^31 = 2_147_483_648
    localparam real AMP_COUNTS = AMP_FRAC * 2147483648.0;

    reg clk = 1'b0;
    always #(T_CLK_NS/2.0) clk = ~clk;

    reg rst = 1'b1;
    reg signed [WORDLENGTH-1:0] din    = 0;
    reg                          dvalid = 1'b0;

    // TPDF dither (same xorshift PRNGs as the other TBs)
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

    // ── Sine generator (updated once per Fs_in tick) ─────────────
    integer       div_cnt = 0;
    integer       sample_n = 0;
    real          phase, sample_real;
    integer       sample_int;

    always @(posedge clk) begin
        if (rst) begin
            div_cnt  <= 0;
            sample_n <= 0;
            dvalid   <= 1'b0;
        end else begin
            div_cnt <= div_cnt + 1;
            if (div_cnt == FS_DIV - 1) begin
                div_cnt    <= 0;
                phase       = 2.0 * PI * F_TONE_HZ * sample_n / FS_IN_HZ;
                sample_real = AMP_COUNTS * $sin(phase);
                sample_int  = $rtoi(sample_real);
                din        <= sample_int;
                dvalid     <= 1'b1;
                sample_n   <= sample_n + 1;
            end else begin
                dvalid <= 1'b0;
            end
        end
    end

    // ── Dump machinery ───────────────────────────────────────────
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

    initial begin
        $display("dac_sine_tb starting:");
        $display("  ORDER=%0d, mclk=108 MHz, Fs_in=%0.1f Hz, F=%0.1f Hz, A=%0.1f dBFS",
                 ORDER, FS_IN_HZ, F_TONE_HZ, AMP_DBFS);
        $display("  AMP_COUNTS = %0.0f", AMP_COUNTS);
        $display("  SAMPLES_PER_RUN = %0d  (%0.1f ms)",
                 SAMPLES_PER_RUN, SAMPLES_PER_RUN * T_CLK_NS / 1.0e6);

        #200 rst = 1'b0;
        #200;

        // settle
        dumped_cnt = 0;
        #(SETTLE_CYCLES * T_CLK_NS);

        fd = $fopen("sine_1kHz_-10dBFS.bin", "wb");
        if (fd == 0) begin
            $display("ERROR: could not open output file");
            $finish;
        end
        $display("dumping to sine_1kHz_-10dBFS.bin");
        dump_active = 1;
        wait (dumped_cnt >= SAMPLES_PER_RUN);
        dump_active = 0;
        $fclose(fd);

        $display("dac_sine_tb done");
        $finish;
    end

endmodule

`default_nettype wire
