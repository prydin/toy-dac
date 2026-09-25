`timescale 1ns / 1ps
`default_nettype none

// Focused FIFO fullness fixture.
// The real ASRC exposes a 16-bit samples-available count, while the I2C
// register presents an 8-bit saturated value. This bench models producer-only
// fill and then drains across the 50% boundary.
module fifo_fullness_tb;

    localparam integer FIFO_DEPTH = 256;
    localparam integer CLK_NS = 10;

    reg clk = 1'b0;
    always #(CLK_NS / 2) clk = ~clk;

    reg rst = 1'b1;
    reg prst = 1'b1;
    reg [15:0] sample_count = 16'd0;
    wire [7:0] fullness = dut.fifo_fullness_mclk;

    wire i2c_scl;
    wire i2c_sda;
    wire i2c_reg3_wr;
    wire [7:0] i2c_reg3_wdata;

    // Pull the unused I2C bus high so the embedded slave remains idle.
    pullup(i2c_scl);
    pullup(i2c_sda);

    registers #(
        .SCLK_HZ_NOM(1_000_000)
    ) dut (
        .mclk             (clk),
        .rst              (rst),
        .sclk             (clk),
        .prst             (prst),
        .rate_locked      (1'b0),
        .rate_code        (2'd0),
        .asrc_enable      (1'b0),
        .asrc_samp_avail_l(sample_count),
        .asrc_samp_avail_r(16'd0),
        .dither_en        (1'b0),
        .mode             (2'd0),
        .bypass_interp_ctrl(1'b0),
        .output_mute_ctrl (1'b0),
        .input_mute_ctrl  (1'b0),
        .i2c_scl          (i2c_scl),
        .i2c_sda          (i2c_sda),
        .i2c_reg3_wr      (i2c_reg3_wr),
        .i2c_reg3_wdata   (i2c_reg3_wdata)
    );

    integer failures = 0;

    task set_count(input integer value);
        begin
            @(negedge clk);
            sample_count = value[15:0];
            repeat (3) @(posedge clk);
            #1;
            if (fullness !== ((value > 255) ? 8'hff : value[7:0])) begin
                $display("FAIL count=%0d fullness=0x%02x expected=0x%02x",
                         value, fullness,
                         ((value > 255) ? 8'hff : value[7:0]));
                failures = failures + 1;
            end else begin
                $display("PASS count=%0d fullness=0x%02x", value, fullness);
            end
        end
    endtask

    initial begin
        $display("=== fifo_fullness_tb ===");
        repeat (3) @(posedge clk);
        rst = 1'b0;
        prst = 1'b0;

        set_count(0);
        set_count(127);
        set_count(128);
        set_count(129);
        set_count(255);
        set_count(256);
        set_count(300);

        // Drain through the reported 50% boundary.
        set_count(130);
        set_count(129);
        set_count(128);
        set_count(127);
        set_count(126);

        if (failures == 0)
            $display("PASS: all FIFO fullness encoding checks passed");
        else
            $display("FAIL: %0d FIFO fullness checks failed", failures);
        $finish;
    end
endmodule

// Exercise the actual ASRC occupancy counter with no output strobes.
module fractional_fill_tb;

    localparam integer CLK_NS = 10;

    reg clk = 1'b0;
    always #(CLK_NS / 2) clk = ~clk;

    reg rst = 1'b1;
    reg enable = 1'b1;
    reg sample_valid = 1'b0;
    wire [15:0] samples_avail;

    fractional_asrc #(
        .COEFF_FILE("fpga/toy-dac/src/rtl/frac_asrc.mem")
    ) dut (
        .clk              (clk),
        .rst              (rst),
        .enable           (enable),
        .sample_in        (32'sd0),
        .sample_valid     (sample_valid),
        .out_strobe       (1'b0),
        .step             (32'd0),
        .data_out         (),
        .dvalid_out       (),
        .in_consumed      (),
        .dbg_phase_acc    (),
        .dbg_mac_cyc      (),
        .dbg_samples_avail(samples_avail)
    );

    task show_count(input integer expected, input [255:0] label);
        begin
            #1;
            if (samples_avail !== expected[15:0])
                $display("FAIL %0s: samples_avail=%0d expected=%0d",
                         label, samples_avail, expected);
            else
                $display("PASS %0s: samples_avail=%0d", label, samples_avail);
        end
    endtask

    initial begin
        $display("=== fractional_fill_tb ===");
        repeat (3) @(posedge clk);
        rst = 1'b0;

        // Fill the ring to the priming threshold. The ASRC then advances
        // consumed_cnt to leave the intended 128-sample starting level.
        sample_valid = 1'b1;
        repeat (256) @(posedge clk);
        sample_valid = 1'b0;
        @(posedge clk);
        show_count(128, "after priming");

        // Continue producing with no output consumption. The raw diagnostic
        // count is expected to rise beyond the physical ring depth; the I2C
        // presentation layer must saturate it rather than wrap its low byte.
        sample_valid = 1'b1;
        repeat (129) @(posedge clk);
        sample_valid = 1'b0;
        @(posedge clk);
        show_count(256, "after overfill");
        $finish;
    end
endmodule

`default_nettype wire
