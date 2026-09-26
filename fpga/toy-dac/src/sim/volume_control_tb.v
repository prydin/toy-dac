`timescale 1ns / 1ps
`default_nettype none

module volume_control_tb;
    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg rst = 1'b1;
    reg [7:0] target_volume = 8'd0;
    reg target_toggle = 1'b0;
    reg signed [31:0] in_left = 32'sd0;
    reg signed [31:0] in_right = 32'sd0;
    reg in_valid_left = 1'b0;
    reg in_valid_right = 1'b0;
    wire signed [31:0] out_left;
    wire signed [31:0] out_right;
    wire out_valid_left;
    wire out_valid_right;

    volume_control #(
        .RAMP_CYCLES(2),
        .GAIN_FILE("../rtl/volume_gain_q23.mem")
    ) dut (
        .clk(clk),
        .rst(rst),
        .target_volume_async(target_volume),
        .target_update_toggle_async(target_toggle),
        .in_left(in_left),
        .in_right(in_right),
        .in_valid_left(in_valid_left),
        .in_valid_right(in_valid_right),
        .out_left(out_left),
        .out_right(out_right),
        .out_valid_left(out_valid_left),
        .out_valid_right(out_valid_right)
    );

    task set_volume(input [7:0] value);
        begin
            @(negedge clk);
            target_volume = value;
            target_toggle = ~target_toggle;
        end
    endtask

    task send_stereo(input signed [31:0] left, input signed [31:0] right);
        begin
            @(negedge clk);
            in_left = left;
            in_right = right;
            in_valid_left = 1'b1;
            in_valid_right = 1'b1;
            @(negedge clk);
            in_valid_left = 1'b0;
            in_valid_right = 1'b0;
            #1;
            if (!out_valid_left || !out_valid_right) begin
                $display("FAIL: output valid timing");
                $finish;
            end
        end
    endtask

    initial begin
        repeat (3) @(posedge clk);
        rst = 1'b0;

        send_stereo(32'sh4000_0000, -32'sh4000_0000);
        if ((out_left !== 32'sh4000_0000) ||
            (out_right !== -32'sh4000_0000)) begin
            $display("FAIL: unity gain left=%0d right=%0d", out_left, out_right);
            $finish;
        end

        set_volume(8'd12);
        repeat (32) @(posedge clk);
        send_stereo(32'sh4000_0000, -32'sh4000_0000);
        if ((out_left < 32'sd537_000_000) || (out_left > 32'sd539_000_000) ||
            (out_right > -32'sd537_000_000) || (out_right < -32'sd539_000_000)) begin
            $display("FAIL: -6 dB gain left=%0d right=%0d", out_left, out_right);
            $finish;
        end

        set_volume(8'hFF);
        repeat (520) @(posedge clk);
        send_stereo(32'sh7FFF_FFFF, -32'sh8000_0000);
        if ((out_left !== 32'sd0) || (out_right !== 32'sd0)) begin
            $display("FAIL: mute left=%0d right=%0d", out_left, out_right);
            $finish;
        end

        $display("PASS: volume_control_tb");
        $finish;
    end
endmodule

`default_nettype wire