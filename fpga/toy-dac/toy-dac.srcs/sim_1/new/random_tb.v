`timescale 1ns / 1ps

module random_tb;

    parameter WORDLENGTH = 32;
    parameter CLK_PERIOD = 10; // 100 MHz

    reg clk;
    reg rst;
    wire signed [WORDLENGTH-1:0] dout;

    random #(
        .WORDLENGTH(WORDLENGTH)
    ) uut (
        .clk(clk),
        .rst(rst),
        .dout(dout)
    );

    // Clock generation
    initial clk = 0;
    always #(CLK_PERIOD / 2) clk = ~clk;

    // Stimulus
    initial begin
        rst = 1;
        #(CLK_PERIOD * 5);
        rst = 0;

        // Let the PRNG run for a while and observe output
        #(CLK_PERIOD * 200);

        $display("Simulation complete.");
        $finish;
    end

    // Monitor output
    initial begin
        $monitor("t=%0t rst=%b dout=%0d (0x%08h)", $time, rst, dout, dout);
    end

endmodule
