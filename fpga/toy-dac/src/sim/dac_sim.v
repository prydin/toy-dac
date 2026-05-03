`timescale 1ns / 1ps

module i2s_sender (
    input wire clk,
    input wire rst,
    input wire signed [15:0] sample, // Mono audio sample to send
    output reg bclk,
    output reg lrclk,
    output wire din
);

// Generate a bclk at 1/16 of clk (toggle every 16 clocks → period = 32 clocks)
reg [3:0] bclk_div;
always @(posedge clk or posedge rst) begin
    if (rst) begin
        bclk_div <= 0;
        bclk <= 0;
    end else begin
        bclk_div <= bclk_div + 1;
        if (bclk_div == 0) begin
            bclk <= ~bclk;
        end
    end 
end

// Generate lrclk: toggle every 32 bclk falling edges → 32 bits per channel
reg [4:0] lrclk_div;
reg prev_bclk;
always @(posedge clk or posedge rst) begin
    if (rst) begin
        lrclk_div <= 0;
        lrclk <= 0;
        prev_bclk <= 0;
    end else if(prev_bclk && ~bclk) begin // On bclk falling edge
        lrclk_div <= lrclk_div + 1;
        if (lrclk_div == 0) begin
            lrclk <= ~lrclk;
        end
    end 
    prev_bclk <= bclk;
end

// Shift out I2S serial data with standard one-bit delay.
// Sample-and-hold: capture `sample` once per frame (on lrclk rising edge)
// and send the same value to both L and R channels.
reg [31:0] shift;
reg bclk_prev;
reg lrclk_prev;
reg signed [15:0] held_sample;

assign din = shift[31]; // Output the MSB of the shift register

always @(posedge clk or posedge rst) begin
    if (rst) begin
        bclk_prev <= 0;
        lrclk_prev <= 0;
        shift <= 0;
        held_sample <= 0;
    end else begin
        bclk_prev <= bclk;
        lrclk_prev <= lrclk;
        if (~bclk & bclk_prev) begin // Shift out data on bclk falling edge
            shift <= {shift[30:0], 1'b0};
        end
        if (lrclk != lrclk_prev) begin // Load on any lrclk edge (wins over shift)
            if (~lrclk & lrclk_prev)        // Falling edge: start of left channel → capture new sample
                held_sample <= sample;
            shift <= {1'b0, held_sample, 15'b0}; // One-bit delay: MSB at bit 30
        end
    end
end

endmodule

module dac_sim(
    );


parameter WORDLENGTH = 32;

// Genrate a clock signal
reg aclk;
reg rst;
initial begin
    aclk = 0;
    #20
    rst = 1;
    #20 rst = 0; // Release reset after 20ns
    forever #5 aclk = ~aclk; // 100MHz clock
end

// Geenrate a test signal
wire signed [31:0] m_axis_data_tdata;
wire m_axis_data_tvalid;

dds_compiler_0 test_signal (
  .aclk(aclk),                                // input wire aclk
  .m_axis_data_tvalid(m_axis_data_tvalid),    // output wire m_axis_data_tvalid
  .m_axis_data_tdata(m_axis_data_tdata)       // output wire [15 : 0] m_axis_data_tdata
);

wire signed [31:0] dither1;
wire signed [31:0] dither2;

random_gen dither_gen1 (
    .clk(aclk),
    .rst(rst),
    .random_out(dither1)
);

random_gen dither_gen2 (
    .clk(aclk),
    .rst(rst),
    .random_out(dither2)
);

wire bclk;
wire lrclk;
wire din;

i2s_sender i2s_gen (
    .clk(aclk),
    .sample(m_axis_data_tvalid ? m_axis_data_tdata[15:0] : 16'h0), // Send test signal as sample
    .rst(rst),
    .bclk(bclk),
    .lrclk(lrclk),
    .din(din)
);

top top_inst (
    .clk(aclk),
    .bclk(bclk),
    .lrclk(lrclk),
    .din(din),
    .dither1(dither1),
    .dither2(dither2)
);

/*

i2s i2s_inst (
    .clk(aclk),
    .rst(rst),
    .bclk(bclk),
    .lrclk(lrclk),
    .din(din),
    .out_left(out_left),
    .out_right(out_right),
    .input_active(input_active)
);

wire bclk;
wire lrclk;
wire din;
wire input_active;
wire signed [WORDLENGTH - 1:0] out_left;
wire signed [WORDLENGTH - 1:0] out_right;
 
dds_compiler_0 dds (
  .aclk(aclk),                                // input wire aclk
  .m_axis_data_tvalid(m_axis_data_tvalid),    // output wire m_axis_data_tvalid
  .m_axis_data_tdata(m_axis_data_tdata)       // output wire [15 : 0] m_axis_data_tdata
);

wire signed [WORDLENGTH-1:0] m_axis_data_tdata;
wire m_axis_data_tvalid;


dac #(
    .WORDLENGTH(WORDLENGTH)
) uut (
    .clk(aclk),
    .rst(rst),
    .din(out_left), // Use left channel for testing
    .dvalid(input_active), // Always valid when not in reset
    .dout(dout)
);

wire dout;
*/
    
endmodule
