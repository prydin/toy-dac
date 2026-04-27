`timescale 1ns / 1ps
`default_nettype none

// Simple ×2 linear interpolation filter.
// For each input sample, outputs two samples:
//   1. midpoint of previous and current  (interpolated)
//   2. current sample                    (original)
// Port names match the Xilinx FIR compiler for drop-in replacement.

module interpolator2x #(
    parameter WIDTH = 32
)(
    input  wire                    aclk,
    // AXI-Stream slave (input)
    input  wire                    s_axis_data_tvalid,
    output wire                    s_axis_data_tready,
    input  wire signed [WIDTH-1:0] s_axis_data_tdata,
    // AXI-Stream master (output)
    output reg                     m_axis_data_tvalid = 0,
    output reg  signed [WIDTH-1:0] m_axis_data_tdata  = 0
);

reg signed [WIDTH-1:0] prev = 0;
reg signed [WIDTH-1:0] curr = 0;
reg phase = 0;  // 0 = idle/output interp, 1 = output original

assign s_axis_data_tready = ~phase;

always @(posedge aclk) begin
    if (!phase) begin
        // IDLE — waiting for input
        if (s_axis_data_tvalid) begin
            prev <= curr;
            curr <= s_axis_data_tdata;
            // Output interpolated midpoint: (prev + new) / 2
            // Arithmetic right-shift each by 1 first to avoid overflow
            m_axis_data_tdata  <= (curr >>> 1) + (s_axis_data_tdata >>> 1);
            m_axis_data_tvalid <= 1'b1;
            phase <= 1'b1;
        end else begin
            m_axis_data_tvalid <= 1'b0;
        end
    end else begin
        // Output the original sample
        m_axis_data_tdata  <= curr;
        m_axis_data_tvalid <= 1'b1;
        phase <= 1'b0;
    end
end

endmodule
