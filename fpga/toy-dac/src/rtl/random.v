module random #(
    parameter SEED1 = 64'h0123456789abcdef,
    parameter SEED2 = 64'hfedcba9876543210
)(
    input wire clk,
    input wire rst,
    output reg signed [31:0] dout
);

function [63:0] rotl(input [63:0] x, input integer k);
    rotl = (x << k) | (x >> (64 - k));
endfunction

reg [63:0] state0;
reg [63:0] state1;
wire [63:0] tmp1 = state0 ^ state1;
wire [63:0] tmp2 = rotl(state0 + tmp1, 17) + state0;

always @(posedge clk) begin
    if (rst) begin
        state0 <= SEED1;
        state1 <= SEED2;
        dout <= 0;
    end else begin
        state0 <= rotl(state0, 49) ^ tmp1 ^ (tmp1 << 21);
        state1 <= rotl(tmp1, 28);
        dout <= tmp2[31:0]; // Output the lower WORDLENGTH bits
    end
end

endmodule
