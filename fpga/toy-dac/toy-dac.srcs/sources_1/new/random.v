module random (
    input wire clk,
    input wire rst,
    output reg signed [31:0] dout
);

function [63:0] rotl(input [63:0] x, input integer k);
    rotl = (x << k) | (x >> (64 - k));
endfunction


/*
static INLINE uint64_t rotl(const uint64_t x, int k)
{
  return (x << k) | (x >> (64 - k));
}

static INLINE uint64_t xoroshiro128_next(uint64_t *s)
{
  const uint64_t s0 = s[0];
  uint64_t s1 = s[1];
  const uint64_t result = s0 + s1;

  s1 ^= s0;
  s[0] = rotl(s0, 24) ^ s1 ^ (s1 << 16); // a, b
  s[1] = rotl(s1, 37);                   // c

  return result;
}

static INLINE uint64_t xoroshiro128plusplus_next(uint64_t *s) {
	const uint64_t s0 = s[0];
	uint64_t s1 = s[1];
	const uint64_t result = rotl(s0 + s1, 17) + s0;

	s1 ^= s0;
	s[0] = rotl(s0, 49) ^ s1 ^ (s1 << 21); // a, b
	s[1] = rotl(s1, 28); // c 

	return result;
} */

reg [63:0] state0;
reg [63:0] state1;
wire [63:0] tmp1 = state0 ^ state1;
wire [63:0] tmp2 = rotl(state0 + tmp1, 17) + state0;

always @(posedge clk) begin
    if (rst) begin
        state0 <= 64'h0123456789abcdef; // Arbitrary non-zero seed
        state1 <= 64'hfedcba9876543210; // Arbitrary non-zero seed
        dout <= 0;
    end else begin
        state0 <= rotl(state0, 49) ^ tmp1 ^ (tmp1 << 21);
        state1 <= rotl(tmp1, 28);
        dout <= tmp2[31:0]; // Output the lower WORDLENGTH bits
    end
end

endmodule
