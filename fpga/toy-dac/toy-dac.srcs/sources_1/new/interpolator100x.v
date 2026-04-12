`timescale 1ns / 1ps
`default_nettype none

// Polyphase x100 interpolation filter.
//
// 1100-tap FIR decomposed into 100 polyphase phases of 11 taps each.
// Coefficients loaded from external .mem file in phase-interleaved order:
//   ROM[phase * 11 + tap] = h[phase + tap * 100]
//
// Single-multiplier MAC engine processes one coefficient per clock cycle.
// Total time: ~1103 clocks per input sample.
// Minimum clock for 44.1 kHz input: 1103 x 44100 = 48.6 MHz.
//
// Port interface matches interpolator2x / interpolator2x_fir for drop-in use.

module interpolator100x #(
    parameter WIDTH = 32
)(
    input  wire                    aclk,
    // AXI-Stream slave (input @ Fs)
    input  wire                    s_axis_data_tvalid,
    output wire                    s_axis_data_tready,
    input  wire signed [WIDTH-1:0] s_axis_data_tdata,
    // AXI-Stream master (output @ 100*Fs)
    output reg                     m_axis_data_tvalid = 0,
    output reg  signed [WIDTH-1:0] m_axis_data_tdata  = 0
);

// ── Filter parameters ──
localparam INTERP  = 100;                  // interpolation factor
localparam NTAPS   = 11;                   // taps per polyphase phase
localparam NCOEFFS = INTERP * NTAPS;       // 1100 total coefficients
localparam CW      = 32;                   // coefficient width (signed)
localparam ACCW    = 68;                   // accumulator: 32+32+4 guard bits
localparam GAIN_SH = 31;                   // right-shift for unity gain

// ── Coefficient ROM (inferred as block RAM) ──
(* rom_style = "block" *)
reg signed [CW-1:0] coeff_rom [0:NCOEFFS-1];
initial $readmemh("interp100x_coeffs.mem", coeff_rom);

// ── Input shift register (11 samples deep) ──
reg signed [WIDTH-1:0] sr [0:NTAPS-1];

// ── BRAM registered read (1-cycle latency) ──
reg signed [CW-1:0] rom_data = 0;
wire [10:0] rom_rd_addr;

// ── Counters and pipeline registers ──
reg [10:0] addr    = 0;     // ROM address counter, 0..NCOEFFS
reg [3:0]  tap     = 0;     // tap counter within phase, 0..10
reg [3:0]  tap_d   = 0;     // delayed tap index (for sr mux in MAC stage)
reg        last_d  = 0;     // delayed "last tap of phase" flag
reg        mac_en  = 0;     // pipeline has valid data for MAC
reg signed [ACCW-1:0] acc = 0;
reg        busy    = 0;

assign s_axis_data_tready = ~busy;

// Clamp ROM address to valid range (avoids OOB during pipeline drain)
assign rom_rd_addr = (addr < NCOEFFS) ? addr : 11'd0;

// BRAM read — separate always block for clean inference
always @(posedge aclk)
    rom_data <= coeff_rom[rom_rd_addr];

// ── Combinational MAC logic ──
wire signed [CW+WIDTH-1:0] product  = rom_data * $signed(sr[tap_d]);
wire signed [ACCW-1:0]     mac_sum  = acc + product;

// Saturating output: extract WIDTH+1 bits, check top 2 for overflow.
// If top two bits are 01 → positive overflow; 10 → negative overflow.
wire signed [WIDTH:0] raw_out = mac_sum[GAIN_SH + WIDTH : GAIN_SH];
wire sat_pos = (~raw_out[WIDTH]) & raw_out[WIDTH-1];   // 01 = too positive
wire sat_neg = raw_out[WIDTH] & (~raw_out[WIDTH-1]);   // 10 = too negative
wire signed [WIDTH-1:0] phase_out = sat_pos ? {1'b0, {(WIDTH-1){1'b1}}} :
                                    sat_neg ? {1'b1, {(WIDTH-1){1'b0}}} :
                                    raw_out[WIDTH-1:0];

integer i;

always @(posedge aclk) begin
    // Default: no output this cycle
    m_axis_data_tvalid <= 1'b0;

    if (!busy) begin
        mac_en <= 1'b0;
        if (s_axis_data_tvalid) begin
            // Push new sample into shift register
            for (i = NTAPS-1; i > 0; i = i-1)
                sr[i] <= sr[i-1];
            sr[0] <= s_axis_data_tdata;

            // Initialize counters — addr 0 will be read by BRAM next cycle
            addr   <= 11'd0;
            tap    <= 4'd0;
            acc    <= {ACCW{1'b0}};
            mac_en <= 1'b0;
            busy   <= 1'b1;
        end
    end else begin
        // ─── Pipeline stage 1: address generation ───
        // Latch tap metadata for MAC stage (available next cycle)
        tap_d  <= tap;
        last_d <= (tap == NTAPS - 1);

        if (addr < NCOEFFS) begin
            mac_en <= 1'b1;
            addr   <= addr + 1'b1;
            if (tap == NTAPS - 1)
                tap <= 4'd0;
            else
                tap <= tap + 1'b1;
        end else begin
            mac_en <= 1'b0;
        end

        // ─── Pipeline stage 2: MAC ───
        if (mac_en) begin
            if (last_d) begin
                // Phase complete → emit output, reset accumulator
                m_axis_data_tdata  <= phase_out;
                m_axis_data_tvalid <= 1'b1;
                acc <= {ACCW{1'b0}};
            end else begin
                acc <= mac_sum;
            end
        end else if (addr >= NCOEFFS) begin
            // Pipeline drained AND all addresses consumed → return to idle
            busy <= 1'b0;
        end
    end
end

// ── Initialize shift register to zero ──
initial begin
    for (i = 0; i < NTAPS; i = i + 1)
        sr[i] = {WIDTH{1'b0}};
end

endmodule
