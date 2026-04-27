`timescale 1ns / 1ps
`default_nettype none

// Asynchronous Sample-Rate Conversion wrapper.
//
// Wraps the entire digital ASRC datapath:
//
//   I2S in ─► input FIFO ─► (NCO+PI servo paces reads)
//                              │
//                              ▼
//                          src AXI-stream  ─►  [external interpolator] ─►
//                                                                       │
//                          ┌────────────────────────────────────────────┘
//                          ▼
//                       output FIFO ─► (NCO tick_x100 paces reads) ─► DAC
//
// The interpolator is left external because it is a Xilinx IP block.
// Everything else lives in here:
//   - input FIFOs (left/right), elastic buffer between bclk-paced
//     I2S writes and NCO-paced reads
//   - nco instance: PI servo + 2 phase-locked NCOs (1× and 100×
//     the input sample rate)
//   - tick-driven pop-and-hold logic (presents AXI-stream to external
//     interpolator)
//   - output FIFOs (left/right), absorbs the interpolator's bursty
//     output and drains uniformly at tick_x100
//   - out_primed gate
//
// Design intent: the only signals crossing the boundary are the I2S
// stream in, the AXI-stream loop through the interpolator, the rate-
// leveled DAC stream out, and a handful of diagnostics.

module asrc #(
    parameter integer WIDTH          = 32,
    parameter integer FIFO_DEPTH     = 64,
    parameter integer OUT_FIFO_DEPTH = 64,
    parameter integer MCLK_HZ        = 54_000_000,
    parameter [31:0]  INC_NOMINAL    = 32'd3_507_557   // 44.1 kHz @ 54 MHz mclk
) (
    input  wire                    clk,
    input  wire                    rst,
    input  wire                    enable,         // gates servo + held-valid

    // Runtime sample-rate override — forwarded to the internal NCO.
    // 0 = use INC_NOMINAL parameter (backward compatible default).
    input  wire [31:0]             inc_nominal_in,

    // ── I2S input side ──
    input  wire signed [WIDTH-1:0] i2s_left,
    input  wire signed [WIDTH-1:0] i2s_right,
    input  wire                    left_valid,
    input  wire                    right_valid,
    output wire                    fifo_full_l,    // for I2S backpressure
    output wire                    fifo_full_r,

    // ── AXI-stream out to external interpolator (source-paced) ──
    output wire signed [WIDTH-1:0] src_left,
    output wire signed [WIDTH-1:0] src_right,
    output wire                    src_valid_left,
    output wire                    src_valid_right,
    input  wire                    src_ready_left,
    input  wire                    src_ready_right,

    // ── AXI-stream in from external interpolator ──
    input  wire signed [WIDTH-1:0] interp_left,
    input  wire signed [WIDTH-1:0] interp_right,
    input  wire                    interp_valid_left,
    input  wire                    interp_valid_right,

    // ── Rate-leveled output to DAC (drained at tick_x100) ──
    output wire signed [WIDTH-1:0] dac_left,
    output wire signed [WIDTH-1:0] dac_right,
    output wire                    dac_dv_left,
    output wire                    dac_dv_right,

    // ── Diagnostics ──
    output wire [$clog2(FIFO_DEPTH+1)-1:0] fifo_count,
    output wire                    fifo_empty_l,
    output wire                    tick,           // ~44.1 kHz sample tick
    output wire                    adjust,         // 1-cycle pulse per PI update
    output wire signed [15:0]      dbg_error,
    output wire signed [31:0]      dbg_inc_adj
);

    localparam integer FIFO_CW     = $clog2(FIFO_DEPTH+1);
    localparam integer OUT_FIFO_CW = $clog2(OUT_FIFO_DEPTH+1);

    // ── Input FIFOs ─────────────────────────────────────────────
    // Both ports clocked on clk. The fifo module is async-capable
    // but degenerates to plain synchronous behavior when wr_clk ==
    // rd_clk.
    wire signed [WIDTH-1:0] fifo_dout_l;
    wire signed [WIDTH-1:0] fifo_dout_r;
    wire                    fifo_empty_r;
    wire [FIFO_CW-1:0]      fifo_rd_count_l;
    wire [FIFO_CW-1:0]      fifo_rd_count_r;
    wire                    fifo_rd_en_l;
    wire                    fifo_rd_en_r;

    fifo #(
        .WIDTH(WIDTH),
        .DEPTH(FIFO_DEPTH)
    ) fifo_l (
        .wr_clk(clk),
        .wr_rst(rst),
        .wr_data(i2s_left),
        .wr_en(left_valid & ~fifo_full_l),
        .full(fifo_full_l),
        .wr_count(),
        .rd_clk(clk),
        .rd_rst(rst),
        .rd_data(fifo_dout_l),
        .rd_en(fifo_rd_en_l),
        .empty(fifo_empty_l),
        .rd_count(fifo_rd_count_l)
    );

    fifo #(
        .WIDTH(WIDTH),
        .DEPTH(FIFO_DEPTH)
    ) fifo_r (
        .wr_clk(clk),
        .wr_rst(rst),
        .wr_data(i2s_right),
        .wr_en(right_valid & ~fifo_full_r),
        .full(fifo_full_r),
        .wr_count(),
        .rd_clk(clk),
        .rd_rst(rst),
        .rd_data(fifo_dout_r),
        .rd_en(fifo_rd_en_r),
        .empty(fifo_empty_r),
        .rd_count(fifo_rd_count_r)
    );

    assign fifo_count = fifo_rd_count_l;

    // ── NCO + PI servo ──────────────────────────────────────────
    // Run the servo whenever enable is high. We deliberately do NOT
    // gate on a non-zero fifo_count: when the FIFO is empty, the
    // negative error is exactly what should be ramping the integrator
    // down to slow the NCO and let depth recover.
    wire nco_tick;
    wire nco_tick_x100;

    nco #(
        .FIFO_DEPTH (FIFO_DEPTH),
        .SETPOINT   (FIFO_DEPTH/2),
        .MCLK_HZ    (MCLK_HZ),
        .INC_NOMINAL(INC_NOMINAL)
    ) nco_inst (
        .clk          (clk),
        .rst          (rst),
        .enable       (enable),
        .fifo_count   (fifo_rd_count_l),
        .inc_nominal_in(inc_nominal_in),
        .tick         (nco_tick),
        .tick_x100    (nco_tick_x100),
        .dbg_error    (dbg_error),
        .dbg_inc_adj  (dbg_inc_adj),
        .dbg_inc_eff  (),
        .adjust       (adjust)
    );

    assign tick = nco_tick;

    // ── Tick-driven FIFO pop and AXI-stream presentation ────────
    // On each NCO tick: pop one sample from each input FIFO into a
    // holding register and assert tvalid. tvalid stays high until
    // the external interpolator accepts the sample (its tready is
    // asserted ~99.9% of the time).
    reg signed [WIDTH-1:0] held_l = 0;
    reg signed [WIDTH-1:0] held_r = 0;
    reg                    valid_l = 0;
    reg                    valid_r = 0;

    always @(posedge clk) begin
        if (rst || !enable) begin
            valid_l <= 1'b0;
            valid_r <= 1'b0;
        end else begin
            // Clear valid when interpolator accepts.
            if (valid_l && src_ready_left)  valid_l <= 1'b0;
            if (valid_r && src_ready_right) valid_r <= 1'b0;
            // Pop on tick. If the FIFO is empty (underrun), repeat
            // the last held sample — audible glitch but no protocol
            // break.
            if (nco_tick) begin
                if (!fifo_empty_l) held_l <= fifo_dout_l;
                if (!fifo_empty_r) held_r <= fifo_dout_r;
                valid_l <= 1'b1;
                valid_r <= 1'b1;
            end
        end
    end

    assign src_left        = held_l;
    assign src_right       = held_r;
    assign src_valid_left  = valid_l;
    assign src_valid_right = valid_r;

    // FIFO read enables: pop exactly when the tick fires (and FIFO
    // has data). When disabled (test modes upstream) the FIFO is
    // left untouched.
    assign fifo_rd_en_l = enable & nco_tick & ~fifo_empty_l;
    assign fifo_rd_en_r = enable & nco_tick & ~fifo_empty_r;

    // ── Output rate-leveling FIFOs ──────────────────────────────
    // The interpolator emits 100 output samples in a tight ~1100-
    // cycle burst, then idles ~124 cycles until the next input
    // arrives. Without re-pacing, the DAC's din_held would hold each
    // sample for a wildly uneven duration, generating 44.1 kHz IM
    // products.
    //
    // These small FIFOs absorb the burst; the DAC consumes one
    // sample per nco_tick_x100 (rate-locked to exactly 100× the
    // input rate), so post-interpolator samples are uniformly
    // spaced.
    //
    // Depth: max instantaneous backlog ≈ 100 × (1 − 1100/1224) ≈ 10
    // samples. 64-deep gives wide margin for transient PI corrections.
    wire signed [WIDTH-1:0] out_dout_l;
    wire signed [WIDTH-1:0] out_dout_r;
    wire                    out_empty_l, out_empty_r;
    wire                    out_full_l,  out_full_r;
    wire [OUT_FIFO_CW-1:0]  out_count_l, out_count_r;

    // Prime: don't drain output FIFO until it has accumulated some
    // samples, otherwise the very first tick_x100s will underrun
    // before the interpolator pipeline has produced anything.
    reg out_primed = 1'b0;
    always @(posedge clk) begin
        if (rst) out_primed <= 1'b0;
        else if (out_count_l >= OUT_FIFO_DEPTH/2) out_primed <= 1'b1;
        else if (out_empty_l) out_primed <= 1'b0;
    end

    wire out_rd_l = nco_tick_x100 & out_primed & ~out_empty_l;
    wire out_rd_r = nco_tick_x100 & out_primed & ~out_empty_r;

    fifo #(.WIDTH(WIDTH), .DEPTH(OUT_FIFO_DEPTH)) out_fifo_l (
        .wr_clk(clk), .wr_rst(rst),
        .wr_data(interp_left), .wr_en(interp_valid_left & ~out_full_l),
        .full(out_full_l), .wr_count(),
        .rd_clk(clk), .rd_rst(rst),
        .rd_data(out_dout_l), .rd_en(out_rd_l),
        .empty(out_empty_l), .rd_count(out_count_l)
    );

    fifo #(.WIDTH(WIDTH), .DEPTH(OUT_FIFO_DEPTH)) out_fifo_r (
        .wr_clk(clk), .wr_rst(rst),
        .wr_data(interp_right), .wr_en(interp_valid_right & ~out_full_r),
        .full(out_full_r), .wr_count(),
        .rd_clk(clk), .rd_rst(rst),
        .rd_data(out_dout_r), .rd_en(out_rd_r),
        .empty(out_empty_r), .rd_count(out_count_r)
    );

    assign dac_left    = out_dout_l;
    assign dac_right   = out_dout_r;
    assign dac_dv_left  = out_rd_l;
    assign dac_dv_right = out_rd_r;

endmodule

`default_nettype wire
