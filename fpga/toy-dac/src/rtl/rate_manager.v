`timescale 1ns / 1ps
`default_nettype none

// rate_manager
// ────────────
// Watches `rate_detect` output, classifies the measured lrclk period
// into one of a small set of supported sample rates, debounces, and
// drives the ASRC's nominal NCO increment + reset/mute lines.
//
// Supported rates (Plan C, FIR is the bottleneck above ~48 kHz):
//   32 kHz, 44.1 kHz, 48 kHz
// Anything outside those ranges raises `unsupported` and keeps the
// audio output muted.
//
// Period boundaries (mclk cycles per lrclk period, computed from
// MCLK_HZ at synthesis):
//
//   period < MCLK/50000   → "too fast" (>50 kHz) → unsupported
//   .. < midpoint(48,44.1) → 48 kHz
//   .. < midpoint(44.1,32) → 44.1 kHz
//   .. < MCLK/30000        → 32 kHz
//   period ≥ MCLK/30000   → "too slow" (<30 kHz) → unsupported
//
// Sequence after reset / rate change
// ──────────────────────────────────
//   1. UNLOCKED: asrc_rst high, mute high. Wait for `rate_valid`
//      pulses. Track a candidate rate; require MATCHES_REQUIRED
//      consecutive matching measurements before committing.
//   2. SETTLE: latch new inc_nominal, release asrc_rst, mute stays
//      high while ASRC's PI servo converges (~SETTLE_MS).
//   3. LOCKED: mute released, rate_locked high. If a new rate is
//      confirmed (different from current), drop straight back to
//      UNLOCKED to repeat the sequence.

module rate_manager #(
    parameter integer MCLK_HZ          = 108_000_000,
    parameter integer FS_OUT_HZ        = 1_687_500,    // mclk / OUT_DIV (= 108 MHz / 64)
    parameter integer MATCHES_REQUIRED = 3,
    parameter integer SETTLE_MS        = 250
)(
    input  wire        clk,
    input  wire        rst,

    // From rate_detect
    input  wire        rate_valid,
    input  wire [15:0] period,        // mclk cycles per lrclk period

    // To ASRC / DAC
    output reg  [31:0] inc_nominal = 32'd3_507_557,  // legacy NCO step (kept alive)
    output reg  [31:0] step_nominal = 32'd112_261_131, // fractional ASRC step (Q0.32 of Fs_in/Fs_out)
    output wire        asrc_rst,                     // driven by stretch counter below
    output reg         mute        = 1'b1,
    output reg         rate_locked = 1'b0,
    output reg         unsupported = 1'b0,
    output reg  [1:0]  rate_code   = 2'd1            // 0=32k 1=44.1k 2=48k 3=invalid
);

    // ── Period classification boundaries ───────────────────────────
    localparam integer P_HIGH_LIMIT = MCLK_HZ / 50_000;
    localparam integer B_48_44      = (2 * MCLK_HZ) / (48_000 + 44_100);
    localparam integer B_44_32      = (2 * MCLK_HZ) / (44_100 + 32_000);
    localparam integer P_LOW_LIMIT  = MCLK_HZ / 30_000;

    // ── INC_NOMINAL lookup: round(2^32 × Fs / MCLK_HZ) ─────────────
    // (Legacy: drives the old NCO-based asrc; kept alive during
    // bring-up so the previous code path remains buildable.)
    localparam [31:0] INC_32K  =
        (((64'd1 << 32) * 32_000)  + (MCLK_HZ/2)) / MCLK_HZ;
    localparam [31:0] INC_44_1 =
        (((64'd1 << 32) * 44_100)  + (MCLK_HZ/2)) / MCLK_HZ;
    localparam [31:0] INC_48K  =
        (((64'd1 << 32) * 48_000)  + (MCLK_HZ/2)) / MCLK_HZ;

    // ── STEP_NOMINAL lookup: round(2^32 × Fs_in / Fs_out) ──────────
    // Fractional-phase ASRC step: how much of an input sample to
    // advance per output strobe. For Fs_out = 1.6875 MHz:
    //   44_100 / 1_687_500 * 2^32 ≈ 112_261_131
    //   48_000 / 1_687_500 * 2^32 ≈ 122_175_407
    //   32_000 / 1_687_500 * 2^32 ≈  81_450_271 (32k flagged unsupported)
    localparam [31:0] STEP_32K =
        (((64'd1 << 32) * 32_000) + (FS_OUT_HZ/2)) / FS_OUT_HZ;
    localparam [31:0] STEP_44_1 =
        (((64'd1 << 32) * 44_100) + (FS_OUT_HZ/2)) / FS_OUT_HZ;
    localparam [31:0] STEP_48K =
        (((64'd1 << 32) * 48_000) + (FS_OUT_HZ/2)) / FS_OUT_HZ;

    localparam [1:0] RC_32K = 2'd0;
    localparam [1:0] RC_44K = 2'd1;
    localparam [1:0] RC_48K = 2'd2;
    localparam [1:0] RC_INV = 2'd3;

    // ── ASRC reset ────────────────────────────────────────────
    // asrc_rst is held HIGH for the entire S_UNLOCKED period.
    // S_UNLOCKED lasts at least ~17 ms (3 debounce measurements at
    // the new rate), which is plenty of time to fully flush the
    // FIFOs and clear NCO/PI state. Driving from FSM state directly
    // ensures asrc_rst stays asserted until inc_nominal has been
    // latched to the new value (which happens on the S_UNLOCKED →
    // S_SETTLE transition).

    // Combinational classifier
    function [1:0] classify;
        input [15:0] p;
        begin
            if      (p <  P_HIGH_LIMIT[15:0]) classify = RC_INV;
            else if (p <  B_48_44[15:0])      classify = RC_48K;
            else if (p <  B_44_32[15:0])      classify = RC_44K;
            else if (p <  P_LOW_LIMIT[15:0])  classify = RC_32K;
            else                              classify = RC_INV;
        end
    endfunction

    function [31:0] inc_for;
        input [1:0] rc;
        begin
            case (rc)
                RC_32K:  inc_for = INC_32K;
                RC_44K:  inc_for = INC_44_1;
                RC_48K:  inc_for = INC_48K;
                default: inc_for = INC_44_1;
            endcase
        end
    endfunction

    function [31:0] step_for;
        input [1:0] rc;
        begin
            case (rc)
                RC_32K:  step_for = STEP_32K;
                RC_44K:  step_for = STEP_44_1;
                RC_48K:  step_for = STEP_48K;
                default: step_for = STEP_44_1;
            endcase
        end
    endfunction

    // ── Candidate tracking ─────────────────────────────────────────
    // After every rate_valid pulse, classify; if it matches the
    // running candidate, increment the match counter. If it doesn't
    // (or comes back invalid), reset the candidate. Once the count
    // reaches MATCHES_REQUIRED we have a confirmed rate.
    reg [1:0] candidate   = RC_INV;
    reg [3:0] match_count = 0;
    wire [1:0] new_rc = classify(period);
    wire       confirmed = (match_count >= MATCHES_REQUIRED[3:0])
                         && (candidate != RC_INV);

    // ── Settle timer ───────────────────────────────────────────────
    localparam integer SETTLE_CYCLES = (MCLK_HZ / 1000) * SETTLE_MS;
    localparam integer SETTLE_W      = $clog2(SETTLE_CYCLES + 1);
    reg [SETTLE_W-1:0] settle_cnt = 0;

    // ── FSM ────────────────────────────────────────────────────────
    localparam [1:0] S_UNLOCKED = 2'd0;
    localparam [1:0] S_SETTLE   = 2'd1;
    localparam [1:0] S_LOCKED   = 2'd2;
    reg [1:0] state = S_UNLOCKED;

    always @(posedge clk) begin
        if (rst) begin
            inc_nominal     <= INC_44_1;
            step_nominal    <= STEP_44_1;
            mute            <= 1'b1;
            rate_locked     <= 1'b0;
            unsupported     <= 1'b0;
            rate_code       <= RC_44K;
            candidate       <= RC_INV;
            match_count     <= 0;
            settle_cnt      <= 0;
            state           <= S_UNLOCKED;
        end else begin
            // ─── Candidate tracking on every rate_valid pulse ──────
            if (rate_valid) begin
                if (new_rc == RC_INV) begin
                    unsupported <= 1'b1;
                    candidate   <= RC_INV;
                    match_count <= 0;
                end else begin
                    unsupported <= 1'b0;
                    if (new_rc == candidate) begin
                        if (match_count < MATCHES_REQUIRED[3:0])
                            match_count <= match_count + 1'b1;
                    end else begin
                        candidate   <= new_rc;
                        match_count <= 4'd1;
                    end
                end
            end

            // ─── FSM ────────────────────────────────────────────────
            case (state)
                S_UNLOCKED: begin
                    mute        <= 1'b1;
                    rate_locked <= 1'b0;
                    if (confirmed) begin
                        rate_code    <= candidate;
                        inc_nominal  <= inc_for(candidate);
                        step_nominal <= step_for(candidate);
                        settle_cnt   <= SETTLE_CYCLES[SETTLE_W-1:0];
                        state        <= S_SETTLE;
                    end
                end

                S_SETTLE: begin
                    mute        <= 1'b1;
                    rate_locked <= 1'b0;
                    if (settle_cnt != 0)
                        settle_cnt <= settle_cnt - 1'b1;
                    else begin
                        mute        <= 1'b0;
                        rate_locked <= 1'b1;
                        state       <= S_LOCKED;
                    end
                    // Abort settle if a different rate is confirmed.
                    // Force re-debounce by clearing match_count so we
                    // can't immediately re-trigger again on the next
                    // cycle.
                    if (confirmed && candidate != rate_code) begin
                        mute        <= 1'b1;
                        match_count <= 4'd0;
                        state       <= S_UNLOCKED;
                    end
                end

                S_LOCKED: begin
                    mute        <= 1'b0;
                    rate_locked <= 1'b1;
                    // On a confirmed rate change, drop back to
                    // S_UNLOCKED. asrc_rst will be asserted (driven
                    // by state==S_UNLOCKED) until the new rate is
                    // re-debounced (~17 ms minimum), at which point
                    // inc_nominal is latched and asrc_rst drops.
                    // Clear match_count so we re-debounce from
                    // scratch.
                    if (confirmed && candidate != rate_code) begin
                        mute        <= 1'b1;
                        rate_locked <= 1'b0;
                        match_count <= 4'd0;
                        state       <= S_UNLOCKED;
                    end
                end

                default: state <= S_UNLOCKED;
            endcase
        end
    end

    // asrc_rst is held high for the entire S_UNLOCKED period (and on
    // top-level rst). This guarantees asrc stays in reset until the
    // moment inc_nominal latches the new rate (S_UNLOCKED → S_SETTLE).
    assign asrc_rst = rst | (state == S_UNLOCKED);

endmodule

`default_nettype wire
