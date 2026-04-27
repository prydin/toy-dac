`timescale 1ns / 1ps
`default_nettype none

// Soft-PLL: nudges the MMCM dynamic phase-shift port at a controlled
// rate so that the output clock tracks an external reference over long
// timescales. Uses FIFO fill depth as the phase detector (depth is the
// integral of producer-vs-consumer frequency error).
//
// How it works
// ────────────
// Each MMCM phase-shift pulse moves an output edge by a fixed fraction
// of the VCO period (≈ 1/56 of T_vco for 7-series MMCMs). A *steady
// stream* of pulses at rate R pulses/sec therefore produces a fractional
// frequency offset Δf/f ≈ R / (56 × f_vco).
//
// The control variable is "pulses per update window". On every update
// tick (UPDATE_HZ Hz), we measure the FIFO fill error, run a P
// controller, and load the resulting signed pulse count into a counter.
// The pulse engine then trickles that many phase-shift pulses out over
// the next update window, evenly enough that the loop sees them as a
// constant frequency offset.
//
// Sign convention: the wizard's PSINCDEC=1 advances phase, which over
// time *reduces* the output frequency slightly (each advance pulls the
// next edge sooner, but successive advances on the same edge stretch
// the period). If your loop runs the wrong way, flip INVERT_DIR.
//
// Loop dynamics (P controller, integrator plant)
// ──────────────────────────────────────────────
// Open-loop: pulse rate → frequency offset → integrated by FIFO depth.
// Closed-loop bandwidth ≈ KP × (pulses-per-fill-unit) × (Hz-per-pulse) /
// (samples per second). With UPDATE_HZ = 1 and KP = a few, the
// bandwidth is well under 1 Hz, which is what you want for ignoring
// short-term jitter.

module soft_pll #(
    parameter integer FIFO_DEPTH    = 64,         // matches fifo.v DEPTH
    parameter integer SETPOINT      = FIFO_DEPTH/2,
    parameter integer MCLK_HZ       = 54_000_000, // nominal mclk
    parameter integer UPDATE_HZ     = 1,          // measurement rate
    parameter integer KP            = 8,          // pulses per fill-unit error per update
    parameter integer MAX_PULSES    = 2048,       // safety clamp on |pulses_per_window|
    parameter         INVERT_DIR    = 1'b0
)(
    input  wire                              mclk,
    input  wire                              rst,
    input  wire [$clog2(FIFO_DEPTH+1)-1:0]   fifo_count,

    // To MMCM dynamic phase-shift port
    output reg                               psen     = 1'b0,
    output reg                               psincdec = 1'b0,
    input  wire                              psdone,

    // Debug
    output reg signed [15:0]                 dbg_error            = 0,
    output reg signed [15:0]                 dbg_pulses_remaining = 0
);

    // ── Update tick (UPDATE_HZ Hz) ────────────────────────────────────
    localparam integer UPDATE_DIV = MCLK_HZ / UPDATE_HZ;
    localparam integer TICK_W     = $clog2(UPDATE_DIV);
    reg [TICK_W-1:0] tick_cnt = 0;
    wire             update_tick = (tick_cnt == UPDATE_DIV - 1);

    // ── Control loop state ────────────────────────────────────────────
    // Sign-extend fifo_count into a signed register wide enough for
    // (count − SETPOINT). 16 bits is plenty for any sensible FIFO_DEPTH.
    wire signed [15:0] fill_signed   = $signed({1'b0, fifo_count});
    wire signed [15:0] error_now     = fill_signed - SETPOINT;

    // Pulse spacing within an update window: emit one pulse every
    // SPACING mclk cycles. Recompute when a new error arrives.
    // SPACING = UPDATE_DIV / |pulses_per_window|, clamped.
    reg [TICK_W-1:0] spacing       = {TICK_W{1'b1}};   // huge → no pulses
    reg [TICK_W-1:0] spacing_cnt   = 0;
    reg              direction     = 1'b0;             // 1 = INC, 0 = DEC

    // Pulse engine state
    localparam PS_IDLE = 1'b0;
    localparam PS_WAIT = 1'b1;
    reg ps_state = PS_IDLE;

    // ── Update tick + PI-style scheduling ─────────────────────────────
    reg signed [31:0] pulses_per_window = 0;  // signed; sign drives direction
    reg signed [31:0] abs_pulses        = 0;

    always @(posedge mclk) begin
        if (rst) begin
            tick_cnt             <= 0;
            pulses_per_window    <= 0;
            abs_pulses           <= 0;
            spacing              <= {TICK_W{1'b1}};
            spacing_cnt          <= 0;
            direction            <= 1'b0;
            dbg_error            <= 0;
            dbg_pulses_remaining <= 0;
        end else begin
            // ── Update tick: latch error, recompute schedule ──
            if (update_tick) begin
                tick_cnt          <= 0;

                // P controller. Negative error (fifo too empty) means the
                // consumer (this clock) is too fast and we must slow it
                // down. Positive error means we need to speed up.
                pulses_per_window <= KP * error_now;

                // Direction bit (and clamp to MAX_PULSES)
                if ((KP * error_now) > MAX_PULSES) begin
                    abs_pulses <= MAX_PULSES;
                    direction  <= INVERT_DIR ? 1'b0 : 1'b1; // speed up
                end else if ((KP * error_now) < -MAX_PULSES) begin
                    abs_pulses <= MAX_PULSES;
                    direction  <= INVERT_DIR ? 1'b1 : 1'b0; // slow down
                end else if ((KP * error_now) >= 0) begin
                    abs_pulses <= KP * error_now;
                    direction  <= INVERT_DIR ? 1'b0 : 1'b1;
                end else begin
                    abs_pulses <= -(KP * error_now);
                    direction  <= INVERT_DIR ? 1'b1 : 1'b0;
                end

                // Pulse spacing for the *next* window. Avoid divide-by-zero.
                if (abs_pulses == 0)
                    spacing <= {TICK_W{1'b1}};
                else
                    spacing <= UPDATE_DIV / abs_pulses;

                spacing_cnt          <= 0;
                dbg_error            <= error_now;
                dbg_pulses_remaining <= abs_pulses[15:0];
            end else begin
                tick_cnt    <= tick_cnt + 1;
                spacing_cnt <= spacing_cnt + 1;
            end

            // ── Pulse engine: emit one psen when spacing_cnt rolls over,
            //    then wait for psdone before allowing the next ──
            psen <= 1'b0; // default
            case (ps_state)
                PS_IDLE: begin
                    if ((abs_pulses != 0) && (spacing_cnt >= spacing)) begin
                        psen        <= 1'b1;
                        psincdec    <= direction;
                        spacing_cnt <= 0;
                        ps_state    <= PS_WAIT;
                        if (dbg_pulses_remaining != 0)
                            dbg_pulses_remaining <= dbg_pulses_remaining - 1;
                    end
                end
                PS_WAIT: begin
                    if (psdone)
                        ps_state <= PS_IDLE;
                end
            endcase
        end
    end

endmodule

`default_nettype wire
