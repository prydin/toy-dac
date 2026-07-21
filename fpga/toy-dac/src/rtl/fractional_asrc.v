`timescale 1ns / 1ps
`default_nettype none

// Fractional-phase polyphase ASRC core (one audio channel).
//
// Interpolates an input sample stream onto an output grid whose rate is
// determined by an externally-supplied `out_strobe`. The fractional
// position between input samples is tracked by a 32-bit phase
// accumulator; on each output strobe the accumulator advances by a
// runtime `step` value. The integer carry-out of the accumulator
// indicates how many input samples have been consumed since the last
// strobe (<= 1 for the audio rates we care about).
//
// Polyphase filter
// ----------------
// Coefficient ROM is PHASES x TAPS x COEFF_W signed, plus a sentinel
// duplicate of phase 0 at row PHASES (so phase index `idx+1` is always
// valid without a wrap branch). Loaded from COEFF_FILE via $readmemh.
// Read as one true-dual-port BRAM: port A reads (phase i, tap k),
// port B reads (phase i+1, tap k).
//
// Phase accumulator decode (assumes PHASES = 256):
//   bits[31:24]  -> phase index `idx` (0..PHASES-1)
//   bits[23: 8]  -> blend factor alpha (Q0.16, 0..65535)
//   bits[ 7: 0]  -> ignored (sub-alpha)
//
// Per-tap blended coefficient (round-half-up):
//   c_k = c_a[k] + (((c_b[k] - c_a[k]) * alpha) + 2^15) >>> 16
//
// Alpha was widened from 8 to 16 bits to push polyphase image-spurs
// from ~-100 dBFS (set by 256x256 phase-grid quantisation) down
// below -130 dBFS. The half-LSB-up rounding term removes the
// signal-correlated truncation bias that the bare arithmetic shift
// would inject, which otherwise shows up as low-level harmonics.
//
// Pipeline (overlapped MACs, single multiplier, single accumulator)
// ----------------------------------------------------------------
// One MAC walks tap = 0..TAPS-1. Six pipeline stages exist; each tap
// flows through every stage, one cycle apart:
//
//   S1 (cyc k):     drive coeff/sample address registers for tap k
//   S2 (cyc k+1):   BRAM/sample read in flight (registered)
//   S3 (cyc k+2):   coeff_q & samp_q valid -> compute lerp,
//                   register samp_d1 and coeff_blend
//   S4 (cyc k+3):   multiply -> register prod
//   S5 (cyc k+4):   accumulate (init on first tap, add otherwise)
//   S6 (cyc k+5):   if last tap, register data_out and assert dvalid_out
//
// Per-MAC latency = TAPS + 5 cycles (= 69 for TAPS=64). New MACs are
// launched every TAPS cycles regardless of MAC tail in flight: at any
// cycle at most one MAC owns each pipeline stage, so the multiplier
// and accumulator are never contended. Wrapper must guarantee strobes
// arrive no closer than TAPS cycles apart; closer strobes are dropped.
//
// Sample ring buffer is 4*TAPS deep (= 256 for TAPS=64). One TAPS-deep
// span is reserved as FIR history (samples_avail must stay <= SAMP_DEPTH-TAPS
// to avoid stale-BRAM reads), the remaining 3*TAPS span is elasticity for
// the rate servo. Setpoint sits in the middle of the elasticity span
// (samples_avail = SAMP_DEPTH/2) so drift in either direction has
// SAMP_DEPTH/4 = 64 samples of slack before the servo runs out of room.

module fractional_asrc #(
    parameter integer DATA_W      = 32,
    parameter integer COEFF_W     = 18,
    parameter integer PHASES      = 256,
    parameter integer TAPS        = 64,
    parameter         COEFF_FILE  = "frac_asrc.mem",
    parameter integer ACC_W       = 60,
    parameter integer OUT_SHIFT   = COEFF_W - 1
)(
    input  wire                       clk,
    input  wire                       rst,
    input  wire                       enable,

    input  wire signed [DATA_W-1:0]   sample_in,
    input  wire                       sample_valid,

    input  wire                       out_strobe,
    input  wire        [31:0]         step,

    output reg  signed [DATA_W-1:0]   data_out,
    output reg                        dvalid_out,

    output wire                       in_consumed,
    output wire        [31:0]         dbg_phase_acc,
    output wire        [$clog2(TAPS+1)-1:0] dbg_mac_cyc,
    // Unconsumed samples in the internal ring buffer
    // (samples_in_cnt - consumed_cnt). Exposed so an external servo
    // can regulate input-vs-output rate without needing a separate
    // FIFO-depth source.
    output wire        [15:0]         dbg_samples_avail
);

    initial begin
        data_out   = {DATA_W{1'b0}};
        dvalid_out = 1'b0;
    end

    // ----------------------------------------------------------------
    // Derived constants
    // ----------------------------------------------------------------
    localparam integer PHASE_W      = $clog2(PHASES);
    localparam integer TAP_W        = $clog2(TAPS);
    // Ring buffer depth. 4*TAPS (= 256 for TAPS=64) gives 3*TAPS samples
    // of elasticity around the servo setpoint, enough for ~1 minute of
    // typical (50 ppm) crystal drift before reaching a rail.
    localparam integer SAMP_DEPTH   = 4 * TAPS;
    localparam integer SAMP_W       = $clog2(SAMP_DEPTH);
    // Prime-jump target for consumed_cnt. Lands samples_avail at
    // SAMP_DEPTH/2 (= 128 for SAMP_DEPTH=256) -- mid of the safe
    // operating range [1, SAMP_DEPTH-TAPS+1] = [1, 193]. Matches the
    // wrapper's SAMP_SETPOINT so the servo starts inside its deadband.
    localparam integer PRIME_CONSUMED = SAMP_DEPTH - (SAMP_DEPTH/2);
    localparam integer COEFF_DEPTH  = (PHASES + 1) * TAPS;
    localparam integer COEFF_AW     = $clog2(COEFF_DEPTH);
    localparam integer PROD_W       = COEFF_W + DATA_W;

    // ----------------------------------------------------------------
    // Coefficient ROM (true dual-port, $readmemh-loaded)
    // ----------------------------------------------------------------
    (* rom_style = "block" *) reg [COEFF_W-1:0] coeff_mem [0:COEFF_DEPTH-1];

    initial begin
        $readmemh(COEFF_FILE, coeff_mem);
    end

    reg  [COEFF_AW-1:0]       coeff_addr_a = {COEFF_AW{1'b0}};
    reg  [COEFF_AW-1:0]       coeff_addr_b = {COEFF_AW{1'b0}};
    reg  signed [COEFF_W-1:0] coeff_a_q    = {COEFF_W{1'b0}};
    reg  signed [COEFF_W-1:0] coeff_b_q    = {COEFF_W{1'b0}};

    always @(posedge clk) begin
        coeff_a_q <= $signed(coeff_mem[coeff_addr_a]);
        coeff_b_q <= $signed(coeff_mem[coeff_addr_b]);
    end

    // ----------------------------------------------------------------
    // Sample ring buffer (2*TAPS deep -> RAW-hazard free w/ overlap)
    //
    // Two free-running counters track production vs. consumption:
    //   samples_in_cnt: total input samples written (incremented on
    //                   sample_valid).
    //   consumed_cnt:   total input samples whose left-phase boundary
    //                   the phase accumulator has crossed (incremented
    //                   by phase_carry).
    // The convolution origin is sample number `consumed_cnt` (the one
    // we just stepped onto), so the MAC's sample[0] address is
    // consumed_cnt[SAMP_W-1:0]. A MAC kickoff is gated by
    //   (samples_in_cnt - consumed_cnt) > 0  (the new origin sample
    //                                          has actually arrived)
    // && (samples_in_cnt >= TAPS)             (enough history to fill
    //                                          the convolution window)
    // ----------------------------------------------------------------
    reg signed [DATA_W-1:0] samp_mem [0:SAMP_DEPTH-1];
    reg [SAMP_W-1:0]        wptr            = {SAMP_W{1'b0}};
    reg [15:0]              samples_in_cnt  = 16'd0;
    reg [15:0]              consumed_cnt    = 16'd0;

    always @(posedge clk) begin
        if (rst) begin
            wptr           <= {SAMP_W{1'b0}};
            samples_in_cnt <= 16'd0;
        end else if (sample_valid) begin
            samp_mem[wptr] <= sample_in;
            wptr           <= wptr + 1'b1;
            samples_in_cnt <= samples_in_cnt + 1'b1;
        end
    end

    wire [15:0] samples_avail = samples_in_cnt - consumed_cnt;
    // Startup priming. samp_mem is not zeroed on reset, so we must
    // hold off MAC launches until the ring buffer is fully primed
    // AND the convolution origin sits far enough into the buffer
    // that the (TAPS-1) prior taps all read written slots, never
    // stale BRAM.
    //
    // Construction: keep consumed_cnt = 0 while filling; once
    // samples_in_cnt reaches SAMP_DEPTH (= 2*TAPS), jump consumed_cnt
    // forward to TAPS in a single cycle (sequencer below). After the
    // jump, samples_avail = SAMP_DEPTH - TAPS = TAPS, which equals
    // the servo's mid-depth setpoint -- so the servo starts inside
    // its deadband and audio comes up cleanly with no multi-second
    // drain transient. The first launch reads slots TAPS, TAPS-1,
    // ..., 1 (all written by the prime).
    reg primed = 1'b0;
    wire        launch_ok     = primed && (samples_avail != 16'd0);

    reg [SAMP_W-1:0]         samp_addr = {SAMP_W{1'b0}};
    reg signed [DATA_W-1:0]  samp_q    = {DATA_W{1'b0}};

    always @(posedge clk) begin
        samp_q <= samp_mem[samp_addr];
    end

    // ----------------------------------------------------------------
    // Phase accumulator
    // ----------------------------------------------------------------
    reg [31:0] phase_acc   = 32'd0;
    reg        phase_carry = 1'b0;

    wire [32:0] phase_next = {1'b0, phase_acc} + {1'b0, step};

    // ----------------------------------------------------------------
    // S0: launch / per-MAC context
    // ----------------------------------------------------------------
    reg               s0_active = 1'b0;
    reg [TAP_W-1:0]   s0_tap    = {TAP_W{1'b0}};
    reg [PHASE_W-1:0] s0_idx    = {PHASE_W{1'b0}};
    reg [15:0]        s0_alpha  = 16'd0;
    reg [SAMP_W-1:0]  s0_wbase  = {SAMP_W{1'b0}};

    // ----------------------------------------------------------------
    // Pipeline qualifiers. The S1 stage (address drive) does not need a
    // separate valid bit: the launch/continuation paths set s2_v <= 1
    // directly at the same posedge they register the address. That
    // posedge is followed by the BRAM-read posedge (S2 -> S3), so by
    // the time s3_v rises the data on coeff_a_q / samp_q matches.
    // ----------------------------------------------------------------
    reg s2_v = 1'b0, s2_first = 1'b0, s2_last = 1'b0; reg [15:0] s2_alpha = 16'd0;
    reg s3_v = 1'b0, s3_first = 1'b0, s3_last = 1'b0; reg [15:0] s3_alpha = 16'd0;
    reg s4_v = 1'b0, s4_first = 1'b0, s4_last = 1'b0;
    reg s5_v = 1'b0, s5_first = 1'b0, s5_last = 1'b0;
    reg s6_v = 1'b0, s6_first = 1'b0, s6_last = 1'b0;
    reg s7_v = 1'b0, s7_first = 1'b0, s7_last = 1'b0;
    reg               s8_last  = 1'b0;

    (* keep = "true" *) reg signed [DATA_W-1:0]  samp_d1     = {DATA_W{1'b0}};
    (* keep = "true" *) reg signed [DATA_W-1:0]  samp_d2     = {DATA_W{1'b0}};
    (* keep = "true" *) reg signed [DATA_W-1:0]  samp_d3     = {DATA_W{1'b0}};
    (* keep = "true" *) reg signed [COEFF_W:0]   coeff_a_d1  = {(COEFF_W+1){1'b0}};
    (* keep = "true" *) reg signed [COEFF_W:0]   coeff_a_d2  = {(COEFF_W+1){1'b0}};
    (* keep = "true" *) reg signed [COEFF_W:0]   coeff_diff_q = {(COEFF_W+1){1'b0}};
    (* keep = "true" *) reg signed [16:0]        alpha_q     = 17'd0;
    (* keep = "true" *) reg signed [COEFF_W+16:0] lerp_rnd_q = {(COEFF_W+17){1'b0}};
    (* keep = "true" *) reg signed [COEFF_W-1:0] coeff_blend = {COEFF_W{1'b0}};
    reg signed [PROD_W-1:0]  prod        = {PROD_W{1'b0}};
    reg signed [ACC_W-1:0]   acc         = {ACC_W{1'b0}};

    // Pipelined coefficient lerp. Alpha is unsigned Q0.16; widen to
    // 17 bits with a leading sign zero so the multiply is signed-by-signed.
    // Round-half-up on the 16-bit shift kills truncation-bias harmonics.
    // The BRAM coefficient read, lerp multiply, blend add, sample multiply,
    // and accumulator are deliberately separate 108 MHz stages.
    //
    // Alpha is unsigned Q0.16; widen to 17 bits with a leading sign
    // zero so the multiply is signed-by-signed. Round-half-up on the
    // 16-bit shift kills the truncation-bias harmonics.
    wire signed [COEFF_W:0] coeff_diff =
        $signed({coeff_b_q[COEFF_W-1], coeff_b_q})
      - $signed({coeff_a_q[COEFF_W-1], coeff_a_q});
    wire signed [COEFF_W + 16 : 0] lerp_rnd_next =
        (coeff_diff_q * alpha_q) + 18'sh08000;
    wire signed [COEFF_W : 0] blended_next =
        coeff_a_d2 + (lerp_rnd_q >>> 16);

    // ----------------------------------------------------------------
    // Sequencer
    // ----------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            phase_acc    <= 32'd0;
            phase_carry  <= 1'b0;
            consumed_cnt <= 16'd0;
            primed       <= 1'b0;
            s0_active    <= 1'b0;
            s0_tap       <= {TAP_W{1'b0}};
            s0_idx       <= {PHASE_W{1'b0}};
            s0_alpha     <= 16'd0;
            s0_wbase     <= {SAMP_W{1'b0}};
            coeff_addr_a <= {COEFF_AW{1'b0}};
            coeff_addr_b <= {COEFF_AW{1'b0}};
            samp_addr    <= {SAMP_W{1'b0}};
            s2_v <= 1'b0; s2_first <= 1'b0; s2_last <= 1'b0; s2_alpha <= 16'd0;
            s3_v <= 1'b0; s3_first <= 1'b0; s3_last <= 1'b0; s3_alpha <= 16'd0;
            s4_v <= 1'b0; s4_first <= 1'b0; s4_last <= 1'b0;
            s5_v <= 1'b0; s5_first <= 1'b0; s5_last <= 1'b0;
            s6_v <= 1'b0; s6_first <= 1'b0; s6_last <= 1'b0;
            s7_v <= 1'b0; s7_first <= 1'b0; s7_last <= 1'b0;
            s8_last      <= 1'b0;
            samp_d1      <= {DATA_W{1'b0}};
            samp_d2      <= {DATA_W{1'b0}};
            samp_d3      <= {DATA_W{1'b0}};
            coeff_a_d1   <= {(COEFF_W+1){1'b0}};
            coeff_a_d2   <= {(COEFF_W+1){1'b0}};
            coeff_diff_q <= {(COEFF_W+1){1'b0}};
            alpha_q      <= 17'd0;
            lerp_rnd_q   <= {(COEFF_W+17){1'b0}};
            coeff_blend  <= {COEFF_W{1'b0}};
            prod         <= {PROD_W{1'b0}};
            acc          <= {ACC_W{1'b0}};
            data_out     <= {DATA_W{1'b0}};
            dvalid_out   <= 1'b0;
        end else begin
            phase_carry <= 1'b0;
            dvalid_out  <= 1'b0;

            // ---- Prime gate: wait for full buffer, then jump
            //      consumed_cnt forward so samples_avail starts in
            //      the middle of the safe range. SAMP_DEPTH=4*TAPS=256:
            //      safe range is samples_avail in [1, 193]; we land
            //      at 128 (= SAMP_DEPTH/2) which matches the wrapper's
            //      SAMP_SETPOINT so the servo starts inside its deadband.
            //      The first MAC reads slots [128, 127, ..., 65], all
            //      written during the prime fill.
            if (!primed && samples_in_cnt >= SAMP_DEPTH) begin
                primed       <= 1'b1;
                consumed_cnt <= PRIME_CONSUMED[15:0];
            end

            // ---- Stage propagation (default; S2 is overridden below) ----
            s3_v     <= s2_v;     s3_first <= s2_first; s3_last <= s2_last; s3_alpha <= s2_alpha;
            s4_v     <= s3_v;     s4_first <= s3_first; s4_last <= s3_last;
            s5_v     <= s4_v;     s5_first <= s4_first; s5_last <= s4_last;
            s6_v     <= s5_v;     s6_first <= s5_first; s6_last <= s5_last;
            s7_v     <= s6_v;     s7_first <= s6_first; s7_last <= s6_last;
            s8_last  <= s7_last;

            // ---- S1: launch / address-drive ----
            if (enable && out_strobe && !s0_active && launch_ok) begin
                // Launch new MAC: drive tap-0 addresses now
                coeff_addr_a <= {{(COEFF_AW-PHASE_W-TAP_W){1'b0}},
                                 phase_acc[31 -: PHASE_W]} * TAPS;
                coeff_addr_b <= ({{(COEFF_AW-PHASE_W){1'b0}},
                                 phase_acc[31 -: PHASE_W]} + {{(COEFF_AW-1){1'b0}}, 1'b1}) * TAPS;
                samp_addr    <= consumed_cnt[SAMP_W-1:0];
                s2_v         <= 1'b1;
                s2_first     <= 1'b1;
                s2_last      <= (TAPS == 1);
                s2_alpha     <= phase_acc[23 -: 16];
                s0_idx       <= phase_acc[31 -: PHASE_W];
                s0_alpha     <= phase_acc[23 -: 16];
                s0_wbase     <= consumed_cnt[SAMP_W-1:0];
                s0_active    <= 1'b1;
                s0_tap       <= {{(TAP_W-1){1'b0}}, 1'b1};   // next cycle: drive tap 1
                phase_acc    <= phase_next[31:0];
                phase_carry  <= phase_next[32];
                consumed_cnt <= consumed_cnt + {15'd0, phase_next[32]};
            end else if (s0_active) begin
                coeff_addr_a <= {{(COEFF_AW-PHASE_W){1'b0}}, s0_idx} * TAPS
                                + {{(COEFF_AW-TAP_W){1'b0}}, s0_tap};
                coeff_addr_b <= ({{(COEFF_AW-PHASE_W){1'b0}}, s0_idx} + {{(COEFF_AW-1){1'b0}}, 1'b1}) * TAPS
                                + {{(COEFF_AW-TAP_W){1'b0}}, s0_tap};
                samp_addr    <= s0_wbase
                                - {{(SAMP_W-TAP_W){1'b0}}, s0_tap};
                s2_v         <= 1'b1;
                s2_first     <= 1'b0;
                s2_last      <= (s0_tap == TAPS - 1);
                s2_alpha     <= s0_alpha;
                if (s0_tap == TAPS - 1) begin
                    s0_active <= 1'b0;
                end else begin
                    s0_tap    <= s0_tap + 1'b1;
                end
            end else begin 
                s2_v     <= 1'b0;
                s2_first <= 1'b0;
                s2_last  <= 1'b0;
                s2_alpha <= s0_alpha;   // hold
            end

            // ---- S3: latch BRAM outputs and interpolation inputs ----
            if (s3_v) begin
                samp_d1      <= samp_q;
                coeff_a_d1   <= $signed({coeff_a_q[COEFF_W-1], coeff_a_q});
                coeff_diff_q <= coeff_diff;
                alpha_q      <= $signed({1'b0, s3_alpha});
            end

            // ---- S4: coefficient-lerp multiply ----
            if (s4_v) begin
                samp_d2    <= samp_d1;
                coeff_a_d2 <= coeff_a_d1;
                lerp_rnd_q <= lerp_rnd_next;
            end

            // ---- S5: coefficient blend ----
            if (s5_v) begin
                samp_d3     <= samp_d2;
                coeff_blend <= blended_next[COEFF_W-1:0];
            end

            // ---- S6: sample multiply ----
            if (s6_v) begin
                prod <= coeff_blend * samp_d3;
            end

            // ---- S7: accumulate (init on first tap, add otherwise) ----
            if (s7_v) begin
                if (s7_first)
                    acc <= {{(ACC_W - PROD_W){prod[PROD_W-1]}}, prod};
                else
                    acc <= acc + {{(ACC_W - PROD_W){prod[PROD_W-1]}}, prod};
            end

            // ---- S8: emit output one cycle after final accumulate ----
            if (s8_last) begin
                data_out   <= sat_round(acc, OUT_SHIFT);
                dvalid_out <= 1'b1;
            end
        end
    end

    // ----------------------------------------------------------------
    // Saturating round-shift to DATA_W signed
    // ----------------------------------------------------------------
    function automatic signed [DATA_W-1:0] sat_round (
        input signed [ACC_W-1:0] value,
        input integer            shift
    );
        reg signed [ACC_W-1:0]  shifted;
        reg signed [DATA_W-1:0] max_pos;
        reg signed [DATA_W-1:0] max_neg;
        begin
            shifted = (value + (shift > 0 ? (64'sd1 <<< (shift - 1)) : 64'sd0)) >>> shift;
            max_pos = {1'b0, {(DATA_W-1){1'b1}}};
            max_neg = {1'b1, {(DATA_W-1){1'b0}}};
            if (shifted > max_pos)
                sat_round = max_pos;
            else if (shifted < max_neg)
                sat_round = max_neg;
            else
                sat_round = shifted[DATA_W-1:0];
        end
    endfunction

    // ----------------------------------------------------------------
    // Diagnostics
    // ----------------------------------------------------------------
    assign in_consumed   = phase_carry;
    assign dbg_phase_acc = phase_acc;
    assign dbg_mac_cyc   = {{($clog2(TAPS+1)-TAP_W){1'b0}}, s0_tap};
    assign dbg_samples_avail = samples_avail;

endmodule

`default_nettype wire
