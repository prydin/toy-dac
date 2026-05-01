`timescale 1ns / 1ps
`default_nettype none

// ── Clock generator ─────────────────────────────────────────────────
// Direct MMCME2_ADV instantiation that replaces the Vivado Clocking
// Wizard IP. Port list is intentionally identical to the wizard's
// `clock` module so the rest of the design (root.v) does not have to
// change.
//
//   12 MHz crystal in  → 108 MHz mclk out
//
// Math (Spartan-7 -1, MMCME2_ADV):
//     VCO = f_in * (M / D)         must be in 600 – 1200 MHz
//     f_out = VCO / O
//   With M = 63, D = 1, O = 7:
//     VCO   = 12 MHz × 63 / 1   = 756 MHz   ✓ in range
//     f_out = 756 MHz   / 7     = 108 MHz   ✓
//
// CLKIN1 is expected to be already buffered (top-level instantiates an
// IBUF on the package pin and feeds clk_ibuf here). The feedback path
// uses an internal BUFG ("ZHOLD" compensation), matching what the
// wizard generates for a single-output MMCM with no input switchover.
//
// Dynamic phase shift ports (psclk/psen/psincdec/psdone) are wired
// through to the MMCM but are unused by the current design (root.v
// ties psen to 0). They are kept on the interface so this module can
// be a literal drop-in replacement for the IP.
module clock (
    input  wire clk_in1,    // buffered reference clock (12 MHz)
    input  wire reset,      // active-high MMCM reset
    output wire locked,     // MMCM lock indicator
    output wire mclk,       // 108 MHz output clock (on a BUFG)

    // Dynamic phase shift interface (unused by the current design).
    input  wire psclk,
    input  wire psen,
    input  wire psincdec,
    output wire psdone
);

    // ── MMCM nets ──
    wire clkfb_out;        // raw feedback from CLKFBOUT
    wire clkfb_in;         // feedback into CLKFBIN, via BUFG
    wire mclk_out;         // raw CLKOUT0
    wire clkout0b_unused, clkout1_unused, clkout1b_unused;
    wire clkout2_unused,  clkout2b_unused;
    wire clkout3_unused,  clkout3b_unused;
    wire clkout4_unused,  clkout5_unused, clkout6_unused;
    wire clkfbout_b_unused;
    wire [15:0] do_unused;
    wire        drdy_unused;
    wire        clkfbstopped_unused, clkinstopped_unused;

    MMCME2_ADV #(
        // ── Bandwidth & compensation ──
        .BANDWIDTH           ("OPTIMIZED"),
        .COMPENSATION        ("ZHOLD"),

        // ── Input clock ──
        .CLKIN1_PERIOD       (83.333),         // 12 MHz
        .CLKIN2_PERIOD       (83.333),         // unused, but must be valid
        .REF_JITTER1         (0.010),
        .REF_JITTER2         (0.010),

        // ── Multiply / divide ──
        .CLKFBOUT_MULT_F     (63.000),
        .CLKFBOUT_PHASE      (0.000),
        .CLKFBOUT_USE_FINE_PS("FALSE"),
        .DIVCLK_DIVIDE       (1),

        // ── Output 0: 108 MHz mclk ──
        .CLKOUT0_DIVIDE_F    (7.000),
        .CLKOUT0_DUTY_CYCLE  (0.500),
        .CLKOUT0_PHASE       (0.000),
        .CLKOUT0_USE_FINE_PS ("FALSE"),

        // ── Outputs 1-6 unused (sensible defaults so DRC is happy) ──
        .CLKOUT1_DIVIDE      (1),
        .CLKOUT2_DIVIDE      (1),
        .CLKOUT3_DIVIDE      (1),
        .CLKOUT4_DIVIDE      (1),
        .CLKOUT5_DIVIDE      (1),
        .CLKOUT6_DIVIDE      (1),
        .CLKOUT4_CASCADE     ("FALSE"),

        // ── Startup ──
        .STARTUP_WAIT        ("FALSE")
    ) mmcm_inst (
        // Outputs
        .CLKFBOUT       (clkfb_out),
        .CLKFBOUTB      (clkfbout_b_unused),
        .CLKOUT0        (mclk_out),
        .CLKOUT0B       (clkout0b_unused),
        .CLKOUT1        (clkout1_unused),
        .CLKOUT1B       (clkout1b_unused),
        .CLKOUT2        (clkout2_unused),
        .CLKOUT2B       (clkout2b_unused),
        .CLKOUT3        (clkout3_unused),
        .CLKOUT3B       (clkout3b_unused),
        .CLKOUT4        (clkout4_unused),
        .CLKOUT5        (clkout5_unused),
        .CLKOUT6        (clkout6_unused),
        .LOCKED         (locked),
        .CLKFBSTOPPED   (clkfbstopped_unused),
        .CLKINSTOPPED   (clkinstopped_unused),

        // Feedback
        .CLKFBIN        (clkfb_in),

        // Inputs
        .CLKIN1         (clk_in1),
        .CLKIN2         (1'b0),
        .CLKINSEL       (1'b1),     // select CLKIN1
        .PWRDWN         (1'b0),
        .RST            (reset),

        // Dynamic reconfiguration port — unused, tied off
        .DADDR          (7'h00),
        .DCLK           (1'b0),
        .DEN            (1'b0),
        .DI             (16'h0000),
        .DWE            (1'b0),
        .DO             (do_unused),
        .DRDY           (drdy_unused),

        // Dynamic phase shift — pass through to the MMCM
        .PSCLK          (psclk),
        .PSEN           (psen),
        .PSINCDEC       (psincdec),
        .PSDONE         (psdone)
    );

    // ── Output buffers ──
    BUFG clkfb_buf (.I(clkfb_out), .O(clkfb_in));
    BUFG mclk_buf  (.I(mclk_out),  .O(mclk));

endmodule

`default_nettype wire
