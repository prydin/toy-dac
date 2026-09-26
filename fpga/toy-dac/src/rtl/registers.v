`default_nettype none

// I2C DAC register map
// ====================
// This block implements the FPGA-side, addressable I2C register map for the
// DAC control/status interface. It is exposed on the 7-bit address 0x50.
//
// Protocol:
//   - single-byte register pointer, no auto-increment
//   - register read: START, slave addr + R, then read 1 byte
//   - register write: START, slave addr + W, reg pointer, data byte
//   - registers 0x03 and 0x0C are writable; the others are read-only
//
// Register 0x00 — RATE_STATUS (RO)
//   Bit 7: family flag, 0 = 44.1 kHz family, 1 = 48 kHz family
//   Bits 6:0: family-multiple / selected-rate code
//     0x00 = no lock / unsupported / 32 kHz case
//     0x01 = 44.1 kHz
//     0x81 = 48 kHz
//   This is derived from rate_manager/rate_detect on mclk and reflects the
//   current audio family classification.
//
// Register 0x01 — FIFO_FULLNESS (RO)
//   8-bit scaled FIFO occupancy from the ASRC ring buffer.
//   0x00 = empty, 0xFF = full or saturated, mapped from asrc_samp_avail_l.
//   The value is synchronized from sclk to mclk with a two-FF pipeline so it
//   can be polled with a low-rate housekeeping read.
//
// Register 0x02 — STATUS (RO)
//   Bit 0: underrun sticky flag
//   Bit 1: overrun sticky flag
//   Bits 7:2: reserved = 0
//   The flags are latched for roughly 1 second after the ASRC enters the
//   unsafe ring-buffer conditions (empty or near-full).
//
// Register 0x03 — CTRL (RW)
//   Bit 0: dither enable (same as btn[1]/dither_en)
//   Bit 1: test-tone mode selection (0 = MODE_I2S, 1 = MODE_DDS)
//   Bit 2: bypass interpolation control (stored, currently unused by datapath)
//   Bit 3: output mute control (forces DAC output pins low)
//   Bit 4: input mute control (forces I2S input samples to zero before ASRC)
//   Bits 7:5: reserved = 0
//   The register is read back as the current control state and written via the
//   I2C slave's reg3_wdata/reg3_wr pulse into the housekeeping logic in root.v.
//
// Register 0x04 — SERVO_ERROR_LO (RO)
//   Low byte of signed two's-complement FIFO error, in samples.
// Register 0x05 — SERVO_ERROR_HI (RO)
//   High byte of signed two's-complement FIFO error, in samples. Reconstruct
//   error as a signed 16-bit value; positive means above the 128-sample target.
//
// Register 0x06..0x09 — SERVO_STEP_ADJ (RO)
//   Four little-endian bytes of the signed two's-complement ASRC step
//   adjustment. Positive values mean the servo is increasing consumption.
//
// Register 0x0A — SERVO_FIFO_COUNT_LO (RO)
// Register 0x0B — SERVO_FIFO_COUNT_HI (RO)
//   The exact FIFO count presented to the servo, little-endian. This is a
//   diagnostic comparison against FIFO_FULLNESS, which crosses to mclk.
//
// Register 0x0C — VOLUME (RW)
//   Attenuation in 0.5 dB steps: 0x00 = 0 dB, 0x01 = -0.5 dB, ...,
//   0xFE = -127 dB, and 0xFF = mute. One value controls both channels.
//
// Notes:
//   - This interface uses the open-drain SDA line and does not clock-stretch.
//   - The slave is intentionally a minimal protocol implementation: no
//     auto-increment, no multiple-byte burst reads, and no slave clock stretch.
module registers #(
    parameter integer SCLK_HZ_NOM = 22_579_200
)(
    input  wire        mclk,
    input  wire        rst,
    input  wire        sclk,
    input  wire        prst,
    input  wire        rate_locked,
    input  wire [1:0]  rate_code,
    input  wire        asrc_enable,
    input  wire [15:0] asrc_samp_avail_l,
    input  wire [15:0] asrc_samp_avail_r,
    input  wire        dither_en,
    input  wire [1:0]  mode,
    input  wire        bypass_interp_ctrl,
    input  wire        output_mute_ctrl,
    input  wire        input_mute_ctrl,
    input  wire signed [15:0] servo_error,
    input  wire signed [31:0] servo_step_adj,
    input  wire        [15:0] servo_fifo_count,
    inout  wire        i2c_scl,
    inout  wire        i2c_sda,
    output wire        i2c_reg3_wr,
    output wire [7:0]  i2c_reg3_wdata,
    output reg  [7:0]  volume_target = 8'd0,
    output reg          volume_update_toggle = 1'b0
);

    // 0x00 sample-rate byte: bit7=family (0=44.1k, 1=48k),
    // bits6:0=multiple of the family base rate.
    reg [7:0] i2c_reg0_rate;
    always @(*) begin
        if (!rate_locked)
            i2c_reg0_rate = 8'h00;
        else case (rate_code)
            2'd1:    i2c_reg0_rate = 8'h01;
            2'd2:    i2c_reg0_rate = 8'h81;
            default: i2c_reg0_rate = 8'h00;
        endcase
    end

    // 0x01 is a polling value, so synchronize the changing sclk bus per bit.
    wire [7:0] fifo_fullness_sclk =
        (asrc_samp_avail_l > 16'd255) ? 8'hFF : asrc_samp_avail_l[7:0];
    reg [7:0] fifo_fullness_mclk_r1 = 8'd0;
    reg [7:0] fifo_fullness_mclk    = 8'd0;
    always @(posedge mclk) begin
        fifo_fullness_mclk_r1 <= fifo_fullness_sclk;
        fifo_fullness_mclk    <= fifo_fullness_mclk_r1;
    end

    // 0x02 status is held for about one second after either ASRC rail is hit.
    wire asrc_overrun_event  = asrc_enable &&
        ((asrc_samp_avail_l >= 16'd192) || (asrc_samp_avail_r >= 16'd192));
    wire asrc_underrun_event = asrc_enable &&
        ((asrc_samp_avail_l == 16'd0) || (asrc_samp_avail_r == 16'd0));

    localparam integer STATUS_HOLD_CYCLES = SCLK_HZ_NOM;
    localparam integer STATUS_HOLD_W = $clog2(STATUS_HOLD_CYCLES + 1);
    reg [STATUS_HOLD_W-1:0] overrun_hold  = {STATUS_HOLD_W{1'b0}};
    reg [STATUS_HOLD_W-1:0] underrun_hold = {STATUS_HOLD_W{1'b0}};
    always @(posedge sclk) begin
        if (prst) begin
            overrun_hold  <= {STATUS_HOLD_W{1'b0}};
            underrun_hold <= {STATUS_HOLD_W{1'b0}};
        end else begin
            if (asrc_overrun_event)
                overrun_hold <= STATUS_HOLD_CYCLES[STATUS_HOLD_W-1:0];
            else if (overrun_hold != 0)
                overrun_hold <= overrun_hold - 1'b1;

            if (asrc_underrun_event)
                underrun_hold <= STATUS_HOLD_CYCLES[STATUS_HOLD_W-1:0];
            else if (underrun_hold != 0)
                underrun_hold <= underrun_hold - 1'b1;
        end
    end

    wire overrun_sticky_sclk  = (overrun_hold != 0);
    wire underrun_sticky_sclk = (underrun_hold != 0);
    reg [1:0] overrun_sync  = 2'b0;
    reg [1:0] underrun_sync = 2'b0;
    always @(posedge mclk) begin
        overrun_sync  <= {overrun_sync[0], overrun_sticky_sclk};
        underrun_sync <= {underrun_sync[0], underrun_sticky_sclk};
    end
    wire [7:0] i2c_reg2_status = {6'b0, underrun_sync[1], overrun_sync[1]};

    wire [7:0] i2c_reg3_rdata = {3'b000, input_mute_ctrl, output_mute_ctrl,
                                 bypass_interp_ctrl, (mode == 2'd1), dither_en};
    wire [7:0] i2c_reg4_servo_error_lo = servo_error[7:0];
    wire [7:0] i2c_reg5_servo_error_hi = servo_error[15:8];
    wire [7:0] i2c_reg6_step_adj_b0 = servo_step_adj[7:0];
    wire [7:0] i2c_reg7_step_adj_b1 = servo_step_adj[15:8];
    wire [7:0] i2c_reg8_step_adj_b2 = servo_step_adj[23:16];
    wire [7:0] i2c_reg9_step_adj_b3 = servo_step_adj[31:24];
    wire [7:0] i2c_reg10_servo_fifo_count_lo = servo_fifo_count[7:0];
    wire [7:0] i2c_reg11_servo_fifo_count_hi = servo_fifo_count[15:8];
    wire [7:0] i2c_reg12_wdata;
    wire       i2c_reg12_wr;

    always @(posedge mclk) begin
        if (rst) begin
            volume_target        <= 8'd0;
            volume_update_toggle <= 1'b0;
        end else if (i2c_reg12_wr) begin
            volume_target        <= i2c_reg12_wdata;
            volume_update_toggle <= ~volume_update_toggle;
        end
    end

    wire i2c_sda_oe;
    assign i2c_scl = 1'bz;
    assign i2c_sda = i2c_sda_oe ? 1'b0 : 1'bz;

    i2c_slave #(
        .I2C_ADDR(7'h50)
    ) i2c_inst (
        .clk       (mclk),
        .rst       (rst),
        .scl_in    (i2c_scl),
        .scl_oe    (),
        .sda_in    (i2c_sda),
        .sda_oe    (i2c_sda_oe),
        .reg0_rdata(i2c_reg0_rate),
        .reg1_rdata(fifo_fullness_mclk),
        .reg2_rdata(i2c_reg2_status),
        .reg3_rdata(i2c_reg3_rdata),
        .reg4_rdata(i2c_reg4_servo_error_lo),
        .reg5_rdata(i2c_reg5_servo_error_hi),
        .reg6_rdata(i2c_reg6_step_adj_b0),
        .reg7_rdata(i2c_reg7_step_adj_b1),
        .reg8_rdata(i2c_reg8_step_adj_b2),
        .reg9_rdata(i2c_reg9_step_adj_b3),
        .reg10_rdata(i2c_reg10_servo_fifo_count_lo),
        .reg11_rdata(i2c_reg11_servo_fifo_count_hi),
        .reg12_rdata(volume_target),
        .reg3_wdata(i2c_reg3_wdata),
        .reg3_wr   (i2c_reg3_wr),
        .reg12_wdata(i2c_reg12_wdata),
        .reg12_wr   (i2c_reg12_wr)
    );
endmodule

`default_nettype wire