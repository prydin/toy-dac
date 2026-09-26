`timescale 1ns / 1ps
`default_nettype none

// i2c_slave
// ─────────
// Minimal I2C-bus target: 7-bit address, single-byte register
// pointer, single-byte read/write data, no auto-increment (a
// multi-byte read/write just re-reads/re-writes the same register).
// Only registers 0x00–0x03 exist; 0x00-0x02 are read-only, 0x03 is
// read/write. Never stretches the clock (scl_oe is left tied off).
//
// scl/sda are open-drain: `*_oe` = 1 drives the line low, 0 releases
// it (external pull-ups, or the bus master, hold it high).

module i2c_slave #(
    parameter [6:0] I2C_ADDR = 7'h50
)(
    input  wire       clk,
    input  wire       rst,

    input  wire        scl_in,  
    output wire        scl_oe,     // unused: this slave never clock-stretches
    input  wire        sda_in,
    output wire        sda_oe,

    input  wire [7:0]  reg0_rdata,  // 0x00 sample rate      (RO)
    input  wire [7:0]  reg1_rdata,  // 0x01 FIFO fullness    (RO)
    input  wire [7:0]  reg2_rdata,  // 0x02 over/underrun    (RO)
    input  wire [7:0]  reg3_rdata,  // 0x03 control readback
    input  wire [7:0]  reg4_rdata,  // 0x04 servo error low byte
    input  wire [7:0]  reg5_rdata,  // 0x05 servo error high byte
    input  wire [7:0]  reg6_rdata,  // 0x06 step adjustment byte 0
    input  wire [7:0]  reg7_rdata,  // 0x07 step adjustment byte 1
    input  wire [7:0]  reg8_rdata,  // 0x08 step adjustment byte 2
    input  wire [7:0]  reg9_rdata,  // 0x09 step adjustment byte 3
    input  wire [7:0]  reg10_rdata, // 0x0A servo FIFO count low byte
    input  wire [7:0]  reg11_rdata, // 0x0B servo FIFO count high byte
    input  wire [7:0]  reg12_rdata, // 0x0C volume attenuation (RW)
    output reg  [7:0]  reg3_wdata = 8'd0,
    output reg         reg3_wr    = 1'b0,  // 1-cycle pulse on write to 0x03
    output reg  [7:0]  reg12_wdata = 8'd0,
    output reg         reg12_wr    = 1'b0  // 1-cycle pulse on write to 0x0C
);

    localparam S_IDLE       = 4'd0;
    localparam S_ADDR       = 4'd1;
    localparam S_ADDR_ACK   = 4'd2;
    localparam S_REGPTR     = 4'd3;
    localparam S_REGPTR_ACK = 4'd4;
    localparam S_WR         = 4'd5;
    localparam S_WR_ACK     = 4'd6;
    localparam S_RD         = 4'd7;
    localparam S_RD_ACK     = 4'd8;

    reg [3:0] state   = S_IDLE;
    reg [3:0] bit_cnt  = 4'd0;
    reg [7:0] shreg_in  = 8'd0;
    reg [7:0] shreg_out = 8'd0;
    reg [7:0] reg_ptr   = 8'd0;
    reg       ack_high_seen = 1'b0;

    // 3-deep synchronizer: [1] is the metastability-safe "now" sample,
    // [2] is the previous cycle's sample, used for edge/condition detect.
    reg [2:0] scl_ff = 3'b111;
    reg [2:0] sda_ff = 3'b111;
    always @(posedge clk) begin
        scl_ff <= {scl_ff[1:0], scl_in};
        sda_ff <= {sda_ff[1:0], sda_in};
    end
    wire scl_now  = scl_ff[1];
    wire scl_prev = scl_ff[2];
    wire sda_now  = sda_ff[1];
    wire sda_prev = sda_ff[2];
    wire scl_rise = scl_now & ~scl_prev;
    wire scl_fall = ~scl_now & scl_prev;
    wire start_cond = scl_now & scl_prev & sda_prev & ~sda_now;
    wire stop_cond  = scl_now & scl_prev & ~sda_prev & sda_now;

    reg sda_drive = 1'b0;
    assign sda_oe = sda_drive;
    assign scl_oe = 1'b0;

    function [7:0] reg_rdata_mux(input [7:0] ptr);
        case (ptr)
            8'd0:    reg_rdata_mux = reg0_rdata;
            8'd1:    reg_rdata_mux = reg1_rdata;
            8'd2:    reg_rdata_mux = reg2_rdata;
            8'd3:    reg_rdata_mux = reg3_rdata;
            8'd4:    reg_rdata_mux = reg4_rdata;
            8'd5:    reg_rdata_mux = reg5_rdata;
            8'd6:    reg_rdata_mux = reg6_rdata;
            8'd7:    reg_rdata_mux = reg7_rdata;
            8'd8:    reg_rdata_mux = reg8_rdata;
            8'd9:    reg_rdata_mux = reg9_rdata;
            8'd10:   reg_rdata_mux = reg10_rdata;
            8'd11:   reg_rdata_mux = reg11_rdata;
            8'd12:   reg_rdata_mux = reg12_rdata;
            default: reg_rdata_mux = 8'hFF;
        endcase
    endfunction

    wire reg_ptr_writable = (reg_ptr == 8'd3) || (reg_ptr == 8'd12);
    wire [7:0] reg_ptr_rdata = reg_rdata_mux(reg_ptr);

    always @(posedge clk) begin
        reg3_wr  <= 1'b0;
        reg12_wr <= 1'b0;
        if (rst) begin
            state     <= S_IDLE;
            bit_cnt   <= 4'd0;
            sda_drive <= 1'b0;
            reg_ptr   <= 8'd0;
            ack_high_seen <= 1'b0;
        end else if (stop_cond) begin
            state     <= S_IDLE;
            sda_drive <= 1'b0;
        end else if (start_cond) begin
            state     <= S_ADDR;
            bit_cnt   <= 4'd0;
            sda_drive <= 1'b0;
        end else begin
            case (state)
                S_IDLE: sda_drive <= 1'b0;

                S_ADDR: if (scl_rise) begin
                    shreg_in <= {shreg_in[6:0], sda_now};
                    if (bit_cnt == 4'd7) begin
                        state   <= S_ADDR_ACK;
                        bit_cnt <= 4'd0;
                    end else begin
                        bit_cnt <= bit_cnt + 1'b1;
                    end
                end

                S_ADDR_ACK: begin
                    if (scl_fall) begin
                        if (ack_high_seen) begin
                            // Release after the ACK high phase, while SCL
                            // is low, so this cannot look like a STOP.
                            sda_drive    <= 1'b0;
                            ack_high_seen <= 1'b0;
                            bit_cnt      <= 4'd0;
                            if (shreg_in[7:1] != I2C_ADDR) begin
                                state <= S_IDLE;
                            end else if (shreg_in[0]) begin
                                // Read with no new pointer this transaction.
                                // Drive the first bit now: the per-bit update
                                // in S_RD only fires on the *next* scl_fall.
                                shreg_out <= reg_rdata_mux(reg_ptr);
                                sda_drive <= ~reg_ptr_rdata[7];
                                state     <= S_RD;
                            end else begin
                                state <= S_REGPTR;
                            end
                        end else begin
                            sda_drive <= (shreg_in[7:1] == I2C_ADDR);
                        end
                    end
                    if (scl_rise)
                        ack_high_seen <= 1'b1;
                end

                S_REGPTR: if (scl_rise) begin
                    shreg_in <= {shreg_in[6:0], sda_now};
                    if (bit_cnt == 4'd7) begin
                        state   <= S_REGPTR_ACK;
                        bit_cnt <= 4'd0;
                    end else begin
                        bit_cnt <= bit_cnt + 1'b1;
                    end
                end

                S_REGPTR_ACK: begin
                    if (scl_fall) begin
                        if (ack_high_seen) begin
                            sda_drive     <= 1'b0;
                            ack_high_seen <= 1'b0;
                            reg_ptr       <= shreg_in;
                            bit_cnt       <= 4'd0;
                            state         <= (shreg_in > 8'd12) ? S_IDLE : S_WR;
                        end else begin
                            sda_drive <= (shreg_in <= 8'd12);
                        end
                    end
                    if (scl_rise)
                        ack_high_seen <= 1'b1;
                end

                // Waiting for a data byte to write. A repeated START
                // (handled above, overrides `state` unconditionally)
                // is how a write-pointer-then-read sequence leaves
                // this state without ever sending a data byte.
                S_WR: if (scl_rise) begin
                    shreg_in <= {shreg_in[6:0], sda_now};
                    if (bit_cnt == 4'd7) begin
                        state   <= S_WR_ACK;
                        bit_cnt <= 4'd0;
                    end else begin
                        bit_cnt <= bit_cnt + 1'b1;
                    end
                end

                S_WR_ACK: begin
                    if (scl_fall) begin
                        if (ack_high_seen) begin
                            sda_drive     <= 1'b0;
                            ack_high_seen <= 1'b0;
                            if (reg_ptr_writable) begin
                                if (reg_ptr == 8'd3) begin
                                    reg3_wdata <= shreg_in;
                                    reg3_wr    <= 1'b1;
                                end else begin
                                    reg12_wdata <= shreg_in;
                                    reg12_wr    <= 1'b1;
                                end
                            end
                            bit_cnt <= 4'd0;
                            state   <= S_WR;
                        end else begin
                            sda_drive <= reg_ptr_writable;
                        end
                    end
                    if (scl_rise)
                        ack_high_seen <= 1'b1;
                end

                S_RD: begin
                    if (scl_fall)
                        sda_drive <= ~shreg_out[7];
                    if (scl_rise) begin
                        shreg_out <= {shreg_out[6:0], 1'b0};
                        if (bit_cnt == 4'd7) begin
                            state   <= S_RD_ACK;
                            bit_cnt <= 4'd0;
                        end else begin
                            bit_cnt <= bit_cnt + 1'b1;
                        end
                    end
                end

                S_RD_ACK: begin
                    if (scl_fall)
                        sda_drive <= 1'b0;   // release; master drives ack/nack
                    if (scl_rise) begin
                        bit_cnt <= 4'd0;
                        if (sda_now) begin
                            state <= S_IDLE;         // NACK: master is done
                        end else begin
                            shreg_out <= reg_rdata_mux(reg_ptr); // no
                                // auto-increment: re-send same register
                            state <= S_RD;
                        end
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
