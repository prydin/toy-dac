`timescale 1ns / 1ps
`default_nettype none

module volume_control #(
    parameter integer RAMP_CYCLES = 11_290,
    parameter         GAIN_FILE   = "volume_gain_q23.mem"
)(
    input  wire                         clk,
    input  wire                         rst,
    input  wire [7:0]                   target_volume_async,
    input  wire                         target_update_toggle_async,
    input  wire signed [31:0]           in_left,
    input  wire signed [31:0]           in_right,
    input  wire                         in_valid_left,
    input  wire                         in_valid_right,
    output wire signed [31:0]           out_left,
    output wire signed [31:0]           out_right,
    output wire                         out_valid_left,
    output wire                         out_valid_right
);

    localparam integer RAMP_COUNT_W =
        (RAMP_CYCLES <= 1) ? 1 : $clog2(RAMP_CYCLES);

    reg [24:0] gain_rom [0:255];
    initial $readmemh(GAIN_FILE, gain_rom);

    reg [7:0] target_meta = 8'd0;
    reg [7:0] target_sync = 8'd0;
    reg       update_meta = 1'b0;
    reg       update_sync = 1'b0;
    reg       update_sync_d = 1'b0;
    reg       update_seen = 1'b0;

    reg [7:0] target_volume = 8'd0;
    reg [7:0] current_volume = 8'd0;
    reg [RAMP_COUNT_W-1:0] ramp_count = {RAMP_COUNT_W{1'b0}};

    always @(posedge clk) begin
        target_meta <= target_volume_async;
        target_sync <= target_meta;
        update_meta <= target_update_toggle_async;
        update_sync <= update_meta;
        update_sync_d <= update_sync;

        if (rst) begin
            update_seen   <= 1'b0;
            target_volume <= 8'd0;
            current_volume <= 8'd0;
            ramp_count    <= {RAMP_COUNT_W{1'b0}};
        end else begin
            if (update_sync_d != update_seen) begin
                update_seen   <= update_sync_d;
                target_volume <= target_sync;
            end

            if (current_volume == target_volume) begin
                ramp_count <= {RAMP_COUNT_W{1'b0}};
            end else if ((RAMP_CYCLES <= 1) ||
                         (ramp_count == RAMP_CYCLES - 1)) begin
                ramp_count <= {RAMP_COUNT_W{1'b0}};
                if (current_volume < target_volume)
                    current_volume <= current_volume + 1'b1;
                else
                    current_volume <= current_volume - 1'b1;
            end else begin
                ramp_count <= ramp_count + 1'b1;
            end
        end
    end

    wire signed [24:0] gain_q23 = $signed(gain_rom[current_volume]);
    reg signed [56:0] product_left = 57'sd0;
    reg signed [56:0] product_right = 57'sd0;
    reg product_valid_left = 1'b0;
    reg product_valid_right = 1'b0;

    always @(posedge clk) begin
        if (rst) begin
            product_left        <= 57'sd0;
            product_right       <= 57'sd0;
            product_valid_left  <= 1'b0;
            product_valid_right <= 1'b0;
        end else begin
            product_valid_left  <= in_valid_left;
            product_valid_right <= in_valid_right;
            if (in_valid_left)
                product_left <= in_left * gain_q23;
            if (in_valid_right)
                product_right <= in_right * gain_q23;
        end
    end

    function signed [31:0] round_q23(input signed [56:0] value);
        reg [57:0] magnitude;
        reg [57:0] rounded_magnitude;
        begin
            magnitude = value[56] ? -value : value;
            rounded_magnitude = magnitude + (58'd1 << 22);
            if (value[56])
                round_q23 = -$signed(rounded_magnitude >> 23);
            else
                round_q23 = $signed(rounded_magnitude >> 23);
        end
    endfunction

    assign out_left        = round_q23(product_left);
    assign out_right       = round_q23(product_right);
    assign out_valid_left  = product_valid_left;
    assign out_valid_right = product_valid_right;

endmodule

`default_nettype wire