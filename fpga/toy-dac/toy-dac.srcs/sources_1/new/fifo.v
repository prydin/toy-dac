`timescale 1ns / 1ps
`default_nettype none

module fifo #(
    parameter WIDTH = 32,
    parameter DEPTH = 64
)(
    input  wire             clk,
    input  wire             rst,

    // Write interface
    input  wire signed [WIDTH-1:0] wr_data,
    input  wire             wr_en,
    output wire             full,

    // Read interface
    output wire signed [WIDTH-1:0] rd_data,
    input  wire             rd_en,
    output wire             empty,

    // Status
    output wire [$clog2(DEPTH+1)-1:0] count
);

    localparam ADDR_W = $clog2(DEPTH);

    reg signed [WIDTH-1:0] mem [0:DEPTH-1];

    reg [ADDR_W:0] wr_ptr = 0;
    reg [ADDR_W:0] rd_ptr = 0;

    wire [ADDR_W-1:0] wr_addr = wr_ptr[ADDR_W-1:0];
    wire [ADDR_W-1:0] rd_addr = rd_ptr[ADDR_W-1:0];

    assign full  = (wr_ptr[ADDR_W] != rd_ptr[ADDR_W]) &&
                   (wr_ptr[ADDR_W-1:0] == rd_ptr[ADDR_W-1:0]);
    assign empty = (wr_ptr == rd_ptr);
    assign count = wr_ptr - rd_ptr;

    assign rd_data = mem[rd_addr];

    always @(posedge clk) begin
        if (rst) begin
            wr_ptr <= 0;
            rd_ptr <= 0;
        end else begin
            if (wr_en && !full) begin
                mem[wr_addr] <= wr_data;
                wr_ptr <= wr_ptr + 1;
            end
            if (rd_en && !empty) begin
                rd_ptr <= rd_ptr + 1;
            end
        end
    end

endmodule

`default_nettype wire
