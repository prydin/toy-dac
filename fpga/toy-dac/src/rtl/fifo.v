`timescale 1ns / 1ps
`default_nettype none

// Asynchronous (dual-clock) FIFO.
//
// Cummings-style design with binary read/write pointers internally,
// converted to Gray code for safe transfer between clock domains, then
// converted back to binary on the receiving side for fill computation.
//
// Pointers are ADDR_W+1 bits wide; the extra MSB lets us distinguish
// full from empty using a single comparator.
//
// DEPTH must be a power of two (the Gray-code wrap relies on it).
//
// Both wr_count and rd_count are conservative estimates of fill depth
// observed in the *local* domain:
//   - wr_count is up-to-date for writes already accepted, but lags
//     behind reads (it sees the read pointer through synchronisers).
//   - rd_count is up-to-date for reads already issued, but lags behind
//     writes for the same reason.
// For a slow control loop (e.g. soft_pll) sampling rd_count once per
// second, the few-cycle synchroniser lag is irrelevant.

module fifo #(
    parameter integer WIDTH = 32,
    parameter integer DEPTH = 64        // must be a power of two
)(
    // Write domain
    input  wire                        wr_clk,
    input  wire                        wr_rst,
    input  wire signed [WIDTH-1:0]     wr_data,
    input  wire                        wr_en,
    output wire                        full,
    output wire [$clog2(DEPTH+1)-1:0]  wr_count,

    // Read domain
    input  wire                        rd_clk,
    input  wire                        rd_rst,
    output wire signed [WIDTH-1:0]     rd_data,
    input  wire                        rd_en,
    output wire                        empty,
    output wire [$clog2(DEPTH+1)-1:0]  rd_count
);

    localparam integer ADDR_W = $clog2(DEPTH);

    // Compile-time check that DEPTH is a power of two.
    initial begin
        if ((DEPTH & (DEPTH - 1)) != 0) begin
            $display("ERROR: fifo DEPTH (%0d) must be a power of two.", DEPTH);
            $finish;
        end
    end

    // ── Storage ─────────────────────────────────────────────────────────
    // Distributed/block RAM with one write port (wr_clk) and one read
    // port (rd_clk-domain combinational read). Vivado infers this as
    // simple dual-port RAM.
    reg signed [WIDTH-1:0] mem [0:DEPTH-1];

    // ── Pointers (binary) ───────────────────────────────────────────────
    reg [ADDR_W:0] wr_bin = 0;
    reg [ADDR_W:0] rd_bin = 0;

    wire [ADDR_W-1:0] wr_addr = wr_bin[ADDR_W-1:0];
    wire [ADDR_W-1:0] rd_addr = rd_bin[ADDR_W-1:0];

    // ── Binary <-> Gray helpers ────────────────────────────────────────
    function automatic [ADDR_W:0] bin2gray(input [ADDR_W:0] b);
        bin2gray = b ^ (b >> 1);
    endfunction

    function automatic [ADDR_W:0] gray2bin(input [ADDR_W:0] g);
        integer i;
        reg [ADDR_W:0] b;
        begin
            b[ADDR_W] = g[ADDR_W];
            for (i = ADDR_W - 1; i >= 0; i = i - 1)
                b[i] = b[i+1] ^ g[i];
            gray2bin = b;
        end
    endfunction

    // ── Gray-coded snapshots of each pointer (clocked in own domain) ──
    reg [ADDR_W:0] wr_gray = 0;   // updated on wr_clk
    reg [ADDR_W:0] rd_gray = 0;   // updated on rd_clk

    // ── Two-flop synchronisers (cross-domain) ──────────────────────────
    (* ASYNC_REG = "TRUE" *) reg [ADDR_W:0] wr_gray_sync1 = 0; // in rd domain
    (* ASYNC_REG = "TRUE" *) reg [ADDR_W:0] wr_gray_sync2 = 0; // in rd domain
    (* ASYNC_REG = "TRUE" *) reg [ADDR_W:0] rd_gray_sync1 = 0; // in wr domain
    (* ASYNC_REG = "TRUE" *) reg [ADDR_W:0] rd_gray_sync2 = 0; // in wr domain

    always @(posedge rd_clk) begin
        if (rd_rst) begin
            wr_gray_sync1 <= 0;
            wr_gray_sync2 <= 0;
        end else begin
            wr_gray_sync1 <= wr_gray;
            wr_gray_sync2 <= wr_gray_sync1;
        end
    end

    always @(posedge wr_clk) begin
        if (wr_rst) begin
            rd_gray_sync1 <= 0;
            rd_gray_sync2 <= 0;
        end else begin
            rd_gray_sync1 <= rd_gray;
            rd_gray_sync2 <= rd_gray_sync1;
        end
    end

    // ── Write side ─────────────────────────────────────────────────────
    wire [ADDR_W:0] rd_bin_in_wr = gray2bin(rd_gray_sync2);

    // full when wr_bin is one full lap ahead of the synced rd_bin
    wire [ADDR_W:0] wr_bin_next = wr_bin + 1;
    assign full = ( wr_bin[ADDR_W]     != rd_bin_in_wr[ADDR_W]    ) &&
                  ( wr_bin[ADDR_W-1:0] == rd_bin_in_wr[ADDR_W-1:0] );

    assign wr_count = wr_bin - rd_bin_in_wr;

    always @(posedge wr_clk) begin
        if (wr_rst) begin
            wr_bin  <= 0;
            wr_gray <= 0;
        end else if (wr_en && !full) begin
            mem[wr_addr] <= wr_data;
            wr_bin       <= wr_bin_next;
            wr_gray      <= bin2gray(wr_bin_next);
        end
    end

    // ── Read side ──────────────────────────────────────────────────────
    wire [ADDR_W:0] wr_bin_in_rd = gray2bin(wr_gray_sync2);

    assign empty   = (rd_bin == wr_bin_in_rd);
    assign rd_data = mem[rd_addr];
    assign rd_count = wr_bin_in_rd - rd_bin;

    wire [ADDR_W:0] rd_bin_next = rd_bin + 1;

    always @(posedge rd_clk) begin
        if (rd_rst) begin
            rd_bin  <= 0;
            rd_gray <= 0;
        end else if (rd_en && !empty) begin
            rd_bin  <= rd_bin_next;
            rd_gray <= bin2gray(rd_bin_next);
        end
    end

endmodule

`default_nettype wire
