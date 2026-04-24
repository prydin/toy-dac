module rate_detect #(
    parameter WINDOW_SIZE = 256 // Number of clock cycles to measure between lrclk edges. But be power of 2.
)(
    input wire clk,
    input wire rst,
    input wire lrclk_pos_edge,
    input wire dvalid,
    output reg rate_valid = 0,
    output reg [31:0] window_period = 0,
    output reg [15:0] period = 0
);

reg [$clog2(WINDOW_SIZE)-1:0] edge_counter = 0; // Counts clock cycles between lrclk edges, wraps around at WINDOW_SIZE
reg [31:0] period_counter = 0;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        period_counter <= 0;
        window_period <= 0;
        period <= 0;
        edge_counter <= 0;
        rate_valid <= 0;
    end else begin
        // Default: deassert rate_valid unless this cycle closes a window
        rate_valid <= 0;

        if (lrclk_pos_edge) begin
            if (edge_counter == WINDOW_SIZE - 1) begin
                // Reached WINDOW_SIZE edges: total clk count for the
                // window is in period_counter. Average period per lrclk
                // is period_counter / WINDOW_SIZE; since WINDOW_SIZE is
                // a power of two, divide by right-shift.
                window_period <= period_counter;
                period        <= period_counter >> $clog2(WINDOW_SIZE);
                edge_counter  <= 0;
                period_counter <= 0;
                rate_valid    <= 1; // New period value valid this cycle
            end else begin
                // Mid-window edge: count it and keep accumulating clks
                edge_counter   <= edge_counter + 1;
                period_counter <= period_counter + 1;
            end
        end else begin
            period_counter <= period_counter + 1;
        end
    end
end

endmodule