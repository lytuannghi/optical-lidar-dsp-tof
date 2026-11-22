// =============================================================================
// File: tof_histogram_engine.v
// Module: tof_histogram_engine
// Project: Automotive Optical LiDAR Time-of-Flight (ToF) DSP ASIC
// Description: 1024-Bin Multi-Peak Time-of-Flight Statistical Histogramming Engine.
//              - Accumulates photon counts to extract weak signals from solar background noise.
//              - Multi-echo extraction: Identifies primary and secondary reflection peaks
//                (e.g. distinguishing glass windshield / rain from solid vehicle obstacles).
// =============================================================================

`timescale 1ns / 1ps

module tof_histogram_engine #(
    parameter BINS = 1024
)(
    input  wire        clk,
    input  wire        rst_n,

    // Histogram Control
    input  wire        hist_clear,
    input  wire        hist_enable,
    input  wire        event_valid,     // Single photon / echo event detected
    input  wire [9:0]  event_bin,       // Coarse bin index (0 to 1023)

    // Multi-Peak Extraction Trigger
    input  wire        find_peaks_start,
    output reg         find_peaks_done,

    // Extracted Targets
    output reg  [9:0]  target1_bin,
    output reg  [15:0] target1_count,
    output reg  [31:0] target1_range_mm,

    output reg  [9:0]  target2_bin,
    output reg  [15:0] target2_count,
    output reg  [31:0] target2_range_mm,

    output reg         target_detected
);

    // 1024-Bin Histogram RAM (16-bit counts per bin)
    reg [15:0] hist_mem [0:BINS-1];
    integer i;

    // Peak search state machine
    reg [1:0]  search_state;
    reg [9:0]  scan_idx;
    localparam ST_IDLE = 2'd0;
    localparam ST_SCAN = 2'd1;
    localparam ST_DONE = 2'd2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            find_peaks_done  <= 1'b0;
            target1_bin      <= 10'd0;
            target1_count    <= 16'd0;
            target1_range_mm <= 32'd0;
            target2_bin      <= 10'd0;
            target2_count    <= 16'd0;
            target2_range_mm <= 32'd0;
            target_detected  <= 1'b0;
            search_state     <= ST_IDLE;
            scan_idx         <= 10'd0;
            for (i = 0; i < BINS; i = i + 1) begin
                hist_mem[i] <= 16'd0;
            end
        end else begin
            find_peaks_done <= 1'b0;

            if (hist_clear) begin
                for (i = 0; i < BINS; i = i + 1) begin
                    hist_mem[i] <= 16'd0;
                end
                target1_count   <= 16'd0;
                target2_count   <= 16'd0;
                target_detected <= 1'b0;
            end else if (hist_enable && event_valid) begin
                // Increment bin count with saturation at 16'hFFFF
                if (hist_mem[event_bin] < 16'hFFFF) begin
                    hist_mem[event_bin] <= hist_mem[event_bin] + 16'd1;
                end
            end

            // Peak Search Pipeline
            case (search_state)
                ST_IDLE: begin
                    if (find_peaks_start) begin
                        scan_idx        <= 10'd0;
                        target1_count   <= 16'd0;
                        target2_count   <= 16'd0;
                        target_detected <= 1'b0;
                        search_state    <= ST_SCAN;
                    end
                end

                ST_SCAN: begin
                    if (hist_mem[scan_idx] > target1_count) begin
                        // New primary peak found; demote old primary to secondary
                        target2_bin      <= target1_bin;
                        target2_count    <= target1_count;
                        target2_range_mm <= target1_range_mm;

                        target1_bin      <= scan_idx;
                        target1_count    <= hist_mem[scan_idx];
                        target1_range_mm <= scan_idx * 32'd600; // 600 mm per bin
                        target_detected  <= 1'b1;
                    end else if (hist_mem[scan_idx] > target2_count && scan_idx != target1_bin) begin
                        // New secondary peak found
                        target2_bin      <= scan_idx;
                        target2_count    <= hist_mem[scan_idx];
                        target2_range_mm <= scan_idx * 32'd600;
                    end

                    if (scan_idx == BINS - 1) begin
                        search_state <= ST_DONE;
                    end else begin
                        scan_idx <= scan_idx + 10'd1;
                    end
                end

                ST_DONE: begin
                    find_peaks_done <= 1'b1;
                    search_state    <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
