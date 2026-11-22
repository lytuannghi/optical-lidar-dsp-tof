// =============================================================================
// File: time_to_digital_converter.v
// Module: time_to_digital_converter
// Project: Automotive Optical LiDAR Time-of-Flight (ToF) DSP ASIC
// Description: Hybrid Coarse-Fine Time-to-Digital Converter (TDC).
//              - Coarse Counter: 16-bit counter @ 250 MHz (4.0 ns / 600 mm resolution)
//              - Fine Interpolator: 6-bit Vernier interpolator (62.5 ps / 9.375 mm bin)
//              - Total Time-of-Flight Output: 22-bit fixed-point timestamp (Coarse + Fine)
// =============================================================================

`timescale 1ns / 1ps

module time_to_digital_converter (
    input  wire        clk,            // 250 MHz System Clock
    input  wire        rst_n,

    // Timing Triggers
    input  wire        tx_start,       // Laser fire pulse (t0 origin)
    input  wire        rx_stop,        // CFD echo detected pulse (t_stop)
    input  wire [5:0]  fine_phase_in,  // From analog Vernier delay line / DLL (6-bit fine phase)

    // Timestamp Outputs
    output reg         tof_valid,      // Valid measurement ready
    output reg  [21:0] tof_timestamp,  // [21:6] Coarse cycles (4ns), [5:0] Fine phase (62.5ps)
    output reg  [31:0] range_mm        // Calculated distance in millimeters (0 - 300,000 mm)
);

    reg [15:0] coarse_cnt;
    reg        measuring;

    // Conversion factor: 1 coarse count (4ns) = 599.58 mm round-trip distance
    // Fixed-point scaling: Range_mm = (Coarse * 600) + (Fine * 600 / 64)
    wire [31:0] coarse_mm = coarse_cnt * 32'd600;
    wire [31:0] fine_mm   = (fine_phase_in * 32'd600) >> 6;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            coarse_cnt    <= 16'd0;
            measuring     <= 1'b0;
            tof_valid     <= 1'b0;
            tof_timestamp <= 22'd0;
            range_mm      <= 32'd0;
        end else begin
            tof_valid <= 1'b0;

            if (tx_start) begin
                // Reset and launch coarse counter at laser emission
                coarse_cnt <= 16'd0;
                measuring  <= 1'b1;
            end else if (measuring) begin
                if (rx_stop) begin
                    // Latch arrival timestamp
                    measuring     <= 1'b0;
                    tof_valid     <= 1'b1;
                    tof_timestamp <= {coarse_cnt, fine_phase_in};
                    range_mm      <= coarse_mm + fine_mm;
                end else if (coarse_cnt == 16'hFFFF) begin
                    // Range timeout (over ~39 km)
                    measuring <= 1'b0;
                end else begin
                    coarse_cnt <= coarse_cnt + 16'd1;
                end
            end
        end
    end

endmodule
