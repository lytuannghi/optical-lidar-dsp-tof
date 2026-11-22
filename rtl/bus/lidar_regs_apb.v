// =============================================================================
// File: lidar_regs_apb.v
// Module: lidar_regs_apb
// Project: Automotive Optical LiDAR Time-of-Flight (ToF) DSP ASIC
// Description: AMBA 3 APB Slave Configuration, Status, and Telemetry Registers.
// =============================================================================

`timescale 1ns / 1ps

module lidar_regs_apb (
    input  wire        pclk,
    input  wire        presetn,

    // APB3 Interface
    input  wire [7:0]  paddr,
    input  wire        psel,
    input  wire        penable,
    input  wire        pwrite,
    input  wire [31:0] pwdata,
    output reg  [31:0] prdata,
    output wire        pready,
    output wire        pslverr,

    // Control Outputs to LiDAR Engine
    output reg         reg_laser_enable,
    output reg         reg_hist_clear,
    output reg         reg_find_peaks_start,
    output reg  [15:0] reg_pulse_period,
    output reg  [7:0]  reg_pulse_width,
    output reg  [11:0] reg_arm_threshold,
    output reg  [1:0]  reg_fraction_sel,
    output reg  [3:0]  reg_delay_taps,

    // Telemetry from LiDAR Engine
    input  wire        status_laser_active,
    input  wire        status_eye_safety_err,
    input  wire        status_target_detected,
    input  wire        status_peaks_done,
    input  wire [31:0] target1_range_mm,
    input  wire [15:0] target1_count,
    input  wire [31:0] target2_range_mm,
    input  wire [15:0] target2_count,
    input  wire [31:0] raw_tdc_range_mm
);

    assign pready  = 1'b1;
    assign pslverr = 1'b0;

    always @(posedge pclk or negedge presetn) begin
        if (!presetn) begin
            reg_laser_enable     <= 1'b0;
            reg_hist_clear       <= 1'b0;
            reg_find_peaks_start <= 1'b0;
            reg_pulse_period     <= 16'd2500; // 100 kHz PRF default
            reg_pulse_width      <= 8'd5;     // 20 ns pulse default
            reg_arm_threshold    <= 12'd100;  // Threshold
            reg_fraction_sel     <= 2'b01;    // 1/4 fraction
            reg_delay_taps       <= 4'd4;     // 4 cycles delay
            prdata               <= 32'd0;
        end else begin
            reg_hist_clear       <= 1'b0; // Auto-clearing strobe
            reg_find_peaks_start <= 1'b0;

            // APB Write Operation
            if (psel && penable && pwrite) begin
                case (paddr)
                    8'h00: begin
                        reg_laser_enable     <= pwdata[0];
                        reg_hist_clear       <= pwdata[2];
                        reg_find_peaks_start <= pwdata[3];
                    end
                    8'h08: begin
                        reg_pulse_period <= pwdata[15:0];
                        reg_pulse_width  <= pwdata[23:16];
                    end
                    8'h0C: begin
                        reg_arm_threshold <= pwdata[11:0];
                        reg_fraction_sel  <= pwdata[13:12];
                        reg_delay_taps    <= pwdata[17:14];
                    end
                    default: ;
                endcase
            end

            // APB Read Operation
            if (psel && !pwrite) begin
                case (paddr)
                    8'h00: prdata <= {28'd0, reg_laser_enable};
                    8'h04: prdata <= {28'd0, status_peaks_done, status_target_detected, status_eye_safety_err, status_laser_active};
                    8'h08: prdata <= {8'd0, reg_pulse_width, reg_pulse_period};
                    8'h0C: prdata <= {14'd0, reg_delay_taps, reg_fraction_sel, reg_arm_threshold};
                    8'h10: prdata <= target1_range_mm;
                    8'h14: prdata <= {16'd0, target1_count};
                    8'h18: prdata <= target2_range_mm;
                    8'h1C: prdata <= {16'd0, target2_count};
                    8'h20: prdata <= raw_tdc_range_mm;
                    default: prdata <= 32'd0;
                endcase
            end
        end
    end

endmodule
