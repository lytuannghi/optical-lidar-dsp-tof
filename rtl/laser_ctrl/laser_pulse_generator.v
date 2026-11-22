// =============================================================================
// File: laser_pulse_generator.v
// Module: laser_pulse_generator
// Project: Automotive Optical LiDAR Time-of-Flight (ToF) DSP ASIC
// Description: Programmable Laser Transmitter Timing Engine with Eye-Safety Interlock.
//              - Generates high-voltage laser fire triggers with programmable PRF.
//              - Hardware watchdog shuts down laser if duty cycle exceeds safe optical threshold.
// =============================================================================

`timescale 1ns / 1ps

module laser_pulse_generator (
    input  wire        clk,             // Master System Clock (250 MHz)
    input  wire        rst_n,

    // Control & Configuration
    input  wire        laser_enable,
    input  wire [15:0] pulse_period,    // Pulse Repetition Interval in clock cycles (e.g. 2500 -> 100 kHz PRF)
    input  wire [7:0]  pulse_width,     // Laser optical pulse duration in cycles (e.g. 5 -> 20 ns)

    // Hardware Interlock
    output reg         laser_fire_pulse,// Output trigger to Laser Diode Driver (GaN FET)
    output reg         tx_start_strobe, // To TDC Coarse Counter: t0 timing origin
    output reg         eye_safety_err   // Alert: Excessive duty cycle detected
);

    reg [15:0] period_cnt;
    reg [7:0]  width_cnt;
    reg [15:0] pulse_high_accum; // Rolling pulse active counter
    reg [15:0] window_cnt;

    // Maximum safe optical duty cycle: 0.1% (1000 ppm)
    localparam [15:0] MAX_SAFE_HIGH_CYCLES = 16'd50;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            laser_fire_pulse <= 1'b0;
            tx_start_strobe  <= 1'b0;
            eye_safety_err   <= 1'b0;
            period_cnt       <= 16'd0;
            width_cnt        <= 8'd0;
            pulse_high_accum <= 16'd0;
            window_cnt       <= 16'd0;
        end else begin
            tx_start_strobe <= 1'b0;

            // Rolling eye-safety window of 50,000 cycles
            if (window_cnt >= 16'd50000) begin
                window_cnt       <= 16'd0;
                pulse_high_accum <= 16'd0;
                if (pulse_high_accum > MAX_SAFE_HIGH_CYCLES) begin
                    eye_safety_err <= 1'b1; // Latch interlock shutdown
                end
            end else begin
                window_cnt <= window_cnt + 16'd1;
                if (laser_fire_pulse)
                    pulse_high_accum <= pulse_high_accum + 16'd1;
            end

            if (laser_enable && !eye_safety_err) begin
                if (period_cnt == 16'd0) begin
                    // Trigger laser fire
                    laser_fire_pulse <= 1'b1;
                    tx_start_strobe  <= 1'b1;
                    width_cnt        <= pulse_width;
                    period_cnt       <= (pulse_period > 16'd100) ? pulse_period : 16'd2500;
                end else begin
                    period_cnt <= period_cnt - 16'd1;

                    if (width_cnt > 8'd1) begin
                        width_cnt        <= width_cnt - 8'd1;
                        laser_fire_pulse <= 1'b1;
                    end else begin
                        laser_fire_pulse <= 1'b0;
                    end
                end
            end else begin
                laser_fire_pulse <= 1'b0;
                period_cnt       <= 16'd0;
            end
        end
    end

endmodule
