// =============================================================================
// File: constant_fraction_discriminator.v
// Module: constant_fraction_discriminator
// Project: Automotive Optical LiDAR Time-of-Flight (ToF) DSP ASIC
// Description: Digital Constant Fraction Discriminator (dCFD) Engine.
//              Eliminates amplitude-dependent timing jitter (time-walk error)
//              by detecting the zero-crossing of the bipolar CFD waveform:
//                V_cfd[n] = f * S[n] - S[n - D]
// =============================================================================

`timescale 1ns / 1ps

module constant_fraction_discriminator (
    input  wire                 clk,
    input  wire                 rst_n,

    // Digitized Echo Input from High-Speed APD/SPAD ADC
    input  wire                 sample_valid,
    input  wire [11:0]          adc_sample_in, // 12-bit unsigned ADC data (0 - 4095)

    // Configuration
    input  wire [11:0]          arm_threshold, // Minimum pulse amplitude required to arm CFD
    input  wire [1:0]           fraction_sel,  // 0: 1/8, 1: 1/4, 2: 1/2, 3: 3/4
    input  wire [3:0]           delay_taps,    // Delay line depth D (1-8 cycles)

    // Echo Detection Outputs
    output reg                  echo_detected, // High-precision pulse arrival trigger
    output reg  [11:0]          echo_intensity // Peak amplitude of detected pulse
);

    // Circular delay line for S[n - D]
    reg [11:0] delay_buf [0:15];
    integer i;

    // Attenuated fraction signal: f * S[n]
    reg [11:0] frac_sample;
    always @(*) begin
        case (fraction_sel)
            2'b00: frac_sample = adc_sample_in >> 3; // 1/8
            2'b01: frac_sample = adc_sample_in >> 2; // 1/4
            2'b10: frac_sample = adc_sample_in >> 1; // 1/2
            2'b11: frac_sample = (adc_sample_in >> 1) + (adc_sample_in >> 2); // 3/4
        endcase
    end

    // CFD Bipolar difference waveform: V_cfd[n] = f*S[n] - S[n-D]
    reg signed [12:0] v_cfd;
    reg signed [12:0] v_cfd_prev;
    reg               cfd_armed;
    reg [11:0]        peak_hold;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            echo_detected  <= 1'b0;
            echo_intensity <= 12'd0;
            v_cfd          <= 13'sd0;
            v_cfd_prev     <= 13'sd0;
            cfd_armed      <= 1'b0;
            peak_hold      <= 12'd0;
            for (i = 0; i < 16; i = i + 1) begin
                delay_buf[i] <= 12'd0;
            end
        end else if (sample_valid) begin
            echo_detected <= 1'b0;

            // Shift delay line
            delay_buf[0] <= adc_sample_in;
            for (i = 1; i < 16; i = i + 1) begin
                delay_buf[i] <= delay_buf[i-1];
            end

            // Compute bipolar CFD signal
            v_cfd_prev <= v_cfd;
            v_cfd      <= $signed({1'b0, frac_sample}) - $signed({1'b0, delay_buf[delay_taps]});

            // Arming comparator: Pulse must exceed arm_threshold to avoid noise false triggers
            if (adc_sample_in >= arm_threshold) begin
                cfd_armed <= 1'b1;
                if (adc_sample_in > peak_hold)
                    peak_hold <= adc_sample_in;
            end

            // Zero-Crossing Detection: V_cfd transitions from positive to negative
            if (cfd_armed && (v_cfd_prev > 13'sd0) && (v_cfd <= 13'sd0)) begin
                echo_detected  <= 1'b1;
                echo_intensity <= peak_hold;
                cfd_armed      <= 1'b0;
                peak_hold      <= 12'd0;
            end

            // Reset arming if pulse falls below threshold for consecutive cycles
            if (adc_sample_in < (arm_threshold >> 2) && !echo_detected) begin
                cfd_armed <= 1'b0;
                peak_hold <= 12'd0;
            end
        end
    end

endmodule
