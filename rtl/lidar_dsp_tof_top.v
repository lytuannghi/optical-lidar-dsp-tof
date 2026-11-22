// =============================================================================
// File: lidar_dsp_tof_top.v
// Module: lidar_dsp_tof_top
// Project: Automotive Optical LiDAR Time-of-Flight (ToF) DSP ASIC
// Description: Full-Flow Synthesizable Top-Level ASIC Automotive LiDAR Subsystem Core.
//              Integrates:
//              - Laser Firing Timing Engine with Eye-Safety Interlock
//              - Digital Constant Fraction Discriminator (dCFD) Time-Walk Compensator
//              - Hybrid Coarse-Fine Time-to-Digital Converter (62.5 ps / 9.375 mm)
//              - 1024-Bin Multi-Peak Statistical Histogramming Engine
//              - AMBA 3 APB Slave Register Interface
// =============================================================================

`timescale 1ns / 1ps

module lidar_dsp_tof_top (
    input  wire        clk,             // 250 MHz Master Clock
    input  wire        rst_n,

    // Optical Interface
    output wire        laser_trigger,   // To Laser Diode Driver (GaN FET)
    input  wire        adc_sample_valid,// High-Speed ADC Sample Strobe
    input  wire [11:0] adc_sample_in,   // Digitized Photodiode Signal (0-4095)
    input  wire [5:0]  vernier_phase,   // Sub-Clock Delay Line Phase (64 bins)

    // AMBA 3 APB Slave Bus
    input  wire [7:0]  paddr,
    input  wire        psel,
    input  wire        penable,
    input  wire        pwrite,
    input  wire [31:0] pwdata,
    output wire [31:0] prdata,
    output wire        pready,
    output wire        pslverr,

    // Interrupt & Status Flags
    output wire        irq_target_detected,
    output wire        status_interlock_shutdown
);

    // =========================================================================
    // APB3 Control Lines
    // =========================================================================
    wire        reg_laser_enable;
    wire        reg_hist_clear;
    wire        reg_find_peaks_start;
    wire [15:0] reg_pulse_period;
    wire [7:0]  reg_pulse_width;
    wire [11:0] reg_arm_threshold;
    wire [1:0]  reg_fraction_sel;
    wire [3:0]  reg_delay_taps;

    wire        eye_safety_err;
    wire        target_detected;
    wire        find_peaks_done;
    wire [31:0] target1_range_mm;
    wire [15:0] target1_count;
    wire [31:0] target2_range_mm;
    wire [15:0] target2_count;
    wire [31:0] raw_tdc_range_mm;

    lidar_regs_apb u_regs (
        .pclk                  (clk),
        .presetn               (rst_n),
        .paddr                 (paddr),
        .psel                  (psel),
        .penable               (penable),
        .pwrite                (pwrite),
        .pwdata                (pwdata),
        .prdata                (prdata),
        .pready                (pready),
        .pslverr               (pslverr),
        .reg_laser_enable      (reg_laser_enable),
        .reg_hist_clear        (reg_hist_clear),
        .reg_find_peaks_start  (reg_find_peaks_start),
        .reg_pulse_period      (reg_pulse_period),
        .reg_pulse_width       (reg_pulse_width),
        .reg_arm_threshold     (reg_arm_threshold),
        .reg_fraction_sel      (reg_fraction_sel),
        .reg_delay_taps        (reg_delay_taps),
        .status_laser_active   (laser_trigger),
        .status_eye_safety_err (eye_safety_err),
        .status_target_detected(target_detected),
        .status_peaks_done     (find_peaks_done),
        .target1_range_mm      (target1_range_mm),
        .target1_count         (target1_count),
        .target2_range_mm      (target2_range_mm),
        .target2_count         (target2_count),
        .raw_tdc_range_mm      (raw_tdc_range_mm)
    );

    // =========================================================================
    // Laser Timing Generator with Interlock Watchdog
    // =========================================================================
    wire tx_start_strobe;

    laser_pulse_generator u_laser_gen (
        .clk             (clk),
        .rst_n           (rst_n),
        .laser_enable    (reg_laser_enable),
        .pulse_period    (reg_pulse_period),
        .pulse_width     (reg_pulse_width),
        .laser_fire_pulse(laser_trigger),
        .tx_start_strobe (tx_start_strobe),
        .eye_safety_err  (eye_safety_err)
    );

    // =========================================================================
    // Digital Constant Fraction Discriminator (dCFD)
    // =========================================================================
    wire        cfd_echo_detected;
    wire [11:0] cfd_echo_intensity;

    constant_fraction_discriminator u_cfd (
        .clk           (clk),
        .rst_n         (rst_n),
        .sample_valid  (adc_sample_valid),
        .adc_sample_in (adc_sample_in),
        .arm_threshold (reg_arm_threshold),
        .fraction_sel  (reg_fraction_sel),
        .delay_taps    (reg_delay_taps),
        .echo_detected (cfd_echo_detected),
        .echo_intensity(cfd_echo_intensity)
    );

    // =========================================================================
    // Time-to-Digital Converter (TDC)
    // =========================================================================
    wire        tdc_tof_valid;
    wire [21:0] tdc_timestamp;

    time_to_digital_converter u_tdc (
        .clk          (clk),
        .rst_n        (rst_n),
        .tx_start     (tx_start_strobe),
        .rx_stop      (cfd_echo_detected),
        .fine_phase_in(vernier_phase),
        .tof_valid    (tdc_tof_valid),
        .tof_timestamp(tdc_timestamp),
        .range_mm     (raw_tdc_range_mm)
    );

    // =========================================================================
    // 1024-Bin Multi-Peak Histogramming Engine
    // =========================================================================
    tof_histogram_engine #(.BINS(1024)) u_histogram (
        .clk             (clk),
        .rst_n           (rst_n),
        .hist_clear      (reg_hist_clear),
        .hist_enable     (reg_laser_enable),
        .event_valid     (tdc_tof_valid),
        .event_bin       (tdc_timestamp[15:6]), // Map coarse cycles to 1024 bins
        .find_peaks_start(reg_find_peaks_start),
        .find_peaks_done (find_peaks_done),
        .target1_bin     (),
        .target1_count   (target1_count),
        .target1_range_mm(target1_range_mm),
        .target2_bin     (),
        .target2_count   (target2_count),
        .target2_range_mm(target2_range_mm),
        .target_detected (target_detected)
    );

    assign irq_target_detected       = target_detected;
    assign status_interlock_shutdown = eye_safety_err;

endmodule
