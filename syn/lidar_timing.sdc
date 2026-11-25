# ==============================================================================
# Synopsys Design Constraints (SDC)
# Design: lidar_dsp_tof_top
# Operating Frequency: 250 MHz (Clock Period = 4.000 ns)
# Process: SkyWater 130nm HD Standard Cell Library (1.80V, 25C)
# ==============================================================================

# Primary Master Clock Definition (250 MHz)
create_clock -name clk -period 4.000 -waveform {0.000 2.000} [get_ports clk]

# Clock Uncertainty and Jitter Constraints
set_clock_uncertainty -setup 0.150 [get_clocks clk]
set_clock_uncertainty -hold  0.080 [get_clocks clk]
set_clock_transition 0.050 [get_clocks clk]

# Input Port Delay Constraints
set_input_delay -clock clk -max 0.800 [get_ports {rst_n adc_sample_valid adc_sample_in* vernier_phase* paddr* psel penable pwrite pwdata*}]
set_input_delay -clock clk -min 0.200 [get_ports {rst_n adc_sample_valid adc_sample_in* vernier_phase* paddr* psel penable pwrite pwdata*}]

# Output Port Delay Constraints
set_output_delay -clock clk -max 0.900 [get_ports {laser_trigger prdata* pready pslverr irq_target_detected status_interlock_shutdown}]
set_output_delay -clock clk -min 0.150 [get_ports {laser_trigger prdata* pready pslverr irq_target_detected status_interlock_shutdown}]

# Drive Strength and Output Load Modeling
set_driving_cell -lib_cell sky130_fd_sc_hd__inv_2 -library sky130_fd_sc_hd [all_inputs]
set_load 0.025 [all_outputs]

# Multicycle Path & False Path Constraints
set_false_path -from [get_ports rst_n]
