# ==============================================================================
# Design Compiler Synthesis Script
# Design: lidar_dsp_tof_top
# Target Library: SkyWater 130nm HD Standard Cell Library (1.80V, 25C)
# ==============================================================================

set DESIGN_NAME "lidar_dsp_tof_top"

set_app_var search_path ". ./rtl ./rtl/laser_ctrl ./rtl/cfd ./rtl/tdc ./rtl/histogram ./rtl/bus /tools/synopsys/libs"
set_app_var target_library "sky130_fd_sc_hd
set_app_var link_library "* sky130_fd_sc_hd sky130_fd_sc_hd

# Analyze Verilog RTL Sources
analyze -format verilog {
    ./rtl/laser_ctrl/laser_pulse_generator.v
    ./rtl/cfd/constant_fraction_discriminator.v
    ./rtl/tdc/time_to_digital_converter.v
    ./rtl/histogram/tof_histogram_engine.v
    ./rtl/bus/lidar_regs_apb.v
    ./rtl/lidar_dsp_tof_top.v
}

# Elaborate Top-Level Module
elaborate $DESIGN_NAME

# Apply Timing Constraints
source ./syn/lidar_timing.sdc

# Check Unresolved References and Constraints
check_design > ./syn/reports/check_design.rpt
check_timing > ./syn/reports/check_timing.rpt

# Compile and Map Logic
compile_ultra -gate_clock -scan

# Generate Reports
report_timing -delay_type max -max_paths 10 > ./syn/reports/timing_report.txt
report_timing -delay_type min -max_paths 10 >> ./syn/reports/timing_report.txt
report_area -hierarchy > ./syn/reports/area_report.txt
report_power -hierarchy > ./syn/reports/power_report.txt
report_qor > ./syn/reports/qor_report.txt

# Export Synthesized Gate-Level Netlist and Constraints
write -format verilog -hierarchy -output ./syn/netlist/lidar_dsp_tof_top_syn.v
write_sdc ./syn/netlist/lidar_dsp_tof_top_syn.sdc

puts "[INFO] Synthesis completed successfully for $DESIGN_NAME."
