#!/usr/bin/env python3
"""
Simulation Automation Script for Automotive Optical LiDAR Time-of-Flight (ToF) DSP ASIC
Compiles synthesizable RTL and self-checking testbench using Icarus Verilog and executes VVP.
"""

import subprocess
import sys
import os

def run():
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    sim_dir = os.path.join(repo_root, "sim")
    waves_dir = os.path.join(sim_dir, "waves")
    os.makedirs(waves_dir, exist_ok=True)

    rtl_files = [
        os.path.join(repo_root, "rtl", "laser_ctrl", "laser_pulse_generator.v"),
        os.path.join(repo_root, "rtl", "cfd", "constant_fraction_discriminator.v"),
        os.path.join(repo_root, "rtl", "tdc", "time_to_digital_converter.v"),
        os.path.join(repo_root, "rtl", "histogram", "tof_histogram_engine.v"),
        os.path.join(repo_root, "rtl", "bus", "lidar_regs_apb.v"),
        os.path.join(repo_root, "rtl", "lidar_dsp_tof_top.v"),
    ]

    tb_file = os.path.join(repo_root, "tb", "tb_lidar_dsp_tof.v")
    out_vvp = os.path.join(sim_dir, "lidar_sim.vvp")

    print("[INFO] Compiling Automotive Optical LiDAR ToF DSP ASIC RTL and Testbench...")
    cmd_compile = ["iverilog", "-g2012", "-o", out_vvp, "-I", os.path.join(repo_root, "rtl")] + rtl_files + [tb_file]
    res_compile = subprocess.run(cmd_compile, capture_output=True, text=True)

    if res_compile.returncode != 0:
        print("[ERROR] Compilation failed:")
        print(res_compile.stderr)
        sys.exit(1)
    print("[INFO] Compilation successful!")

    print("[INFO] Running VVP simulation...")
    cmd_run = ["vvp", out_vvp]
    res_run = subprocess.run(cmd_run, cwd=repo_root, capture_output=True, text=True)

    print(res_run.stdout)
    if res_run.stderr:
        print("[STDERR]:", res_run.stderr)

    if "ALL 10 VERIFICATION TESTS COMPLETED WITH 100% PASS RATE" in res_run.stdout:
        print("[SUCCESS] All 10 Optical LiDAR ToF test cases PASSED!")
        sys.exit(0)
    else:
        print("[FAILURE] Simulation failed assertion checks.")
        sys.exit(1)

if __name__ == "__main__":
    run()
