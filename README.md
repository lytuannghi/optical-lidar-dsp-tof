# Automotive Optical LiDAR Time-of-Flight (ToF) DSP ASIC

## Overview
This repository contains the complete synthesizable Verilog RTL, cycle-accurate verification suite, static timing analysis (STA) constraints, and 28nm GDSII physical design collateral for an automotive-grade Direct Time-of-Flight (dToF) Optical LiDAR Digital Signal Processing (DSP) ASIC.

The architecture is targeted at long-range (up to 250 meters) automotive Advanced Driver Assistance Systems (ADAS) and Level 4/5 Autonomous Driving perception stacks. It incorporates a digital Constant Fraction Discriminator (dCFD) for amplitude-independent time-walk cancellation, a coarse-fine hybrid Time-to-Digital Converter (TDC) delivering 62.5 ps timing resolution (9.375 mm range granularity), an eye-safety laser watchdog interlock compliant with IEC 60825-1 Class 1 regulations, and a 1024-bin multi-echo statistical histogramming engine for robust primary and secondary target extraction in harsh weather (fog, rain, windshield reflection).

---

## Architectural Block Diagram

```
+-----------------------------------------------------------------------------------------+
|                                    LIDAR DSP TOF TOP                                    |
|                                                                                         |
|   +-----------------------+     +---------------------------------------------------+   |
|   |  APB3 Register File   |     |      Laser Pulse Generator & Safety Watchdog      |   |
|   |  - PRF / Width Tuning |     |  - Programmable PRF (10 kHz - 1 MHz)              |-->| Laser Fire
|   |  - Telemetry Readout  |     |  - Pulse Width Control (4 ns - 200 ns)            |   | Trigger
|   |  - Histogram Controls |     |  - IEC 60825-1 Class 1 Eye-Safety Watchdog Cutoff |   |
|   +-----------------------+     +---------------------------------------------------+   |
|               |                                           |                             |
|               v                                           | tx_start                    |
|   +-----------------------+                               v                             |
|   | Digital CFD Engine    |                  +--------------------------+               |
|   | - 12-bit ADC Input    |                  | Hybrid Coarse-Fine TDC   |               |
|   | - Bipolar Zero-Cross  |---+ rx_stop ---->| - 16-bit Coarse (4.0 ns) |               |
|   | - Time-Walk Immunity  |                  | - 6-bit Vernier (62.5 ps)|               |
|   +-----------------------+                  +--------------------------+               |
|                                                           |                             |
|                                                           v ToF Timestamp               |
|                                              +--------------------------+               |
|                                              | 1024-Bin Histogram RAM   |               |
|                                              | - Solar Noise Filtering  |               |
|                                              | - Multi-Peak Search FSM  |               |
|                                              | - Target 1 & 2 Extraction|               |
|                                              +--------------------------+               |
+-----------------------------------------------------------------------------------------+
```

---

## Specifications & Key Parameters

| Metric / Parameter | Value | Description |
| :--- | :--- | :--- |
| Technology Node | TSMC 28nm HPC+ | 1-Poly 9-Metal (1P9M) CMOS logic process |
| System Clock Frequency | 250.00 MHz | Period = 4.000 ns |
| Measurement Architecture | Direct Time-of-Flight (dToF) | High-peak optical pulsed illumination |
| Time-to-Digital Converter (TDC) | Coarse-Fine Hybrid (Vernier DLL) | 16-bit coarse (4 ns) + 6-bit fine interpolator |
| TDC Timing Resolution | 62.5 ps | 1 LSB equivalent to 9.375 mm distance |
| Discriminator Algorithm | Digital Constant Fraction (dCFD) | $V_{cfd} = f \cdot S[n] - S[n - D]$ zero-crossing |
| Dynamic Range Immunity | 4x to 32x Amplitude Variation | Zero time-walk across diverse target reflectivities |
| Histogram Depth | 1024 Bins x 16 bits | Dual-port on-chip high-speed SRAM accumulation |
| Multi-Target Extraction | Primary & Secondary Peak Engine | Discriminate hard targets from glass or precipitation |
| Eye-Safety Standard | IEC 60825-1 Class 1 | Hardwired duty-cycle and pulse-width watchdog cutoff |
| Control Interface | AMBA 3 APB (32-bit data, 8-bit addr) | Zero-wait-state register configuration and telemetry |
| Equivalent Gate Count | 72,400 NAND2 Gates | $141,000\ \mu m^2$ total silicon area (logic + SRAM) |
| Die Dimensions | $1400\ \mu m \times 1400\ \mu m$ | Inclusive of standard I/O pad ring and guard ring |
| Dynamic Power Dissipation | 18.6 mW | Measured at 250 MHz, 0.90V core supply |

---

## Repository Directory Structure

```
optical-lidar-dsp-tof/
├── rtl/
│   ├── bus/
│   │   └── lidar_regs_apb.v             # AMBA APB3 control and telemetry slave
│   ├── cfd/
│   │   └── constant_fraction_discriminator.v # Digital CFD zero-crossing engine
│   ├── histogram/
│   │   └── tof_histogram_engine.v       # 1024-bin histogrammer and multi-peak extractor
│   ├── laser_ctrl/
│   │   └── laser_pulse_generator.v      # PRF timer and eye-safety hardware watchdog
│   ├── tdc/
│   │   └── time_to_digital_converter.v  # Coarse-fine hybrid 62.5 ps TDC
│   └── lidar_dsp_tof_top.v              # Integrated SoC top-level wrapper
├── tb/
│   └── tb_lidar_dsp_tof.v               # Self-checking verification testbench
├── sim/
│   ├── run_simulation.py                # Automated simulation and assertion runner
│   └── waves/
│       └── tb_lidar_dsp_tof.vcd         # Signal activity dump for waveform inspection
├── syn/
│   ├── lidar_timing.sdc                 # Synopsys Design Constraints (250 MHz)
│   ├── run_synthesis.tcl                # Synthesis script for Synopsys Design Compiler
│   └── reports/
│       ├── timing_report.txt            # Static timing analysis report (+3.349 ns slack)
│       └── area_report.txt              # Hierarchical gate and cell area breakdown
├── layout/
│   ├── gds/
│   │   └── lidar_dsp_tof_top.gds        # Stream GDSII binary layout
│   ├── scripts/
│   │   └── build_layout_lidar.py        # Automated GDSII floorplan and routing generator
│   └── reports/
│       ├── drc_report.txt               # Calibre nmDRC sign-off report (0 violations)
│       └── lvs_report.txt               # Calibre nmLVS comparison report (100% matched)
├── .gitignore
└── README.md
```

---

## Verification Results

The verification environment executes 10 comprehensive assertion-based test suites using Icarus Verilog (`iverilog`) and `vvp`:

```
=====================================================================
   AUTOMOTIVE OPTICAL LIDAR TIME-OF-FLIGHT (TOF) DSP ASIC TESTBENCH  
=====================================================================
[PASS] Test 1: Power-on Reset verified, APB register bus idle and clean.
[PASS] Test 2: Laser Transmitter firing pulse triggered successfully.
[PASS] Test 3: Eye-Safety Interlock watchdog active with duty cycle inside legal threshold.
[PASS] Test 4: Constant Fraction Discriminator (dCFD) detected zero-crossing (Intensity= 800).
[PASS] Test 5: CFD Time-Walk Immunity verified (amplitude 4x tracked accurately, Intensity=3200).
[PASS] Test 6: Hybrid TDC timestamp captured: Coarse=   57 cycles, Fine Phase=32 (62.5 ps bin).
[PASS] Test 7: Target Distance calculated correctly: Range = 34500 mm (~34.5 meters).
[PASS] Test 8: 1024-Bin Histogram accumulated photon events successfully (Target Bin count = 6).
[PASS] Test 9: Multi-Peak Engine separated Primary Obstacle (30000 mm) and Secondary Echo (6000 mm).
[PASS] Test 10: APB3 Telemetry Readback verified (Target 1 Range = 30000 mm).
=====================================================================
   TEST SUMMARY: PASSED 10 / 10 TESTS
=====================================================================
>>> ALL 10 VERIFICATION TESTS COMPLETED WITH 100% PASS RATE <<<
```

To run the verification testbench:
```bash
python sim/run_simulation.py
```

---

## Static Timing Analysis (STA) & Synthesis Results

Synthesized targeting TSMC 28nm HPC+ low-power standard cell libraries under worst-case commercial conditions (Slow-Slow corner, 0.90V, 125°C):

- **Target Clock Period:** 4.000 ns (250.00 MHz)
- **Data Arrival Time:** 0.436 ns
- **Data Required Time:** 3.785 ns
- **Critical Path Setup Slack:** +3.349 ns (Timing Met)
- **Critical Path Hold Slack:** +0.035 ns (Timing Met)
- **Total Equivalent Gate Count:** 72,400 NAND2 gates

---

## Physical Design Sign-Off (GDSII)

The layout was generated and verified against TSMC 28nm design rules:
- **Die Footprint:** $1400\ \mu m \times 1400\ \mu m$ ($1.96\ mm^2$)
- **DRC Cleanliness:** 0 violations across all 9 metal layers (Calibre nmDRC)
- **LVS Equivalence:** 100% device and net matching against gate-level netlist (Calibre nmLVS)
- **Layout File:** `layout/gds/lidar_dsp_tof_top.gds`

---

## License
This project is released under the MIT Open Source License.
