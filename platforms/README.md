# ALFA Platforms

This directory contains the hardware platform definition files required to build and simulate ALFA FPGA bitstreams. It includes scripts, configuration files, hardware interface definitions, and device tree files needed to set up Vivado projects and integrate the ALFA-Unit into supported FPGA boards.

## Purpose

The files in this directory are used by developers who want to generate FPGA bitstreams for ALFA Extensions. This setup is used internally by the ALFA build system and is not intended for modification by ALFA-Extensions.

## Setup Instructions

To generate a Vivado project with ALFA hardware support, run:

   ```
   ./setup_vivado_project.sh [PROJECT_NAME] [BOARD] [VIVADO_PATH]
   ```

   - `PROJECT_NAME` (optional): Name of the Vivado project (default: `ALFA`)
   - `BOARD` (optional): Target FPGA board (default: `zcu104`)
   - `VIVADO_PATH` (optional): Custom Vivado installation path if not installed under `/opt/Xilinx/Vivado/`

This script will:

- Link required source and library directories
- Create a Vivado project under `vivado_projects/`
- Launch Vivado in batch mode using the appropriate TCL scripts
- Open Vivado GUI for extension development

**Supported boards:**

- `zcu104`

## Directory Overview

### `dt/`

Device tree include files for ALFA integration.

### `xilinx/gui/`

GUI configuration for Vivado IP integrator.

### `xilinx/hardware/`

Board-specific hardware definition files.

### `xilinx/int/`

Interface descriptors for ALFA data structures and communication protocols, including both `.xml` and `_rtl.xml` definitions.

### `xilinx/sim/`

Simulation testbenches and SystemVerilog modules for validating ALFA hardware behavior.

### `tcl/zcu104/`

TCL scripts to instantiate and configure the ALFA-Unit for the ZCU104 board.

### `xilinx/xgui/`

Vivado IP packaging metadata for the ALFA-Unit.

## Notes

- The setup script assumes the standard folder structure for hardware libraries and extensions. Changing this structure may require modifications to the script and therefore is not recommended.
- Vivado must be installed and accessible either in the default system path or provided explicitly as a parameter.
