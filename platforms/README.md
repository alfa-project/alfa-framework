# ALFA Platforms

This directory contains all platform-specific assets required to deploy ALFA. It centralizes configuration, IP packaging, integration logic, and automation scripts to streamline project setup and hardware generation.

## Directory Structure

- `dt/` - Contains device tree snippets (`.dtsi`) that integrate ALFA-specific components into the board-level device tree.
- `xilinx/` - Main platform integration directory for Vivado and Xilinx tools. It includes:
  - `gui/`: UI files for customizing Vivado IP parameters.
  - `hardware/`: Stores `.xsa` files exported from Vivado for each platform configuration. These are used for Petalinux builds or hardware/software co-design.
  - `int/`: Integration-specific logic such as wrappers, bridges, or interface glue modules.
  - `sim/`: Simulation files and testbenches used for IP functional verification.
  - `tcl/`: Automation scripts for Vivado (e.g., IP packaging, constraints, block design generation).
  - `src/`: Source files for ALFA Core components.
  - `reps/`: Source files for ALFA Libs and ALFA Extensions that are reused across multiple platforms.
  - `xgui/`: Vivado IP GUI metadata to integrate ALFA blocks into the IP catalog.
  - `component.xml`: Vivado descriptor that defines the IP core structure and interface.
  - `setup_vivado_project.sh`: Shell script to initialize and configure the Vivado project for this platform.

## Project initialization

### Vivado Setup Script

The `setup_vivado_project.sh` script automates the creation of a Vivado project, launching Vivado with the correct board configuration. The script will setup all the symbolic links to the ALFA source core, create a folder structure (vivado_projects/<project_name>), and initialize the Vivado project with the basic components.

### Usage
Inside the `platforms/xilinx` directory, run the following command to set up your Vivado project:
```bash
./setup_vivado_project.sh <project_name> <board_name> <optional_vivado_path>
```

where:

- `project_name` (optional): Name of the Vivado project (default: ALFA)
- `board_name` (optional): Supported board (default: zcu104)
- `optional_vivado_path` (optional): Path to Vivado installation (used if not installed in /opt/Xilinx/Vivado)
