# ALFA-Monitor

The ALFA-Monitor is a high-level C++ application designed for desktop systems to visualize, configure, and evaluate components within the ALFA ecosystem. It provides a GUI for interacting with point cloud data, managing ALFA-Extensions, and monitoring system performance in real-time or offline.

## Features

### Point Cloud Visualization

- Load and display point cloud data from:
  - Live LiDAR sensors
  - Pre-recorded ROS 2 bag files
- Playback controls:
  - Play, pause, step forward/backward, jump to specific frames
- Dual viewer support:
  - Compare original vs. processed point clouds side-by-side
- Customizable visualization:
  - Color by intensity, label, RGB, or distance
  - Save and load camera settings for reproducible views
  - Interactive camera control using mouse navigation

### Extension Configuration and Monitoring

- View and modify configuration parameters of ALFA-Extensions at runtime
- Monitor performance metrics such as:
  - Number of points per frame
  - Processing time
  - Extension-specific statistics (e.g., number of ground points)
- Real-time feedback for parameter tuning and debugging

### Evaluation Tools

- **Noise Injection Tool**:
  - Define regions in the point cloud and insert synthetic noise
  - Evaluate robustness of denoising or filtering algorithms

- **Automated Evaluation Scripts**:
  - Run filtering or segmentation tests on labeled datasets
  - Generate performance reports including:
    - True positives / false positives
    - Quantitative accuracy per class or region

## User Guide

A full usage guide for the ALFA-Monitor is available [here](https://github.com/alfa-project/alfa-framework/blob/main/docs/guides/monitor_user_guide.md). This guide includes instructions for launching the monitor, interacting with the interface, interpreting performance metrics, and running evaluation tools.
