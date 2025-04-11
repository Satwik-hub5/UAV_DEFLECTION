# UAV Strategic Deconfliction System

## Overview
This system provides strategic deconfliction for drones operating in shared airspace by checking for spatial and temporal conflicts between a primary drone's mission and other simulated drone trajectories.

## Features
- Spatial conflict detection with configurable safety buffer
- Temporal conflict detection with time interpolation
- 2D and 3D visualization of drone trajectories
- Animated visualization of drone movements
- Detailed conflict reporting

## Requirements
- Python 3.8+
- Required packages: numpy, matplotlib, scipy

## Installation
1. Clone this repository
2. Create and activate a virtual environment:
   ```bash
   python -m venv uav_env
   source uav_env/bin/activate  # On Windows: uav_env\Scripts\activate