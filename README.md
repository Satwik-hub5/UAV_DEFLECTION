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
1. Clone this repository:
   ```bash
   git clone https://github.com/Satwik-hub5/UAV_DEFLECTION.git
   cd UAV_DEFLECTION
   ```

2. Create and activate a virtual environment:
   ```bash
   python -m venv uav_env
   # On Windows:
   uav_env\Scripts\activate
   # On macOS/Linux:
   source uav_env/bin/activate
   ```

3. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

## Usage
Run the main simulation:
```bash
python src/uav_deconfliction.py
```

This will:
1. Generate sample drone trajectories
2. Detect conflicts
3. Create visualization plots in the `outputs/` folder
4. Generate an animation (`drone_animation.gif`)

## Output Examples
![2D Trajectories](outputs/2d_trajectories.png)
![3D Trajectories](outputs/3d_trajectories.png)
![Animation](outputs/drone_animation.gif)

## Project Structure
```
UAV_DEFLECTION/
├── src/                      # Source code
│   ├── uav_deconfliction.py  # Main implementation
│   ├── __init__.py
├── outputs/                  # Generated files
│   ├── drone_animation.gif
│   ├── 2d_trajectories.png
│   ├── 3d_trajectories.png
├── docs/                     # Documentation
│   ├── REFLECTION.md
│   ├── ASSIGNMENT_README.md
├── tests/                    # Test cases
├── README.md                 # This file
├── requirements.txt          # Dependencies
├── .gitignore
```

## License
[MIT License](LICENSE) - Feel free to use and modify this project.