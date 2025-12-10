# AIS Coursework 2 - Cozmo Robot Navigation & Localization

This repository contains implementations of advanced robotics algorithms for the Cozmo robot, including navigation, probabilistic localization, and SLAM (Simultaneous Localization and Mapping).

## Project Overview

This coursework demonstrates various autonomous robotics capabilities using the Anki Cozmo robot:

- **Navigation**: Path planning with obstacle avoidance and wall detection
- **Probabilistic Localization**: Particle filter-based robot localization using wall markers
- **SLAM**: Simultaneous mapping and localization in unknown environments
- **Exploration**: Autonomous exploration with obstacle detection and path planning

## Files Description

### Core Modules

- **`frame2d.py`**: 2D coordinate frame transformations using homogeneous matrices. Provides the `Frame2D` class for robot pose representation and coordinate transformations.

- **`cozmo_interface.py`**: Low-level robot control interface including:
  - Forward and inverse kinematics for differential drive
  - Trajectory planning (linear and spline-based controllers)
  - Odometry noise parameters
  - Sensor models for cube detection

- **`distributions.py`**: Probability distribution utilities for particle filtering and localization.

### Main Programs

- **`navigate_matte.py`**: Navigation system with:
  - Target-based path planning
  - Wall detection using image processing
  - Obstacle avoidance
  - Visual path tracking and performance metrics
  - Supports point-to-point navigation with dynamic replanning

- **`ProbabilisticLocalisation.py`**: Particle filter implementation featuring:
  - 500-particle localization system
  - Wall marker detection and association
  - Measurement model incorporating distance and angle observations
  - Motion model with odometry noise
  - Real-time pose estimation

- **`SLAM.py`**: Simultaneous Localization and Mapping:
  - FastSLAM-style particle-based SLAM
  - Incremental map building
  - Wall and cube landmark detection
  - Uncertainty tracking for landmarks
  - Exploration strategy with frontier-based planning

### Exploration Module

- **`Exploration/ExploreAPI.py`**: Autonomous exploration framework
- **`Exploration/frame2d.py`**: Coordinate transformation utilities for exploration

## Dependencies

```bash
# Core dependencies
pip install cozmo
pip install numpy
pip install matplotlib
pip install Pillow

# Optional for advanced features
conda install numpy scipy matplotlib
```

## Setup

1. **Install Cozmo SDK**:
   ```bash
   pip install cozmo
   ```

2. **Configure Cozmo Connection**:
   - Connect your mobile device with the Cozmo app
   - Enable SDK mode in the app
   - Ensure your computer and device are on the same network

3. **Clone Repository**:
   ```bash
   git clone https://github.com/MatteGombia/AIS_Coursework2.git
   cd AIS_Coursework2
   ```

## Usage

### Navigation

Run the navigation demo to move Cozmo to a target position while avoiding walls:

```bash
python3 navigate_matte.py
```

The robot will:
- Navigate to specified target coordinates
- Detect and avoid walls using camera-based detection
- Generate visualization of the path taken

### Probabilistic Localization

Execute particle filter localization:

```bash
python3 ProbabilisticLocalisation.py
```

This demonstrates:
- Robot pose estimation using wall markers
- Particle filtering with resampling
- Convergence to true position over time

### SLAM

Run the SLAM implementation:

```bash
python3 SLAM.py
```

Features:
- Simultaneous map building and localization
- Landmark detection (walls and cubes)
- Exploration of unknown environments
- Uncertainty representation

## Key Algorithms

### 1. Particle Filter Localization
- **Particles**: 500 particles representing possible robot poses
- **Motion Model**: Incorporates odometry noise (configurable in `cozmo_interface.py`)
- **Measurement Model**: Uses distance and angle to detected wall markers
- **Resampling**: Low-variance resampling when effective particle count drops

### 2. Path Planning
- **Linear Controller**: Proportional control for target approach
- **Obstacle Avoidance**: Path-blocking detection using line-segment distance
- **Wall Detection**: Image difference-based wall proximity detection

### 3. SLAM Implementation
- FastSLAM approach with particle-based map representation
- Separate uncertainty tracking for each landmark
- Graph-based exploration using connectivity map

## Configuration

Key parameters can be adjusted in respective files:

**`cozmo_interface.py`**:
```python
cozmoOdomNoiseX = 0.2      # X-axis odometry noise
cozmoOdomNoiseY = 0.2      # Y-axis odometry noise
cozmoOdomNoiseTheta = 0.001  # Rotational odometry noise
wheelDistance = 80          # Wheel separation (mm)
```

**`navigate_matte.py`**:
```python
WHEEL_SPEED = 250           # Movement speed (mm/s)
DISTANCE_PER_MOVE = 100     # Step size (mm)
WALL_THRESHOLD = 50         # Wall detection sensitivity
```

**`ProbabilisticLocalisation.py`** / **`SLAM.py`**:
```python
num_particles = 500         # Particle filter size
WALL_RADIUS = 100          # Detection radius for walls
```

## Output & Visualization

The programs generate various visualizations:

- **Path plots**: Showing robot trajectory, target, and obstacles
- **Particle distributions**: Visualizing localization uncertainty
- **Maps**: SLAM-generated maps with landmarks and uncertainty ellipses
- **Performance metrics**: Distance error, time taken, success rate

Screenshots are saved to the `screenshots/` directory.

## Troubleshooting

### Connection Issues
- Ensure Cozmo app is in SDK mode
- Check network connectivity between device and computer
- Restart the Cozmo app if connection drops

### Poor Localization
- Adjust odometry noise parameters
- Increase number of particles
- Ensure wall markers are clearly visible
- Calibrate `wheelDistance` parameter experimentally

### Navigation Failures
- Adjust `WALL_THRESHOLD` for environment lighting
- Tune controller gains in `target_pose_to_velocity_linear`
- Increase `clearance_mm` in obstacle avoidance checks

## Project Structure

```
AIS_Coursework2/
├── README.md                          # This file
├── navigate_matte.py                  # Navigation implementation
├── ProbabilisticLocalisation.py       # Particle filter localization
├── SLAM.py                            # SLAM implementation
├── cozmo_interface.py                 # Robot control interface
├── frame2d.py                         # 2D transformations
├── distributions.py                   # Probability utilities
├── Exploration/                       # Exploration module
│   ├── ExploreAPI.py
│   └── frame2d.py
└── screenshots/                       # Output visualizations
```

## Academic Context

This project is part of the Artificial Intelligence Systems coursework, demonstrating:
- Probabilistic robotics techniques
- Sensor fusion and state estimation
- Autonomous navigation in uncertain environments
- Real-time decision making under uncertainty

## Author

Matteo Gombia

## License

This project is for academic purposes. Please refer to university guidelines for code sharing and collaboration policies.
