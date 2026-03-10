
## Gae-Sim Gazebo-based Autonomous Car Simulation

This repository contains the simulation environment and related tools for the ITU ZES Solar Car Team's autonomous vehicle project. Use the SSH option to clone the repository to install the project correctly.

## Table of Contents

- [Installation](#installation)
- [Usage](#usage)
- [Topics](#topics)
- [Project Structure](#project-structure)
- [Dependencies](#dependencies)
- [Configuration](#configuration)
- [Known Issues](#known-issues)
- [Screenshots](#screenshots)
- [Contributing](#contributing)


## Installation

1. Create workspace directory and clone the repository into it.

```bash
mkdir -p ~/ws_gae_sim/src
cd ~/ws_gae_sim/src
git clone --recursive git@github.com:ITU-ZES-Solar-Car-Team/gae_sim.git
```

2. Build the workspace.

```bash
cd ~/ws_gae_sim
colcon build
```

3. Source the workspace.

```bash
source install/setup.bash
```

## Usage

1. Launch the simulation environment.

```bash
ros2 launch gae_sim_gazebo gae_sim.launch.py
```

## Topics

### Lidar Point Cloud Topic

- `/lidar_front/points_in`
  - Point cloud data from the front Lidar sensor.

### ZED Camera Topics

- `/zed2_left_camera/...`
- `/zed2_left_raw_camera/...`
  - Topics related to the ZED camera including:
    - Raw and processed image data
    - Depth information
    - Point clouds


## Project Structure

```bash
.
├── gae_sim_gazebo
│   ├── CMakeLists.txt
│   ├── include
│   │   └── gae_sim_gazebo
│   │       └── GaeSimInterfacePlugin.hpp
│   ├── launch
│   │   └── gae_sim.launch.py
│   ├── package.xml
│   ├── src
│   │   └── GaeSimInterfacePlugin.cpp
│   └── worlds
│       └── empty.world
├── gae_vehicle_description
│   ├── CMakeLists.txt
│   ├── meshes
│   │   ├── gae_vehicle_body.dae
│   │   ├── wheel.dae
│   │   └── ZED2.dae
│   ├── package.xml
│   └── urdf
│       ├── gae_vehicle.urdf.xacro
│       └── zed2.xacro
├── README.md
└── velodyne_simulator
```

## Dependencies

- [ROS](https://www.ros.org/)
- [Gazebo](http://gazebosim.org/)
- [Xacro](http://wiki.ros.org/xacro)

## Configuration

The simulation environment can be configured using the launch file.


## Known Issues

- None

## Screenshots

- Will be added!

## Contributing

1. Fork the repository.
2. Create a new branch.
3. Make your changes.
4. Commit your changes.
5. Push the changes.
6. Create a pull request.

