##  GAE Autonomous Robotics Workspace

This repository contains a comprehensive ROS 2 development environment tailored for autonomous vehicle research and development. It leverages VS Code Dev Containers and Docker to provide a consistent, isolated, and high-performance workspace, ensuring all dependencies and tools are pre-configured.

### Key Features

    Containerized Development: Fully managed environment using Docker to eliminate "it works on my machine" issues.

    Integrated Desktop Environment: Built-in VNC support (NoVNC) for GUI-based tools like RViz2 and Gazebo.

    Autonomous Suite: Specialized packages for Perception, Localization (Sensor Fusion), and Path Planning.

    Autoware Integration: Compatibility with Autoware-based stacks and message types.

### Prerequisites

Before you begin, ensure you have the following installed on your host machine:

    Docker Desktop or Docker Engine

    Visual Studio Code

    Dev Containers Extension (Microsoft)

### Getting Started
1. **Clone the Repository**
Bash

git clone https://github.com/metehansaygili/my_ros2_project.git
cd my_ros2_project

2. **Set Permissions (Linux Only)**

To allow VS Code to communicate with the Docker daemon, run:
Bash

sudo chmod 666 /var/run/docker.sock

3. **Open in Dev Container**

    Open the folder in VS Code: code ..

    When prompted with "Folder contains a Dev Container configuration file", click Reopen in Container.

    Wait for the container to build; this will automatically install ROS 2 and all necessary system dependencies.

### Project Structure

The workspace is organized into modular packages within the src/ directory:

    gae_msgs: Custom ROS 2 interfaces, messages, and service definitions.

    gae_sim: Configuration files and launch scripts for simulation environments.

    multi_sensor_fusion_localization: Algorithms for fusing LiDAR, IMU, and Odometry data for precise positioning.

    perception-lidar: Point cloud processing and object detection pipelines.

    planner: Global and local trajectory planning for autonomous navigation.

    gae_shell_yolo: Integration of YOLO-based vision detection systems.

### Build and Run

Once inside the container terminal, follow these steps to compile the workspace:
Bash

### Update and install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -y -r

### Build the workspace
colcon build --symlink-install --continue-on-error

### Source the environment
source install/setup.bash

### References & Acknowledgements

This project utilizes several open-source frameworks and templates:

    ROS 2 (Robot Operating System 2): The core middleware used for robot communication and task management.

    Autoware Foundation: Various packages and message types are derived from the Autoware.Universe and Tier4 autonomous driving stacks.

    LCAS ROS 2 Template: The Docker and Dev Container configuration is based on the LCAS (Lincoln Centre for Autonomous Systems) ROS 2 package template.
