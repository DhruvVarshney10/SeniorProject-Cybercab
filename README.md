# RoboCore Robot Platform

A comprehensive ROS2 robotics platform featuring modular components for mobile robot control, perception, and navigation.

## Overview

RoboCore is a flexible robotics framework built using ROS2, designed for differential drive robots with a focus on modularity, extensibility, and modern C++ practices. The platform provides integrated functionality for:

- Motion control and odometry
- Parameter management
- State estimation and filtering
- Safety monitoring and collision avoidance
- Hardware interfacing

## Package Structure

The project is organized into the following packages:

- **robocore_launcher**: Launch files for various robot configurations
- **robocore_control**: Motion control, kinematics, and odometry
- **robocore_cpp_demos**: Example implementations of ROS2 communication patterns
- **robocore_model**: Robot URDF models, meshes, and visualization
- **robocore_hardware**: Hardware interface implementations
- **robocore_positioning**: Sensor fusion, state estimation, and localization
- **robocore_navigation**: Mapping and navigation capabilities
- **robocore_interfaces**: Custom message, service, and action definitions
- **robocore_utilities**: Helper utilities and safety modules

## Key Features

### Modern C++ Design Patterns

The codebase employs modern C++ design patterns to enhance maintainability and extensibility:

- **Strategy Pattern**: Used in motion control for different kinematics models
- **Observer Pattern**: Used in parameter management for monitoring configuration changes
- **Component-Based Design**: Used in the safety system for separation of concerns
- **Template Metaprogramming**: Used in state estimation for flexible filter implementations

### Safety Features

The platform includes an emergency halt system that:
- Monitors LiDAR data to detect obstacles
- Creates warning and danger zones for collision prevention
- Implements graduated speed control based on obstacle proximity
- Provides emergency braking for imminent collisions

### State Estimation

The positioning system includes:
- Odometry calculation from wheel encoders
- Sensor fusion with IMU data
- Kalman filtering for noise reduction
- Extensible filtering framework

## Building the Project

### Prerequisites

- ROS2 Humble or newer
- C++17 compatible compiler
- Eigen3 library
- Hardware dependencies:
  - Differential drive robot or simulator
  - LiDAR sensor (for safety features)
  - IMU (for state estimation)
  - Encoders (for odometry)

### Installation

1. Clone the repository into your ROS2 workspace src directory:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/yourusername/robocore.git
   ```

2. Install dependencies:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. Build the workspace:
   ```bash
   colcon build --symlink-install
   ```

4. Source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Usage

### Starting the Robot

For a simulated robot:
```bash
ros2 launch robocore_launcher simulated_robot.launch.py
```

For a physical robot:
```bash
ros2 launch robocore_launcher real_robot.launch.py
```

### Key Nodes

- **Motion Controller**: Controls robot movement based on velocity commands
  ```bash
  ros2 run robocore_control motion_controller
  ```

- **State Estimator**: Filters sensor data for improved odometry
  ```bash
  ros2 run robocore_positioning state_estimator
  ```

- **Emergency Halt**: Monitors for obstacles and provides safety control
  ```bash
  ros2 run robocore_utilities emergency_halt
  ```

### Configuration

Most nodes support runtime parameter configuration through the ROS2 parameter system:

```bash
# Example: Set the warning distance for collision detection
ros2 param set /emergency_halt_node warning_distance 0.8
```

## Architecture

The system follows a layered architecture:

1. **Hardware Interface Layer**: Direct communication with sensors and actuators
2. **Control Layer**: Motion control and kinematics
3. **Perception Layer**: Sensor fusion and state estimation
4. **Planning Layer**: Navigation and path planning
5. **Safety Layer**: Collision avoidance and emergency handling

Components communicate through ROS2 topics, services, and actions, with a focus on loose coupling and high cohesion.

## Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues to improve the platform.

 
