# autonomous_robot_gardener
this is a repo for my autonomous robot gardener.

[explanatory video](https://youtu.be/5GHLeow5bYI)


## Getting Started

Clone this repository:
```bash
git clone https://github.com/jonathanrandall/autonomous_robot_gardener.git
```

Navigate into the workspace:
```bash
cd autonomous_robot_gardener/ros_ws
```

Build the workspace:
```bash
colcon build --symlink-install
```

---

## Running the Simulation

To launch the simulation:
```bash
ros2 launch autonomous_robot launch_sim.launch.py
```

To view the robot arm in RViz:
```bash
ros2 launch xarm_description view_robot.launch.py
```

To control the robot arm with a gamepad:
```bash
ros2 launch xarm_description joystick_ik.launch.py
```

To launch just the robot arm in gazebo
```bash
ros2 launch xarm_description launch_sim.xarm.launch.py
```

---

## Running the Real Robot

### On the Mini PC

Launch the robot:
```bash
ros2 launch autonomous_robot mini_pc.launch.py
```

### On the Raspberry Pi

Launch the pan-tilt control:
```bash
ros2 launch autonomous_robot pi.launch.py
```

Launch the camera publisher:
```bash
ros2 launch autonomous_robot camera_publisher.launch.py
```

### Back on the Mini PC

Launch the joint states merger:
```bash
ros2 launch autonomous_robot merge_joint_states.launch.py
```

Launch the pan-tilt joystick control:
```bash
ros2 launch pan_tilt_description pan_tilt.joy.launch.py
```

Run the camera to end-effector node:
```bash
ros2 run autonomous_robot camera_to_ee.py
```

Run the IK vertical angle node (replace `angle` with your desired angle value):
```bash
ros2 run xarm_description ik_vertical_angle_node.py --ros-args -p vertical_angle:=angle
```

Launch the image sync processor:
```bash
ros2 launch autonomous_robot image_sync_processor.launch.py
```

Launch the leaf tracker:
```bash
ros2 launch autonomous_robot leaf_tracker.launch.py
```

---

## Hardware Code Notes
I'm using the hiwonder xarm with the esp32 board. The micropython code is in the [xarm_software/micorpython_xarm_control](https://github.com/jonathanrandall/autonomous_robot_gardener/tree/main/xarm_software/micropython_xarm_control) directory. I used PyMakr to load it onto the esp32 board.

The esp32 code for the motor control is in the [esp32_controls_diffbot_serial](https://github.com/jonathanrandall/autonomous_robot_gardener/tree/main/esp32_controls_diffbot_serial) directory

---

# Repo Structure Overview

This repository is organized into several key directories, each serving a specific purpose in the development and operation of the autonomous robot gardener.

## docs/

The documentation and configuration directory containing development tools, reference materials, and project documentation:

- **dot_vscode/**: VSCode workspace configuration files, primarily containing C++ include paths and build settings for proper IntelliSense support when working with ROS2 packages.

- **photos/**: Visual documentation and images used throughout the repository, including build photos and system diagrams.

- **robot_docker_files/**: Docker container configuration and build files for the development and runtime environment. Using Docker is highly recommended as it:
  - Allows ROS version upgrades without OS upgrades
  - Provides environment isolation preventing dependency conflicts
  - Enables easy environment reconstruction if issues arise
  - Ensures consistent development environments across different machines

- **trouble_shooting_and_tips/**: Collection of troubleshooting guides and useful resources including:
  - Gamepad setup instructions (adding user to input group for ROS2 joystick access)
  - ROS bridge message summaries from Gazebo
  - ROS2 CLI command cheat sheet
  - `esp32_mapping.txt`: Guide for mapping ESP32 devices to static USB ports using udev rules
  - `twist_mux.yaml`: Updated velocity multiplexer configuration adapted from Articubot One
  - `xarm_measurements.jpg`: Physical measurements of the XArm for kinematics calculations

- **chassis_bom.md**: Complete bill of materials for the mechanical chassis components including frame, wheels, motors, and mounting hardware.

- **electronics_bom.md**: Bill of materials for all electronic components including microcontrollers, motor drivers, sensors, and wiring.

## esp32_controls_diffbot_serial/

Contains the embedded firmware for the ESP32 microcontroller that interfaces with the motor controllers. This code:
- Receives velocity commands from the ROS2 diff_drive controller via serial communication
- Implements PID control loops for accurate wheel speed control
- Provides encoder feedback for odometry calculations
- Manages four-wheel drive motor control with individual wheel control
- Handles low-level motor driver communication and safety features

## ros_ws/src/

The ROS2 workspace source directory containing custom packages developed for this robot:

### 1. autonomous_robot
The main integration package that brings together all robot components. Features include:
- **Robot description**: Combined URDF/Xacro files integrating the mobile base with the XArm manipulator
- **Launch files**: System startup scripts for both simulation and real hardware
- **Configuration files**: ROS2 Control configurations, navigation parameters, and controller settings
- **Chassis integration**: Mobile base controller configuration and sensor integration
- **Camera processing nodes**: Image processing and computer vision for plant detection
- Built upon Josh Newans' Articubot One project with extensive modifications for arm integration and autonomous gardening capabilities

### 2. diffdrive_arduino
Hardware interface package providing ROS2 Control integration for the differential drive mobile base:
- **Serial communication**: Interfaces with ESP32 motor controller via serial protocol
- **Odometry publishing**: Calculates and publishes robot position and velocity based on encoder data
- **Command handling**: Translates ROS2 velocity commands to motor controller instructions
- **Four-wheel drive support**: Adapted from the original two-wheel Articubot One design to support four independently controlled wheels
- Named "diffdrive_arduino" to maintain compatibility with the original Josh Newans package structure

### 3. xarm_description
Description and visualization package for the HiWonder XArm robotic manipulator:
- **URDF/Xacro models**: Complete robot description with accurate link geometries, joint limits, and inertial properties
- **Mesh files**: 3D visual and collision meshes for simulation and visualization
- **Launch files**:
  - RViz visualization configurations
  - Gazebo simulation launch files
  - Joystick teleoperation for manual arm control
- **Inverse kinematics nodes**: Custom IK solvers for positioning the end-effector
- **Controllers**: Joint trajectory controller and position controller configurations

### 4. xarm_hardware
ROS2 Control hardware interface for the HiWonder XArm with ESP32 control board:
- **Hardware interface**: Implements ROS2 Control's hardware interface for real-time joint control
- **Serial communication**: Communicates with the ESP32 running MicroPython firmware on the XArm
- **Joint state publishing**: Reads and publishes current joint positions from the arm
- **Command execution**: Sends joint position commands to the servos
- **Safety features**: Joint limit enforcement and error handling 


# Bill of Materials (BOM)
![chassis picture](https://github.com/jonathanrandall/autonomous_robot_gardener/blob/main/docs/photos/chassis_pic1)


This project has separate BOMs for mechanical and electronic components:

- [Chassis BOM](docs/chassis_bom.md)  
- [Electronics BOM](docs/electronics_bom.md)
