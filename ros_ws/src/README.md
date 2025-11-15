# Autonomous Robot Gardener - ROS 2 Packages

This workspace contains packages for an autonomous robot gardener system built with ROS 2 Jazzy. The system integrates computer vision, depth sensing, robotic manipulation, and differential drive control.

## System Overview

The autonomous robot gardener combines:
- **Vision System**: RGB webcam + ToF (Time-of-Flight) depth camera with YOLO object detection
- **Pan-Tilt Mechanism**: Active camera tracking with PCA9685 servo control
- **Robotic Arm**: 4-DOF xArm for manipulation tasks
- **Mobile Base**: Differential drive robot with Arduino control
- **Coordinate Transforms**: TF2-based camera-to-end-effector transformations

---

## Packages

### 1. `camera_publisher`

**Type**: Python package
**Description**: Publishes compressed images from Arducam ToF and RGB webcam cameras.

**Nodes**:
- `tof_publisher` - Publishes depth images from ToF camera
- `webcam_publisher` - Publishes RGB images from webcam

**Topics Published**:
- `/tof/image_raw/compressed` - Compressed ToF depth images
- `/webcam/image_raw/compressed` - Compressed RGB images

**Use Case**: Provides synchronized camera data for vision processing pipeline.

---

### 2. `image_sync_processor`

**Type**: Python package
**Description**: Synchronizes and processes webcam and ToF images for object detection with distance estimation.

**Key Features**:
- Synchronizes RGB and ToF image streams using `message_filters`
- YOLO-based object detection (leaf and remote detection)
- K-means clustering on depth data for segmentation
- Distance calculation from shifted depth images
- Filters detections by confidence (>0.35) and distance (<60cm threshold)
- Publishes detection data as JSON with bounding boxes and distances

**Nodes**:
- `image_sync_node` - Main synchronization and processing node

**Topics Subscribed**:
- `/webcam/image_raw/compressed`
- `/tof/image_raw/compressed`

**Topics Published**:
- `/detections/labels_distances` (String) - JSON array of detections
- `/processed_image/compressed` - Annotated image with bounding boxes

**Detection Format**:
```json
{
  "class": "leaf",
  "distance_cm": 45.2,
  "confidence": 0.95,
  "x_center": 180,
  "y_center": 85,
  "xn": 320,
  "yn": 240
}
```

---

### 3. `pan_tilt_track`

**Type**: Python package
**Description**: Servo-based object tracking for pan-tilt camera mount using detection data.

**Nodes**:
- `person_tracker` - Tracks person/remote objects (legacy)
- `leaf_tracker` - Tracks leaf objects with distance filtering

**Key Features** (leaf_tracker):
- Filters detections with confidence > 0.35
- Selects closest object (minimum distance_cm)
- Ignores objects farther than 45cm
- Proportional control with configurable scale factors
- Deadzone (±0.015 rad) to prevent jitter
- Publishes target distance to `dist_camera` topic

**Topics Subscribed**:
- `/detections/labels_distances` - Detection data from image_sync_processor
- `/pi/joint_states` - Current pan/tilt servo positions

**Topics Published**:
- `/pi/pan_tilt_controller/commands` - Servo position commands
- `/dist_camera` (PointStamped) - Target object distance in camera frame

**Launch Files**:
- `leaf_tracker.launch.py` - Start leaf tracking with configurable parameters

**Parameters**:
- `scale_factor_x` (default: 0.2) - Pan axis gain
- `scale_factor_y` (default: 0.2) - Tilt axis gain
- `controller_namespace` (default: 'pi') - Servo controller namespace
- `robot_namespace` (default: '') - Robot TF frame namespace

---

### 4. `pan_tilt_description`

**Type**: Python package
**Description**: URDF description and simulation setup for pan-tilt camera mechanism.

**Key Features**:
- URDF/Xacro robot description
- Gazebo Harmonic simulation support
- Joystick teleoperation
- ROS 2 Control integration with `forward_command_controller`

**Launch Files**:
- Simulation and teleop launch configurations

**Dependencies**:
- `ros_gz_sim` - Gazebo simulation
- `gz_ros2_control` - Control integration
- `joy` - Joystick input

---

### 5. `pan_tilt_hardware`

**Type**: C++ package (ament_cmake)
**Description**: ROS 2 Control hardware interface for PCA9685-based pan-tilt servos.

**Key Features**:
- Hardware interface plugin for `ros2_control`
- I2C communication with PCA9685 PWM controller
- Position command interface for pan and tilt joints
- Joint state feedback

**Plugin**: Exports hardware interface for controller manager

---

### 6. `autonomous_robot`

**Type**: Mixed C++/Python package
**Description**: High-level coordination and transform management for the robot system.

**Nodes**:
- `camera_to_ee` - Transforms camera frame coordinates to end-effector frame
- `joint_state_merger` - Merges joint states from multiple robot components

**Key Features** (camera_to_ee):
- Subscribes to `/dist_camera` topic from pan_tilt_track
- Uses TF2 to transform points from camera frame to robot base frame
- Publishes transformed points for manipulation planning

**Topics**:
- Subscribed: `/dist_camera` (PointStamped)
- Published:
  - `/cam_to_ee/camera_point` - Point in camera frame
  - `/cam_to_ee/ee_point` - Point in end-effector frame

**Parameters**:
- `robot_namespace` - TF frame namespace prefix

---

### 7. `xarm_description`

**Type**: C++ package (ament_cmake)
**Description**: URDF description and simulation for 4-DOF xArm robotic manipulator.

**Key Features**:
- Complete robot URDF with meshes
- Gazebo simulation support
- Inverse kinematics using IKPy library
- Joystick control node
- Trajectory execution

**Launch Files**:
- `gz.launch.py` - Gazebo simulation
- `arm.launch.py` - Hardware bringup
- `joystick_ik.launch.py` - Joystick IK control
- `view_robot.launch.py` - RViz visualization
- `joint_slider_gui.launch.py` - GUI joint control

**Dependencies**:
- `python3-ikpy` - Inverse kinematics solver
- `trajectory_msgs` - Trajectory execution
- `gz_ros2_control` - Gazebo control integration

---

### 8. `xarm_hardware`

**Type**: C++ package (ament_cmake)
**Description**: Hardware interface for xArm using serial communication with ESP32.

**Key Features**:
- `ros2_control` hardware interface plugin
- LibSerial-based UART communication
- Joint trajectory controller support
- Joint state broadcasting

**Controllers**:
- `joint_state_broadcaster` - Publishes joint states
- `joint_trajectory_controller` - Executes trajectories

**Communication**: Serial protocol with ESP32 for servo control

**Launch Files**:
- `xarm_hardware.launch.py` - Start hardware interface and controllers

---

### 9. `diffdrive_arduino`

**Type**: C++ package (ament_cmake)
**Description**: ROS 2 Control hardware interface for differential drive robot with Arduino.

**Key Features**:
- Hardware interface for differential drive base
- LibSerial communication with Arduino
- Odometry calculation
- Velocity command execution

**Controllers**:
- `diff_drive_controller` - Differential drive control
- `joint_state_broadcaster` - Wheel state publishing

**Launch Files**:
- `diffbot.launch.py` - Start differential drive hardware interface

**Communication**: Serial protocol with Arduino for motor control

---

## System Architecture

```
┌─────────────────┐
│  Camera Input   │
│  - Webcam       │
│  - ToF Sensor   │
└────────┬────────┘
         │ (camera_publisher)
         ▼
┌─────────────────┐
│ Image Processor │ ──► Detections (JSON)
│  - YOLO         │
│  - K-means      │
│  - Distance Est.│
└────────┬────────┘
         │ (image_sync_processor)
         ▼
┌─────────────────┐
│  Pan-Tilt Track │ ──► Servo Commands
│  - Leaf Tracker │ ──► dist_camera
└────────┬────────┘
         │ (pan_tilt_track)
         ▼
┌─────────────────┐
│  Camera to EE   │ ──► Transform Points
│  - TF2 Lookup   │
└────────┬────────┘
         │ (autonomous_robot)
         ▼
┌─────────────────┐
│   xArm Control  │ ──► Manipulation
│  - IK Solver    │
│  - Trajectory   │
└─────────────────┘
         │
         ▼
┌─────────────────┐
│  Mobile Base    │ ──► Navigation
│  - Diff Drive   │
└─────────────────┘
```

---

## Building the Workspace

```bash
cd /workspaces/jazzy_docker/autonomous_robot_gardener/ros_ws
colcon build
source install/setup.bash
```

### Build Individual Packages

```bash
# Python packages
colcon build --packages-select camera_publisher
colcon build --packages-select image_sync_processor
colcon build --packages-select pan_tilt_track
colcon build --packages-select autonomous_robot

# C++ packages
colcon build --packages-select pan_tilt_hardware
colcon build --packages-select xarm_hardware
colcon build --packages-select diffdrive_arduino
colcon build --packages-select xarm_description
colcon build --packages-select pan_tilt_description
```

---

## Quick Start

### 1. Start Camera Publishers

```bash
ros2 run camera_publisher webcam_publisher
ros2 run camera_publisher tof_publisher
```

### 2. Start Image Processing

```bash
ros2 run image_sync_processor image_sync_node
```

### 3. Start Leaf Tracking

```bash
ros2 launch pan_tilt_track leaf_tracker.launch.py
```

### 4. Start Camera-to-EE Transform

```bash
ros2 run autonomous_robot camera_to_ee
```

### 5. Start Robot Hardware (choose one)

#### xArm:
```bash
ros2 launch xarm_hardware xarm_hardware.launch.py
```

#### Diff Drive:
```bash
ros2 launch diffdrive_arduino diffbot.launch.py
```

---

## Configuration

### Camera Parameters
- Camera resolution: 320x240 (default)
- ToF range: 0-200cm
- Detection confidence threshold: 0.35
- Max tracking distance: 45cm

### Pan-Tilt Control
- Pan/tilt limits: ±1.57 rad (±90°)
- Scale factors: 0.2 (default)
- Deadzone: ±0.015 rad
- Control loop: 5s cycle

### Detection Pipeline
- Model: YOLO (custom trained)
- Classes: leaf, remote
- Distance calculation: Average of pixels <60cm (if >30% qualify)
- Fallback distance: 100cm

---

## Dependencies

### System Dependencies
- ROS 2 Jazzy
- Gazebo Harmonic
- OpenCV with Python bindings
- LibSerial

### Python Dependencies
- NumPy
- IKPy (inverse kinematics)
- JSON
- cv_bridge
- message_filters

### ROS 2 Dependencies
- `ros2_control`
- `gz_ros2_control`
- `controller_manager`
- `tf2_ros`
- `sensor_msgs`
- `geometry_msgs`

---

## Maintainers

- Jonathan Randall - jonnyrandall@live.com.au

## License

- Most packages: Apache-2.0
- camera_publisher, image_sync_processor: MIT

---

## Notes

- All packages are designed for ROS 2 Jazzy
- Hardware interfaces require appropriate USB/serial permissions
- Camera devices must be accessible at `/dev/video*` or appropriate paths
- TF tree must be properly configured for coordinate transformations
- YOLO model weights must be provided for image_sync_processor
