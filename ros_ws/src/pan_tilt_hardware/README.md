# Pan-Tilt Hardware Interface

ROS2 Control hardware interface for PCA9685-based pan-tilt camera mount.

## Overview

This package provides a ROS2 Control hardware interface for controlling a pan-tilt servo mount connected via a PCA9685 PWM controller on a Raspberry Pi.

## Hardware Requirements

- Raspberry Pi with I2C enabled
- PCA9685 PWM/Servo Controller
- Two servos (pan and tilt)
- I2C connection to PCA9685 (typically `/dev/i2c-1`)

## Features

- Full ROS2 Control integration
- Configurable servo channels and PWM pulse ranges
- Position-based control interface
- Joint state broadcasting

## Configuration

The hardware interface can be configured via the URDF xacro file. Default parameters:

- **i2c_device**: `/dev/i2c-1`
- **i2c_address**: `64` (0x40)
- **pan_channel**: `0` (PCA9685 channel for pan servo)
- **tilt_channel**: `1` (PCA9685 channel for tilt servo)
- **pan_min_pulse**: `150` (PWM pulse for 0°)
- **pan_max_pulse**: `600` (PWM pulse for 180°)
- **tilt_min_pulse**: `150` (PWM pulse for 0°)
- **tilt_max_pulse**: `600` (PWM pulse for 180°)
- **pwm_frequency**: `50` Hz

You can customize these in your robot's URDF by passing parameters to the xacro macro:

```xml
<xacro:include filename="$(find pan_tilt_hardware)/urdf/pan_tilt.urdf.xacro"/>
<xacro:pan_tilt_system
  name="pan_tilt_system"
  pan_channel="3"
  tilt_channel="4"
  pan_min_pulse="200"
  pan_max_pulse="500"/>
```

## Building

```bash
cd /robot_gardener/ros_pi_ws
colcon build --packages-select pan_tilt_hardware
source install/setup.bash
```

## Usage

### Launch the controller manager and controllers:

```bash
ros2 launch pan_tilt_hardware pan_tilt_control.launch.py
```

### Send position commands:

```bash
# Command both joints (pan and tilt) in radians
ros2 topic pub /pan_tilt_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5, -0.3]"
```

The position values are in radians:
- `0.0` = center position (90°)
- `-1.57` = -90° (min position)
- `+1.57` = +90° (max position)

### Monitor joint states:

```bash
ros2 topic echo /joint_states
```

## Joint Names

- `pan_joint` - Horizontal rotation
- `tilt_joint` - Vertical rotation

## Permissions

You may need to add your user to the i2c group:

```bash
sudo usermod -a -G i2c $USER
```

Then log out and back in for the changes to take effect.

## Troubleshooting

### I2C Device Not Found
- Ensure I2C is enabled on your Raspberry Pi: `sudo raspi-config` → Interface Options → I2C
- Check if the device appears: `ls /dev/i2c-*`
- Verify PCA9685 address: `i2cdetect -y 1`

### Permission Denied
- Add user to i2c group (see Permissions section)
- Or run with sudo (not recommended for production)

### Servos Not Moving
- Verify correct channel numbers in configuration
- Check PWM pulse range matches your servos
- Ensure servos are powered externally (PCA9685 cannot power servos from I2C)
