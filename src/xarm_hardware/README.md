# xArm Hardware Interface

This package provides a ROS 2 hardware interface for the xArm robot using libserial communication with an ESP32 controller.

## Features

- **Serial Communication**: Uses libserial to communicate with ESP32 via USB serial
- **6-DOF Control**: Supports 6 joint position and velocity interfaces
- **Force/Torque Sensing**: Includes force-torque sensor interfaces
- **Real-time Control**: Designed for real-time control loops
- **Error Handling**: Comprehensive error handling and logging

## Dependencies

### System Dependencies
```bash
# Install libserial
sudo apt-get install libserial-dev

# Or build from source
git clone https://github.com/crayzeewulf/libserial.git
cd libserial
mkdir build && cd build
cmake ..
make
sudo make install
```

### ROS 2 Dependencies
- `hardware_interface`
- `pluginlib`
- `rclcpp`
- `rclcpp_lifecycle`

## Building

```bash
# In your workspace
cd src/xarm_description/hardware
colcon build --packages-select xarm_hardware
```

## Usage

### 1. Hardware Configuration

Add the hardware interface to your robot's URDF configuration:

```xml
<ros2_control name="xarm_control" type="system">
  <hardware>
    <plugin>xarm_hardware/XArmHardware</plugin>
  </hardware>
  
  <param name="serial_port">/dev/ttyUSB0</param>
  <param name="baud_rate">115200</param>
  <param name="timeout">0.1</param>
  
  <!-- Joint interfaces -->
  <joint name="joint1">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <!-- Repeat for joints 2-6 -->
</ros2_control>
```

### 2. ESP32 Communication Protocol

The ESP32 should implement these commands:

- `ping` → responds with `pong`
- `get_positions()` → responds with `[pos1, pos2, pos3, pos4, pos5, pos6]>>>`
- `set_positions([pos1, pos2, pos3, pos4, pos5, pos6], time_ms)` → executes movement

### 3. Running the Controller

```bash
# Launch your robot
ros2 launch your_robot_launch.py

# Or manually start the controller
ros2 control load_controller joint_trajectory_controller
ros2 control set_controller_state joint_trajectory_controller active
```

## API Reference

### Core Methods

- `send_msg(command)`: Sends a command string to the ESP32
- `read_response()`: Reads and parses the ESP32 response
- `set_joint_positions(positions)`: Sends joint position commands
- `get_joint_positions()`: Retrieves current joint positions

### Communication Protocol

- **Baud Rate**: 115200 (configurable)
- **Timeout**: 100ms default (configurable)
- **End of Message**: `\r\n`
- **Response End**: `>>>`

## Troubleshooting

### Common Issues

1. **Serial Port Not Found**
   ```bash
   # Check available ports
   ls /dev/ttyUSB*
   ls /dev/ttyACM*
   
   # Set permissions
   sudo chmod 666 /dev/ttyUSB0
   ```

2. **Communication Timeout**
   - Check ESP32 is responding
   - Verify baud rate matches
   - Check cable connections

3. **Build Errors**
   ```bash
   # Ensure libserial is installed
   pkg-config --exists libserial
   
   # Check library path
   ldconfig -p | grep libserial
   ```

### Debug Mode

Enable debug logging:
```bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}]: {message}"
export RCUTILS_LOGGING_USE_STDOUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=1
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## License

This project is licensed under the Apache License 2.0 - see the LICENSE file for details.
