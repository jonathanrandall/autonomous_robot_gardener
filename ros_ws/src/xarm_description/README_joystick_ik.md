# XArm Joystick IK Control

This package provides a joystick-based inverse kinematics (IK) control interface for the XArm robot arm.

## Features

- **Joystick Control**: Use a USB joystick to control the XArm end-effector position
- **Real-time IK**: Continuous inverse kinematics calculation as you move the joystick
- **Safety Limits**: Automatic position adjustment to prevent the arm from reaching unsafe positions
- **Visual Feedback**: Real-time display of current position, joint angles, and safety status

## Joystick Mapping

- **Axis 2**: X position control (left/right)
- **Axis 3**: Y position control (forward/backward)  
- **Button 6**: Z position up
- **Button 7**: Z position down

## Configuration

The joystick configuration is stored in `config/joystick_ik.yaml` and includes:

- Axis and button mappings
- Position increment per input (default: 0.01)
- Update rate (default: 200ms)
- Safety limits (default: max position norm 0.36)
- Position adjustment factors

## Usage

### 1. Launch the Joystick IK GUI

```bash
ros2 launch xarm_description joystick_ik_gui.launch.py
```

This will launch:
- `joy_node` - for joystick input
- `joystick_ik_gui` - the main control interface

### 2. Control the Arm

- Move the joystick axes 2 and 3 to control X and Y position
- Press buttons 6 and 7 to move the Z position up and down
- The GUI will show real-time position updates and calculated joint angles
- Use the "Send Position" button to send the current position to the robot controller
- Use the "Reset Position" button to return to the default position

### 3. Safety Features

- **Position Norm Limit**: If the position norm exceeds 0.36, the system automatically scales down the position
- **Adaptive Adjustment**: For X/Y movements that exceed limits, Z is reduced; for Z movements that exceed limits, X/Y are reduced
- **Visual Indicators**: The position norm display turns red when safety limits are exceeded

## Testing

To test joystick input without the full GUI:

```bash
ros2 run xarm_description test_joystick.py
```

This will display raw joystick messages to verify your joystick is working correctly.

## Requirements

- ROS2 Humble
- `joy` package for joystick input
- `sensor_msgs` for joystick message types
- `ikpy` for inverse kinematics calculations
- USB joystick/gamepad

## Troubleshooting

### Joystick Not Detected
- Ensure your joystick is connected and recognized by the system
- Check that the `joy` package is installed: `sudo apt install ros-humble-joy`
- Verify device permissions: `ls -l /dev/input/js*`

### IK Calculation Fails
- Check that the URDF file path is correct in the launch file
- Ensure the IK chain is properly configured
- Verify that the target position is within the robot's reachable workspace

### Position Not Updating
- Check that the joystick is sending messages (use the test script)
- Verify the axis and button mappings in the config file
- Ensure the GUI is fully initialized before moving the joystick

## Customization

You can modify the joystick behavior by editing `config/joystick_ik.yaml`:

- Change axis and button mappings
- Adjust position increment sensitivity
- Modify safety limits and adjustment factors
- Change the update rate

## Architecture

The system consists of:

1. **Joystick Input**: `joy_node` publishes `sensor_msgs/Joy` messages
2. **Position Processing**: Continuous position updates based on joystick input
3. **Safety System**: Automatic position adjustment to maintain safe limits
4. **IK Calculation**: Real-time inverse kinematics using the `ikpy` library
5. **GUI Display**: Tkinter-based interface showing current status
6. **Trajectory Publishing**: Sends calculated joint positions to the robot controller
