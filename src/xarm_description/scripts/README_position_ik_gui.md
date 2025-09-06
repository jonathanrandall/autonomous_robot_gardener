# Position IK GUI for XArm

This GUI allows you to control the XArm robot using position-based inverse kinematics. Instead of directly controlling joint angles, you can specify the desired end-effector position (x, y, z) and the system will automatically calculate the required joint angles.

## Features

- **Position Sliders**: Control x, y, z positions of the end-effector
- **Real-time IK Calculation**: Automatically calculates inverse kinematics as you move the sliders
- **Joint Angle Display**: Shows the calculated joint angles for all 6 joints
- **Direct Controller Integration**: Sends calculated joint positions directly to the joint trajectory controller
- **Reset Functionality**: Reset to default position with one click

## Requirements

- ROS 2 (tested with Humble)
- Python packages: `ikpy`, `numpy`, `tkinter`
- XArm robot description and URDF files
- Joint trajectory controller running

## Usage

### Method 1: Direct Python Execution
```bash
cd project_ws/src/xarm_description/scripts
python3 position_ik_gui.py
```

### Method 2: Using Launch File
```bash
cd project_ws
source install/setup.bash
ros2 launch xarm_description position_ik_gui.launch.py
```

### Method 3: Using ROS2 Run
```bash
cd project_ws
source install/setup.bash
ros2 run xarm_description position_ik_gui.py
```

## How It Works

1. **Position Input**: Use the x, y, z sliders to specify the desired end-effector position
2. **IK Calculation**: The system automatically calculates inverse kinematics using the ikpy library
3. **Joint Display**: The calculated joint angles are displayed in real-time
4. **Controller Integration**: Click "Send Position" to send the calculated joint positions to the joint trajectory controller

## Default Position

The default position is set to `[0.0, 0.2, -0.1]` which corresponds to:
- X: 0.0 (center)
- Y: 0.2 (forward)
- Z: -0.1 (down)

## Position Ranges

The sliders are configured with the following ranges:
- X: -0.5 to 0.5 meters
- Y: -0.5 to 0.5 meters  
- Z: -0.5 to 0.5 meters

## Joint Names

The system controls the following 6 joints (matching the IK chain active links):
- `xarm_1_joint`
- `xarm_2_joint`
- `xarm_3_joint`
- `xarm_4_joint` (automatically inverted as per your test script)
- `xarm_5_joint`
- `xarm_6_joint`

## Controller Topic

The GUI publishes to the joint trajectory controller topic:
```
/hiwonder_xarm_controller/joint_trajectory
```

## Troubleshooting

### IK Calculation Fails
- Check that the URDF file exists at `../urdf/xarm_v2_ik_final.urdf` or `../urdf/xarm_v2.urdf`
- Ensure the target position is within the robot's reachable workspace
- Check the console for detailed error messages

### Controller Not Responding
- Verify that the joint trajectory controller is running
- Check that the controller topic name matches your setup
- Ensure the robot is in a safe state before sending commands

### GUI Not Starting
- Make sure all Python dependencies are installed
- Check that ROS 2 environment is properly sourced
- Verify the script has execute permissions

## Dependencies

The following Python packages are required:
- `rclpy` - ROS 2 Python client library
- `trajectory_msgs` - ROS 2 trajectory message types
- `std_msgs` - ROS 2 standard message types
- `ikpy` - Inverse kinematics library
- `numpy` - Numerical computing library
- `tkinter` - GUI framework (usually included with Python)

## File Structure

```
project_ws/src/xarm_description/
├── scripts/
│   ├── position_ik_gui.py          # Main GUI script
│   └── README_position_ik_gui.md   # This file
├── launch/
│   └── position_ik_gui.launch.py   # Launch file
├── urdf/
│   ├── xarm_v2.urdf               # Source URDF
│   └── xarm_v2_ik_final.urdf      # Generated URDF for IK
└── package.xml                     # Package dependencies
```
