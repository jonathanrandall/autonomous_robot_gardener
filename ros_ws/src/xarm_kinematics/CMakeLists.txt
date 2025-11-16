cmake_minimum_required(VERSION 3.5)
project(xarm_kinematics)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclpy REQUIRED)

# Generate action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ArmPickup.action"
  DEPENDENCIES geometry_msgs
)

# Install Python module directory
install(DIRECTORY xarm_kinematics/
  DESTINATION lib/python3.12/site-packages/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# Install Python executables
install(PROGRAMS
  xarm_kinematics/ik_vertical_angle_node.py
  xarm_kinematics/ik_arm_pickup.py
  xarm_kinematics/joint_slider_gui.py
  xarm_kinematics/min_joy_listen_v2.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
