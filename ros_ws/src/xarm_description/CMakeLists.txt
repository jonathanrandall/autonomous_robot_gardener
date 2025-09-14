cmake_minimum_required(VERSION 3.5)
project(xarm_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY config meshes launch worlds urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install the Python package correctly for the active Python version
ament_python_install_package(xarm_description)

# Install the Python script as executables
install(
  PROGRAMS scripts/joint_slider_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/min_joy_listen_v2.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
