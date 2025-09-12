cmake_minimum_required(VERSION 3.5)
project(xarm_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config meshes launch worlds urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install the Python package
install(
  DIRECTORY xarm_description
  DESTINATION lib/python3.10/site-packages
)

# Install the Python script as an executable
install(
  PROGRAMS scripts/joint_slider_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install the Python script as an executable


install(
  PROGRAMS scripts/min_joy_listen_v2.py
  DESTINATION lib/${PROJECT_NAME}
)

# install(
#   PROGRAMS scripts/test_joy.py
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()

