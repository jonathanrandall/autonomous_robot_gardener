cmake_minimum_required(VERSION 3.5)
project(xarm_description)

find_package(ament_cmake REQUIRED)

# Install robot description files
install(
  DIRECTORY config meshes launch worlds urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
