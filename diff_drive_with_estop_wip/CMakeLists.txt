cmake_minimum_required(VERSION 3.8)
project(my_diffdrive_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(diff_drive_controller REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/diff_drive_with_estop.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  controller_interface
  hardware_interface
  diff_drive_controller
  pluginlib
  std_msgs
)

pluginlib_export_plugin_description_file(controller_interface plugin.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/ DESTINATION include)

install(FILES plugin.xml DESTINATION share/${PROJECT_NAME})

ament_package()

