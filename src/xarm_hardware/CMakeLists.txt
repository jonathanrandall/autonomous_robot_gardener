cmake_minimum_required(VERSION 3.16)
project(xarm_hardware LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  xarm_hardware
  SHARED
  hardware/xarm_hardware.cpp
  # hardware/serial_comms.cpp
)
target_compile_features(xarm_hardware PUBLIC cxx_std_17)
target_include_directories(xarm_hardware PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/xarm_hardware>
)
ament_target_dependencies(
  xarm_hardware PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(xarm_hardware PUBLIC serial)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "xarm_hardware_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface xarm_hardware.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/xarm_hardware
)
# install(
#   DIRECTORY description/launch description/ros2_control description/urdf description/rviz
#   DESTINATION share/xarm_hardware
# )
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/xarm_hardware
)
install(TARGETS xarm_hardware
  EXPORT export_xarm_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_xarm_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()