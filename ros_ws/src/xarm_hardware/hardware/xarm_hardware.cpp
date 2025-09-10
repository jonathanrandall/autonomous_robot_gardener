// Copyright 2024 xArm Hardware Interface
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "xarm_hardware/serial_comms_esp32.hpp"
#include "xarm_hardware/xarm_hardware.hpp"
// #include "serial_comms.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <iostream>

namespace xarm_hardware
{

CallbackReturn XArmHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR(rclcpp::get_logger("XArmHardware"), 
               "XArm Hardware failed to initialize");
    return CallbackReturn::ERROR;
  }

  // Initialize joint data for 6-DOF xArm trajectory controller
  joint_position_commands_.assign(6, 0.0);    // Only position commands
  joint_position_states_.assign(6, 0.0);      // Position states
  joint_velocity_states_.assign(6, 0.0);      // Velocity states (calculated)



  // Store joint names
  for (const auto & joint : info_.joints)
  {
    joint_names_.push_back(joint.name);
  }

  // Initialize serial communication object
  serial_comms_ = std::make_unique<SerialComms>();

  RCLCPP_INFO(rclcpp::get_logger("XArmHardware"), 
               "XArm Hardware initialized with %zu joints", info_.joints.size());

  return CallbackReturn::SUCCESS;
}

CallbackReturn XArmHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("XArmHardware"), "Configuring XArm Hardware...");
  
  // Get serial configuration from hardware info
  std::string serial_port = info_.hardware_parameters["serial_port"];
  int baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  double timeout = std::stod(info_.hardware_parameters["timeout"]);
  
  // Temporarily disable serial communication for testing
  RCLCPP_INFO(rclcpp::get_logger("XArmHardware"), 
               "Serial config: %s, %d, %.2f (disabled for testing)", 
               serial_port.c_str(), baud_rate, timeout);
  
  
  if (!serial_comms_->init_serial(serial_port, baud_rate, timeout))
  {
    RCLCPP_ERROR(rclcpp::get_logger("XArmHardware"), "Failed to initialize serial connection");
    return CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("XArmHardware"), "Serial connection initialized successfully");
  }
  
  
  
  RCLCPP_INFO(rclcpp::get_logger("XArmHardware"), "Skipping ping test for now - controller should start");

  return CallbackReturn::SUCCESS;
}

CallbackReturn XArmHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("XArmHardware"), "Activating XArm Hardware...");
  
  // Initialize joint states to safe default values
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    joint_position_states_[i] = 0.0;  // Default to 0 radians
    joint_velocity_states_[i] = 0.0;  // Default to 0 rad/s
    joint_position_commands_[i] = 0.0; // Default to 0 radians
  }
  
  RCLCPP_INFO(rclcpp::get_logger("XArmHardware"), "XArm Hardware activated successfully");
  
  return CallbackReturn::SUCCESS;
}

CallbackReturn XArmHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("XArmHardware"), "Deactivating XArm Hardware...");
  
  // Stop all joint movements by setting commands to current positions
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    joint_position_commands_[i] = joint_position_states_[i];
  }
  
  RCLCPP_INFO(rclcpp::get_logger("XArmHardware"), "XArm Hardware deactivated successfully");
  
  return CallbackReturn::SUCCESS;
}

CallbackReturn XArmHardware::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("XArmHardware"), "Cleaning up XArm Hardware...");
  
  if (serial_comms_)
  {
    serial_comms_->stop();
  }
  
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> XArmHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < joint_names_.size(); i++)
  {
    // Position interface for each joint
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(joint_names_[i], hardware_interface::HW_IF_POSITION, &joint_position_states_[i]));
    
    // Velocity interface for each joint
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &joint_velocity_states_[i]));
  }

  RCLCPP_INFO(rclcpp::get_logger("XArmHardware"), 
               "Exported %zu state interfaces", state_interfaces.size());

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> XArmHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < joint_names_.size(); i++)
  {
    // Only position command interface for each joint (no velocity commands)
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(joint_names_[i], hardware_interface::HW_IF_POSITION, &joint_position_commands_[i]));
  }

  RCLCPP_INFO(rclcpp::get_logger("XArmHardware"), 
               "Exported %zu command interfaces", command_interfaces.size());

  return command_interfaces;
}

double hardware_to_ros(int ticks) {
    double min_hw = 0.0;
    double max_hw = 1000.0;
    double min_ros = -2.35619;// -0.75 * M_PI;
    double max_ros =  2.35619;//0.75 * M_PI;

    return min_ros + ( (ticks - min_hw) / (max_hw - min_hw) ) * (max_ros - min_ros);
}



return_type XArmHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // When serial communication is disabled, provide dummy data for testing
  // RCLCPP_INFO(rclcpp::get_logger("XArmHardware"), "Serial communication print, providing data for testing");

    
  bool testing = false;
  if (!testing){
  if (!serial_comms_->is_running())
  {
    RCLCPP_INFO(rclcpp::get_logger("XArmHardware"), "not connected");

    // Provide dummy joint positions for testing
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      joint_position_states_[i] = 0.0;  // Default to 0 radians
      joint_velocity_states_[i] = 0.0;  // Default to 0 rad/s
    }
    return return_type::OK;
  }

  // Read current joint positions from ESP32
  std::vector<double> current_positions = serial_comms_->read_joint_position();// serial_comms_->get_joint_positions();
  // current_positions = serial_comms_->read_joint_position();
  // RCLCPP_INFO(rclcpp::get_logger("XArmHardware"), "Serial communication print, providing data for testing");
  
  // Update joint positions and calculate velocities
  static std::vector<double> prev_position(6, 0.0);
  
  for (size_t i = 0; i < joint_names_.size() && i < current_positions.size(); ++i)
  {
    current_positions[i] = std::max(0.0, std::min(1000.0, current_positions[i])); // Clip to valid range
    current_positions[i] = hardware_to_ros(current_positions[i]); // Transform to radians
    // Update position state
    joint_position_states_[i] = (current_positions[i]);
    
    // Calculate velocity state
    joint_velocity_states_[i] = (current_positions[i] - prev_position[i]) / 0.01; // Assuming 10ms control loop
    
    // Store current position for next iteration
    prev_position[i] = current_positions[i];
  }
}

  return return_type::OK;
}

return_type XArmHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // When serial communication is disabled, simulate movement for testing
  // RCLCPP_INFO(rclcpp::get_logger("XArmHardware"), "Serial  write");
  if (!serial_comms_->is_running())
  {
    // Log position commands for debugging
    RCLCPP_INFO(rclcpp::get_logger("XArmHardware"), 
                  "Dummy mode - Position commands: [%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]",
                  joint_position_commands_[0], joint_position_commands_[1], joint_position_commands_[2],
                  joint_position_commands_[3], joint_position_commands_[4], joint_position_commands_[5]);
    
    // In dummy mode, simulate movement by gradually updating joint positions
   
    
    return return_type::OK;
  }


  bool success = set_joint_positions(joint_position_commands_);
  // serial_comms_->write_position();

  // RCLCPP_INFO(rclcpp::get_logger("XArmHardware"), "Serial  write 3");
  
  if (!success)
  {
    RCLCPP_WARN(rclcpp::get_logger("XArmHardware"), "Failed to send joint position commands to ESP32");
    return return_type::ERROR;
  }

  // RCLCPP_INFO(rclcpp::get_logger("XArmHardware"), "Serial  write 4");

  // Log position commands for debugging
  // RCLCPP_INFO(rclcpp::get_logger("XArmHardware"), 
  //               "Position commands: [%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]",
  //               joint_position_commands_[0], joint_position_commands_[1], joint_position_commands_[2],
  //               joint_position_commands_[3], joint_position_commands_[4], joint_position_commands_[5]);

  return return_type::OK;
}

bool XArmHardware::set_joint_positions(const std::vector<double>& positions)
{
  if (positions.size() != 6)
  {
    RCLCPP_ERROR(rclcpp::get_logger("XArmHardware"), 
                  "Expected 6 joint positions, got %zu", positions.size());
    return false;
  }
  
  // Transform positions from radians to 0-1000 range for ESP32
  std::vector<double> transformed_positions = transform_positions_for_esp32(positions);
  
  // Delegate to SerialComms
  // RCLCPP_INFO(rclcpp::get_logger("XArmHardware"), 
  //               "Transformed positions for ESP32: [%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]",
  //               transformed_positions[0], transformed_positions[1], transformed_positions[2],
  //               transformed_positions[3], transformed_positions[4], transformed_positions[5]);
  return serial_comms_->set_latest_command(transformed_positions);
}

std::vector<double> XArmHardware::transform_positions_for_esp32(const std::vector<double>& positions)
{
  std::vector<double> transformed(positions.size());
  
  // Transform from radians to 0-1000 range
  // xArm joint limits: -2.35619 to 2.35619 radians (-135° to 135°)
  const double min_rad = -2.35619;
  const double max_rad = 2.35619;
  const double out_max = 1000.0;
  
  for (size_t i = 0; i < positions.size(); ++i)
  {
    // Scale from radians to 0-1000 range
    double scaled = (positions[i] - min_rad) / (max_rad - min_rad) * out_max;
    // Clip to valid range
    scaled = std::max(0.0, std::min(out_max, scaled));
    transformed[i] = scaled;
  }
  
  return transformed;
}


std::vector<double> XArmHardware::get_joint_positions()
{
  // Delegate to SerialComms
  return serial_comms_->get_joint_positions();
}

}  // namespace xarm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  xarm_hardware::XArmHardware, hardware_interface::SystemInterface)
