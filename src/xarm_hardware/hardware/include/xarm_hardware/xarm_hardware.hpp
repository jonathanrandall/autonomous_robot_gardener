#ifndef XARM_HARDWARE_HPP_
#define XARM_HARDWARE_HPP_

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

// Include SerialComms class
#include "serial_comms_esp32.hpp"

using hardware_interface::return_type;

namespace xarm_hardware
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class XArmHardware : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  // Joint control methods (delegated to SerialComms)
  bool set_joint_positions(const std::vector<double>& positions);
  std::vector<double> get_joint_positions();

private:
  // Transform joint positions from radians to ESP32 servo range (0-1000)
  std::vector<double> transform_positions_for_esp32(const std::vector<double>& positions);
  
  // Serial communication object
  std::unique_ptr<SerialComms> serial_comms_;
  
  // Joint data for trajectory joint controller
  std::vector<double> joint_position_commands_;    // Only position commands (no velocity commands)
  std::vector<double> joint_position_states_;      // Position states from hardware
  std::vector<double> joint_velocity_states_;      // Velocity states (calculated from position changes)
  
  // Joint names
  std::vector<std::string> joint_names_;
};

}  // namespace xarm_hardware

#endif  // XARM_HARDWARE_HPP_
