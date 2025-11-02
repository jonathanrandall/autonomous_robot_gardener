#ifndef PAN_TILT_HARDWARE__PAN_TILT_HARDWARE_HPP_
#define PAN_TILT_HARDWARE__PAN_TILT_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <cstdint>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace pan_tilt_hardware
{

// PCA9685 I2C driver class
class PCA9685
{
public:
  PCA9685(const std::string& i2c_device, int address = 0x40);
  ~PCA9685();

  void set_frequency(float freq);
  void set_pwm(uint8_t channel, uint16_t on, uint16_t off);

private:
  int file_;
  int address_;

  void write_byte(uint8_t reg, uint8_t value);
  uint8_t read_byte(uint8_t reg);
};

// Hardware interface for pan-tilt mount
class PanTiltHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(PanTiltHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parameters from URDF
  std::string i2c_device_;
  int i2c_address_;
  int pan_channel_;
  int tilt_channel_;
  uint16_t pan_min_pulse_;
  uint16_t pan_max_pulse_;
  uint16_t tilt_min_pulse_;
  uint16_t tilt_max_pulse_;
  float pwm_frequency_;

  // Store the command and state values for the two joints
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;

  // PCA9685 driver
  std::unique_ptr<PCA9685> pca9685_;

  // Helper function to convert angle to PWM pulse
  uint16_t angle_to_pulse(double angle, uint16_t min_pulse, uint16_t max_pulse);
};

}  // namespace pan_tilt_hardware

#endif  // PAN_TILT_HARDWARE__PAN_TILT_HARDWARE_HPP_
