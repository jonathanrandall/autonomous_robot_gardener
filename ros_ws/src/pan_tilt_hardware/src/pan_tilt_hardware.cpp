#include "pan_tilt_hardware/pan_tilt_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pan_tilt_hardware
{

// PCA9685 register addresses
constexpr uint8_t PCA9685_MODE1 = 0x00;
constexpr uint8_t PCA9685_PRESCALE = 0xFE;
constexpr uint8_t PCA9685_LED0_ON_L = 0x06;

// ==================== PCA9685 Implementation ====================

PCA9685::PCA9685(const std::string& i2c_device, int address)
  : file_(-1), address_(address)
{
  file_ = open(i2c_device.c_str(), O_RDWR);
  if (file_ < 0) {
    throw std::runtime_error("Failed to open I2C device: " + i2c_device);
  }

  if (ioctl(file_, I2C_SLAVE, address_) < 0) {
    close(file_);
    throw std::runtime_error("Failed to acquire bus access for I2C address");
  }

  // Reset the device
  write_byte(PCA9685_MODE1, 0x00);
  usleep(5000);

  // Set auto-increment mode (bit 5)
  uint8_t mode = read_byte(PCA9685_MODE1);
  mode |= 0x20;  // Set AI (auto-increment) bit
  write_byte(PCA9685_MODE1, mode);
  usleep(5000);
}

PCA9685::~PCA9685()
{
  if (file_ >= 0) {
    close(file_);
  }
}

void PCA9685::write_byte(uint8_t reg, uint8_t value)
{
  uint8_t buffer[2] = {reg, value};
  if (write(file_, buffer, 2) != 2) {
    RCLCPP_ERROR(rclcpp::get_logger("PCA9685"), "Failed to write to I2C device");
  }
}

uint8_t PCA9685::read_byte(uint8_t reg)
{
  if (write(file_, &reg, 1) != 1) {
    RCLCPP_ERROR(rclcpp::get_logger("PCA9685"), "Failed to write register address");
    return 0;
  }
  uint8_t value;
  if (read(file_, &value, 1) != 1) {
    RCLCPP_ERROR(rclcpp::get_logger("PCA9685"), "Failed to read from I2C device");
    return 0;
  }
  return value;
}

void PCA9685::set_frequency(float freq)
{
  // Calculate prescale value
  float prescale_val = 25000000.0;    // 25MHz
  prescale_val /= 4096.0;             // 12-bit
  prescale_val /= freq;
  prescale_val -= 1.0;

  uint8_t prescale = static_cast<uint8_t>(floor(prescale_val + 0.5));

  uint8_t old_mode = read_byte(PCA9685_MODE1);
  uint8_t new_mode = (old_mode & 0x7F) | 0x10;  // Sleep
  write_byte(PCA9685_MODE1, new_mode);
  write_byte(PCA9685_PRESCALE, prescale);
  write_byte(PCA9685_MODE1, old_mode);
  usleep(5000);
  write_byte(PCA9685_MODE1, old_mode | 0x80);  // Restart
}

void PCA9685::set_pwm(uint8_t channel, uint16_t on, uint16_t off)
{
  uint8_t base_reg = PCA9685_LED0_ON_L + 4 * channel;
  write_byte(base_reg, on & 0xFF);
  write_byte(base_reg + 1, on >> 8);
  write_byte(base_reg + 2, off & 0xFF);
  write_byte(base_reg + 3, off >> 8);
}

// ==================== PanTiltHardware Implementation ====================

hardware_interface::CallbackReturn PanTiltHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get parameters from URDF
  i2c_device_ = info_.hardware_parameters["i2c_device"];
  i2c_address_ = std::stoi(info_.hardware_parameters["i2c_address"]);
  pan_channel_ = std::stoi(info_.hardware_parameters["pan_channel"]);
  tilt_channel_ = std::stoi(info_.hardware_parameters["tilt_channel"]);
  pan_min_pulse_ = std::stoi(info_.hardware_parameters["pan_min_pulse"]);
  pan_max_pulse_ = std::stoi(info_.hardware_parameters["pan_max_pulse"]);
  tilt_min_pulse_ = std::stoi(info_.hardware_parameters["tilt_min_pulse"]);
  tilt_max_pulse_ = std::stoi(info_.hardware_parameters["tilt_max_pulse"]);
  pwm_frequency_ = std::stof(info_.hardware_parameters["pwm_frequency"]);

  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // PanTiltHardware has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("PanTiltHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("PanTiltHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("PanTiltHardware"),
        "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("PanTiltHardware"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PanTiltHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("PanTiltHardware"), "Configuring ...please wait...");

  // Initialize PCA9685
  try {
    pca9685_ = std::make_unique<PCA9685>(i2c_device_, i2c_address_);
    pca9685_->set_frequency(pwm_frequency_);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(
      rclcpp::get_logger("PanTiltHardware"),
      "Failed to initialize PCA9685: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Set default position (90 degrees for both joints)
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_states_[i] = 0.0;  // radians (0 rad = 90 degrees if centered)
    hw_commands_[i] = 0.0;
  }

  RCLCPP_INFO(rclcpp::get_logger("PanTiltHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> PanTiltHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> PanTiltHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn PanTiltHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("PanTiltHardware"), "Activating ...please wait...");

  // Set initial servo positions to center (0 radians = 90 degrees)
  for (uint i = 0; i < hw_commands_.size(); i++)
  {
    if (std::isnan(hw_commands_[i]))
    {
      hw_commands_[i] = 0.0;
    }
    if (std::isnan(hw_states_[i]))
    {
      hw_states_[i] = 0.0;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("PanTiltHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PanTiltHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("PanTiltHardware"), "Deactivating ...please wait...");

  RCLCPP_INFO(rclcpp::get_logger("PanTiltHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type PanTiltHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Since PWM servos don't provide feedback, we just report the last commanded position
  // The state is updated in write()
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PanTiltHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Write commands to the servos
  for (uint i = 0; i < hw_commands_.size(); i++)
  {
    // Update state to match command (since we have no feedback)
    hw_states_[i] = hw_commands_[i];

    // Convert radians to degrees, then to PWM pulse
    double angle_deg = (hw_commands_[i] * 180.0 / M_PI) + 90.0;  // Convert to 0-180 range

    uint16_t pulse;
    if (i == 0) {  // pan_joint (first joint)
      pulse = angle_to_pulse(angle_deg, pan_min_pulse_, pan_max_pulse_);
      pca9685_->set_pwm(pan_channel_, 0, pulse);
    } else {  // tilt_joint (second joint)
      pulse = angle_to_pulse(angle_deg, tilt_min_pulse_, tilt_max_pulse_);
      pca9685_->set_pwm(tilt_channel_, 0, pulse);
    }
  }

  return hardware_interface::return_type::OK;
}

uint16_t PanTiltHardware::angle_to_pulse(double angle, uint16_t min_pulse, uint16_t max_pulse)
{
  // Clamp angle to 0-180 range
  if (angle < 0.0) angle = 0.0;
  if (angle > 180.0) angle = 180.0;

  // Map angle (0-180) to pulse width
  uint16_t pulse = min_pulse + static_cast<uint16_t>((angle / 180.0) * (max_pulse - min_pulse));
  return pulse;
}

}  // namespace pan_tilt_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  pan_tilt_hardware::PanTiltHardware, hardware_interface::SystemInterface)
