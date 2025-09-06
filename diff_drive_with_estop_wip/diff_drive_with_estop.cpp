#include "my_diffdrive_controller/diff_drive_with_estop.hpp"

namespace my_diffdrive_controller
{

controller_interface::CallbackReturn DiffDriveWithEStop::on_init()
{
  auto ret = diff_drive_controller::DiffDriveController::on_init();

  if (ret != controller_interface::CallbackReturn::SUCCESS) {
    return ret;
  }

  // Create emergency stop subscription
  try {
    auto node = get_node();
    e_stop_sub_ = node->create_subscription<std_msgs::msg::Bool>(
      "~/emergency_stop", rclcpp::SystemDefaultsQoS(),
      [this](std_msgs::msg::Bool::SharedPtr msg) {
        e_stop_active_ = msg->data;
        if (e_stop_active_) {
          RCLCPP_WARN(get_node()->get_logger(), "Emergency stop activated!");
        } else {
          RCLCPP_INFO(get_node()->get_logger(), "Emergency stop cleared.");
        }
      });
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to create e-stop subscriber: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type DiffDriveWithEStop::update()
{
  if (e_stop_active_) {
    // Zero wheel commands directly at hardware interface level
    for (auto & wheel : registered_left_wheel_handles_) {
      wheel.set_command(0.0);
    }
    for (auto & wheel : registered_right_wheel_handles_) {
      wheel.set_command(0.0);
    }
    return controller_interface::return_type::OK;
  }

  // Normal diff_drive behavior
  return diff_drive_controller::DiffDriveController::update();
}

}  // namespace my_diffdrive_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_diffdrive_controller::DiffDriveWithEStop, controller_interface::ControllerInterface)

