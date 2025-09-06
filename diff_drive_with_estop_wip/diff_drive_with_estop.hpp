#pragma once

#include "diff_drive_controller/diff_drive_controller.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rclcpp/rclcpp.hpp"

namespace my_diffdrive_controller
{

class DiffDriveWithEStop : public diff_drive_controller::DiffDriveController
{
public:
  controller_interface::CallbackReturn on_init() override;

  controller_interface::return_type update() override;

private:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr e_stop_sub_;
  bool e_stop_active_{false};
};

}  // namespace my_diffdrive_controller

