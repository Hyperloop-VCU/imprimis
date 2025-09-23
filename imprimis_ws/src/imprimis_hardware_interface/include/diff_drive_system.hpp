#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"

class DiffDriveSystem : public hardware_interface::SystemInterface
{
public:
  using CallbackReturn = hardware_interface::CallbackReturn;
  using return_type    = hardware_interface::return_type;

  // ---- Lifecycle ----
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // ---- I/O loop ----
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ---- Helpers ----
  double get_param_double(const std::string & key, double def) const;

  // ---- Parameters ----
  double wheel_radius_{0.05};       // [m]
  double wheel_separation_{0.30};   // [m]
  double max_wheel_speed_{20.0};    // [rad/s]
  double cmd_timeout_s_[1]{0.5};    // [s]

  // ---- State (left=0, right=1) ----
  std::vector<double> pos_;      // HW_IF_POSITION (rad)
  std::vector<double> vel_;      // HW_IF_VELOCITY (rad/s)
  std::vector<double> cmd_vel_;  // command: HW_IF_VELOCITY (rad/s)

  // ---- Backend / transport placeholders ----
  double simulated_velocity_[2]{0.0, 0.0};

  // ---- Timing ----
  double last_cmd_time_{0.0};
  double current_time_s_{0.0};
};
