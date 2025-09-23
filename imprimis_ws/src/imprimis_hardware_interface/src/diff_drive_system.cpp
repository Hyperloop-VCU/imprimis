// diff_drive_system.cpp
#include <cmath>
#include <vector>
#include <string>
#include <memory>
#include <algorithm>

#include "diff_drive_hw/diff_drive_system.hpp"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

class DiffDriveSystem : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
      return CallbackReturn::ERROR;
    }

    // Expected joints: left_wheel_joint, right_wheel_joint
    if (info.joints.size() != 2) {
      RCLCPP_ERROR(rclcpp::get_logger("DiffDriveSystem"),
                   "Expected exactly 2 joints, got %zu", info.joints.size());
      return CallbackReturn::ERROR;
    }

    // Read parameters (with defaults)
    wheel_radius_     = get_param_double("wheel_radius", 0.05);       // [m]
    wheel_separation_ = get_param_double("wheel_separation", 0.30);   // [m]
    max_wheel_speed_  = get_param_double("max_wheel_speed", 20.0);    // [rad/s]
    cmd_timeout_s_    = get_param_double("cmd_timeout", 0.5);         // [s] safety

    // Init state/command arrays: [left, right]
    pos_.assign(2, 0.0);
    vel_.assign(2, 0.0);
    cmd_vel_.assign(2, 0.0);

    last_cmd_time_ = 0.0;

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> state_ifaces;
    // joint 0 -> position, velocity
    state_ifaces.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_POSITION, &pos_[0]);
    state_ifaces.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &vel_[0]);
    // joint 1 -> position, velocity
    state_ifaces.emplace_back(info_.joints[1].name, hardware_interface::HW_IF_POSITION, &pos_[1]);
    state_ifaces.emplace_back(info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &vel_[1]);
    return state_ifaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> cmd_ifaces;
    // Command wheel angular velocity [rad/s]
    cmd_ifaces.emplace_back(info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &cmd_vel_[0]);
    cmd_ifaces.emplace_back(info_.joints[1].name, hardware_interface::HW_IF_VELOCITY, &cmd_vel_[1]);
    return cmd_ifaces;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    // Zero everything on activate
    std::fill(pos_.begin(), pos_.end(), 0.0);
    std::fill(vel_.begin(), vel_.end(), 0.0);
    std::fill(cmd_vel_.begin(), cmd_vel_.end(), 0.0);
    last_cmd_time_ = 0.0;
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveSystem"), "Activated");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveSystem"), "Deactivated");
    return CallbackReturn::SUCCESS;
  }

  // READ: pull latest states from hardware (here we simulate simple integration)
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override
  {
    const double dt = period.seconds();

    // In a real driver: read encoders -> update pos_/vel_
    // Here: integrate last commanded velocity
    for (size_t i = 0; i < 2; ++i) {
      vel_[i] = simulated_velocity_[i];         // whatever HW reported last cycle
      pos_[i] += vel_[i] * dt;                  // integrate
    }

    // Track time for command timeout logic
    current_time_s_ = time.seconds();

    return return_type::OK;
  }

  // WRITE: send commands to hardware
  return_type write(const rclcpp::Time & time, const rclcpp::Duration &) override
  {
    // Clamp commands and apply timeout
    const double now = time.seconds();

    // If no new command for too long, stop
    const bool timed_out = (now - last_cmd_time_) > cmd_timeout_s_;

    for (size_t i = 0; i < 2; ++i) {
      double v = cmd_vel_[i];
      v = std::clamp(v, -max_wheel_speed_, max_wheel_speed_);
      simulated_velocity_[i] = timed_out ? 0.0 : v;

      // In a real driver: convert rad/s to bus units and write to motor controller
      // e.g., can_interface_.sendVelocityCommand(i, v);
    }

    // Mark we’ve “consumed” the command at this time
    last_cmd_time_ = now;

    return return_type::OK;
  }

private:
  // Helpers
  double get_param_double(const std::string & key, double def) const
  {
    auto it = info_.hardware_parameters.find(key);
    if (it == info_.hardware_parameters.end()) return def;
    try { return std::stod(it->second); }
    catch (...) { return def; }
  }

  // Params
  double wheel_radius_{0.05};
  double wheel_separation_{0.30};
  double max_wheel_speed_{20.0};
  double cmd_timeout_s_{0.5};

  // State (left=0, right=1)
  std::vector<double> pos_, vel_, cmd_vel_;

  // Backend / transport placeholders
  double simulated_velocity_[2]{0.0, 0.0};

  // Timing
  double last_cmd_time_{0.0};
  double current_time_s_{0.0};
};

PLUGINLIB_EXPORT_CLASS(DiffDriveSystem, hardware_interface::SystemInterface)
