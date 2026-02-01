// Copyright 2021 ros2_control Development Team
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

#ifndef imprimis_hardware_platform__DIFFBOT_SYSTEM_HPP_
#define imprimis_hardware_platform__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "imprimis_hardware_platform/serial_comms.hpp"

namespace imprimis_hardware_platform
{
class DiffBotSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffBotSystemHardware);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override {
      auto x = previous_state; // prevent compile warnings
      return hardware_interface::CallbackReturn::SUCCESS;
    }

  

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

  /// Get the logger of the SystemInterface.
  /**
   * \return logger of the SystemInterface.
   */
  rclcpp::Logger get_logger() const { return *logger_; }

  /// Get the clock of the SystemInterface.
  /**
   * \return clock of the SystemInterface.
   */
  rclcpp::Clock::SharedPtr get_clock() const { return clock_; }

private:

  // Parameters (TODO remove these)
  double hw_start_sec_;
  double hw_stop_sec_;

  // Comms link with microcontroller
  std::shared_ptr<SerialLink> esp32;

  // debugging flags
  bool PRINT_READ_STATES = false;
  bool PRINT_COMMANDS = false;

  // Objects for logging
  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Clock::SharedPtr clock_;

  // State and command interface data
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  double imuHeading_gpio;
  double mode_gpio;
  double boardBConnected_gpio;
  double dummy_gpio_cmd;
  double gpsFix_gpio;
  double gpsLat_gpio;
  double gpsLong_gpio;
  double gpsAlt_gpio;
  double gpsHdop_gpio;
};

}  // namespace imprimis_hardware_platform

#endif  // imprimis_hardware_platform__DIFFBOT_SYSTEM_HPP_
