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

#include "imprimis_hardware_platform/diffbot_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>
#include <iostream>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "imprimis_hardware_platform/serial_comms.hpp"

namespace imprimis_hardware_platform {




// Initialization: runs once on startup.
// The command and state interfaces are defined in the ros2 control XACRO.
hardware_interface::CallbackReturn DiffBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{

  // initialize child class - SystemInterface
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // setup logger and clock
  logger_ = std::make_shared<rclcpp::Logger>(
    rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system.DiffBot"));
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  //hw_start_sec_ =
    //hardware_interface::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  //hw_stop_sec_ =
    //hardware_interface::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // Allocates memory for the state/command buffers and initializes each to NAN.
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());


  // Ensures that the state/command interface definitionsline up with what it expects.
  // These state/command interfaces are defined in the ros2 control XACRO.
  // This is probably removable, but don't want to take any chances
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as second state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}





// Runs once on startup. Defines all the state interfaces (wheel velocities, estop, etc) for ros2 control.
std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}






// Runs once on startup. Defines all the command interfaces (movable wheels) for ros2 control.
std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}






// Runs once when the hardware interface is activated.
// Tries to initialize serial comms with board A on three different ports.
// Fatal error if it can't.
hardware_interface::CallbackReturn DiffBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // setup serial connection
  esp32 = std::make_shared<SerialLink>(115200, 0.01);

  // try ports
  const char* ports[] =
  {
    "/dev/ttyUSB0",
    "/dev/ttyUSB1",
    "/dev/ttyUSB2"
  };

  bool serial_init_success = false;
  for (size_t i = 0; i < std::size(ports); i++)
  {
    if (esp32->initialize_link(ports[i]) != SerialLink::Status::Ok)
    {
      RCLCPP_INFO(get_logger(), "Tried and failed to initialize comms with board A on port: %s", ports[i]);
    }
    else
    {
      RCLCPP_INFO(get_logger(), "\n\n[INFO] Successfully initialized comms with board A on port: %s\n", ports[i]);
      serial_init_success = true;
      break;
    }
  }

  // fatal error if we can't open any of the ports
  if (!serial_init_success)
  {
    RCLCPP_FATAL(get_logger(), "\n\n[FATAL] could not reach board A on any of the configured ports.\n\n");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // default zeros for command and state interfaces
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}





// Runs once when the hardware interface is deactivated.
// SerialLink is automatically closed in the destructor.
hardware_interface::CallbackReturn DiffBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{


  return hardware_interface::CallbackReturn::SUCCESS;
}





// Runs continuously. This is called by ros2 control to read state information from the ESP board.
// Reads data from the ESP to update all the state interfaces.
// This is where a bad/broken connection is detected and handled.
// Since the hardware only gives us the velocity, we just integrate it to get position.
// period: the time elapsed since the last call to read()
// the first parameter is unused, it's just the overall ROS time.
hardware_interface::return_type DiffBotSystemHardware::read(const rclcpp::Time &, const rclcpp::Duration & period)
{


  // read velocity
  float leftAngvel, rightAngvel;
  auto read_status = esp32->read_current_angvels(leftAngvel, rightAngvel);
  if (read_status == SerialLink::Status::Ok)
  {
    // read succeeded
    hw_velocities_[0] = static_cast<float>(leftAngvel);
    hw_velocities_[1] = static_cast<float>(rightAngvel);
    // integrate velocity to get position
    hw_positions_[0] += period.seconds() * hw_velocities_[0];
    hw_positions_[1] += period.seconds() * hw_velocities_[1];
  }
  else
  {
    // read failed
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500, "Could not read hardware states from microcontroller.");
    // velocity kept to what it was previously
  }


  // print the newly-read states if debugging
  if (PRINT_READ_STATES && read_status == SerialLink::Status::Ok)
  {
    std::stringstream ss;
    ss << "Reading states:";

    // left wheel
    ss << std::fixed << std::setprecision(2) << std::endl
        << "\t"
          "position "
        << hw_positions_[0] << " and velocity " << hw_velocities_[0] << " for '"
        << info_.joints[0].name.c_str() << "'!";

    // right wheel
    ss << std::fixed << std::setprecision(2) << std::endl
        << "\t"
          "position "
        << hw_positions_[1] << " and velocity " << hw_velocities_[1] << " for '"
        << info_.joints[1].name.c_str() << "'!";

    
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
  }

  return hardware_interface::return_type::OK;
}





// Runs continuously. This is called by ros2 control to write commands to the connected ESP board.
// hw_commands_[] contains the latest angular velocity requests (rad/s).
// this function looks at those requests and forwards them to the ESP.
// Both parameters are unused, and they are identical to read().
// time: the overall ros time
// period: the time elapsed since the last call to write()
// We don't need to use those because the firmware handles the DT calculation.
hardware_interface::return_type imprimis_hardware_platform::DiffBotSystemHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  // print commands written if debugging
  if (PRINT_COMMANDS)
  {
    std::stringstream ss;
    ss << "Writing commands:";
    ss << std::fixed << std::setprecision(2) << std::endl
        << "\t" << "command " << hw_commands_[0] << " for '" << info_.joints[0].name.c_str() << "'!";
    ss << std::fixed << std::setprecision(2) << std::endl
        << "\t" << "command " << hw_commands_[1] << " for '" << info_.joints[1].name.c_str() << "'!";
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
  }

  // write commands
  if (esp32->write_angvel_commands(hw_commands_[0], hw_commands_[1]) != SerialLink::Status::Ok)
  {
    // Handle failed serial writing. We don't need to do anything here - If board A didn't get the data,
    // it will not send data back, read() will fail, and we will handle a bad/lost connection in there.
  }

  return hardware_interface::return_type::OK;
}




}  // end namespace imprimis_hardware_platform


// export this class for pluginlib
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  imprimis_hardware_platform::DiffBotSystemHardware, hardware_interface::SystemInterface)
