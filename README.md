# Imprimis
This repository contains necessary software for Imprimis, a fully autonomous ground vehicle developed by the HyperRobotics VIP at VCU. The ROS packages inside imprimis_ws run on the robot's main PC to process sensor information, make autonomous decisions, and communicate with the robot's onboard microcontrollers. The microcontroller firmwares in EspFirmware run on the robot's microcontrollers which handle hardware connectivity and control.

# Microcontroller Firmwares in EspFirmware
* Board A: ESP32 connected to the Main PC, status lights, radio receiver, and wirelessly to board B
* Board B: ESP32 connected to the robot's motors and wirelessly to board A
* GPS: Arduino connected to the Main PC and a GPS module

# ROS Packages in imprimis_ws/src
* imprimis_hardware_platform: Contains all configuration files for ros2 control and implements the hardware interface.
* imprimis_description: Describes the robot with URDF files for other packages.
* imprimis_navigation: Contains all configuration files for localization and nav2.
* gps_nav_bridge: Handles input navigation waypoints and reads GPS data from the Arduino.
* sim_gps_from_odom: Provides fake GPS data derived from local state estimation.
* um7: Driver for the um7 IMU.
* serial-ros2: Serial library for microcontroller communications.

# Install/run instructions for Offboard Computer
This software usually runs on the robot's main computer, but it is possible to run in a simulated mode on a different machine with the following steps:

  1. Ensure you are running Ubuntu 22.04 and [install ROS2 Humble.](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
  2. Source ROS: ```source /opt/ros/humble/setup.bash``` 
  3. Clone this repository: ```git clone https://github.com/Hyperloop-VCU/imprimis.git```
  4. Navigate to workspace root: ```cd imprimis/imprimis_ws```
  5. Install ROS dependencies: ```rosdep install --from-paths src --ignore-src -r -y```
  6. Build workspace: ```colcon build```
  7. Source workspace: ```source install/setup.bash```
  8. Launch fake hardware: ```ros2 launch imprimis_hardware_platform hardware_type:=fake```

You should now see the robot's URDF model appear in rviz, and you can drive it by opening a new terminal and running the following:

  1. ```source /opt/ros/humble/setup.bash```
  2. ```ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap /cmd_vel:=/diffbot_base_controller/cmd_vel -p stamped:=true```



