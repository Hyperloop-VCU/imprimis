# Imprimis
This repository contains necessary software for Imprimis, a fully autonomous ground vehicle developed by the HyperRobotics VIP at VCU. The ROS packages inside imprimis_ws run on the robot's main PC to process sensor information, make autonomous decisions, and communicate with the robot's onboard microcontrollers. The microcontroller firmwares in EspFirmware run on the robot's microcontrollers which handle hardware connectivity and control.

# Microcontroller Firmwares in EspFirmware

## Board A 
This is an ESP32 connected to the main PC. It controls the onboard status lights, switches between manual/autonomous operating modes, and wirelessly forwards motor commands to board B. It provides ROS with access to the wheel velocities and other hardware data over the USB port.

## Board B
This is an ESP32 connected to the motors. It is wirelessly connected to board A and handles the PID control loop with encoder readings while wirelessly sending wheel velocities back to board A.

## GPS
This is an Arduino board connected to the main PC. It is connected to a UART-enabled GPS and acts as a parser/bridge, sending the GPS fix data over USB as JSON.

# ROS packages in imprimis_ws/src
##imprimis_hardware_platform
Configuration of ros2 control and hardware interface implementation. Has the ```launch_hardware.launch.py``` launch file, which starts up all code necessary for the PC to control the motors.
##imprimis_description
