# Overview
This repository contains necessary software for Imprimis, a fully autonomous ground vehicle developed by the HyperRobotics VIP at VCU. The ROS packages inside imprimis_ws run on the robot's main PC to process sensor information, make autonomous decisions, and communicate with the robot's onboard microcontrollers. Firmware for our two ESP32 microcontrollers in EspFirmware handles hardware connectivity and control.
[Diagrams](https://drive.google.com/file/d/14b34cyZjVn4FuwoEjJM1487I_e60O_WF/view)

# Using the real robot
The real robot's laptop has everything already set up; you do not need to install anything on it. See the [guide](https://docs.google.com/document/d/1qO5M7a82_PlFDrOgXVI_bzZw0K6WJ0vdav_g9hEhckY/edit?tab=t.oms1ptj8oagv) for turn on + turn off procedures, manual + autonomous operation, and battery charging. If you have never used the real robot before and want to learn, contact one of the IMPRIMIS team leads or other experienced VIP members.

# Microcontroller Code in EspFirmware
## Board A
Board A is the interface between the robot's main PC and non USB or ethernet enabled hardware. It's wirelessly connected to Board B via ESP-NOW, USB-connected to the robot's main PC, UART-connected to the FlySky I-BUS Radio Receiver, and GPIO-connected to the yellow and green status lights.

* **Motor Control**: Board A constantly sends desired motor setpoints/efforts and other commands to Board B. It also receives the current wheel velocities from Board B and forwards them to the main PC via USB.

* **Operating Modes**: A switch on the FlySky RC controller determines manual/autonomous mode. In autonomous mode, the board will ignore the joystick input and forward the main PC's angular velocity commands to Board B. In manual mode, the board will ignore the main PC's commands and forward the joystick input to Board B. The current angular velocities, operating mode, and Board B / motor status (connected or not) are always sent to the main PC regardless of operating mode. In other words, the only way the main PC can tell what mode we're in is via the sent status; the communication interface does not change between modes.

* **Status Lights**: When Board B is on and connected, the green light is on. The yellow light flashes on/off continuously when in autonomous mode, and is always on when in manual mode. The red light is directly connected to power and not controllable by Board A.

* **Safety logic**: If the robot is in autonomous mode and no commands are being sent to Board A over the USB connection, it will stop sending commands to Board B, which will cause the robot to stop after a short delay.

## Board B
Board B manages the motors exclusively. It is powered by the motor driver's 5V output, so it is only on when the motors are on. It's wirelessly connected to Board A via ESP-NOW, UART-connected to the Cytron MDDS60 Motor driver, and GPIO-connected to both wheel encoders through a 3.3V <-> 5V level shifter.

* **Control modes**: In autonomous mode, it wirelessly receives desired angular velocity setpoint commands for each wheel, and runs a closed-loop PID controller for each motor. In manual mode, it wirelessly receives joint efforts (0-100% power) for each motor, and simply sets the speeds directly without any PID control.

* **Encoders**: This board uses the ESP32's dedicated pulse counter hardware to read the encoder pulses from each wheel. These pulses are converted to the wheel angular velocities and sent to Board A. In closed-loop mode, the wheel angvels are also used to provide feedback to the PID controllers. A bidirectional 3.3V <-> 5V logic level shifter is used between the ESP32 (3.3V only) and encoders (5V only).

* **Safety logic**: Board B expects constant commands from Board A. If it doesn't receive any for a short (configurable) time, it immediately stops the robot.

Both the **boardA** and **boardB** folders are **PlatformIO projects.** The PlatformIO VSCode extension is a nice way to work on MCU firmware without the Arduino IDE. You can clone the repo, download VSCode, get the PlatformIO extension, and open either board A or board B as a PlatformIO project in VSCode to edit the code. The **common** folder contains packet definitions and configurations shared between the two boards.

# ROS Packages in imprimis_ws/src
This folder contains some of the ROS packages required for Imprimis. Some are custom-made, and some are third-party. In addition to these packages, Imprimis depends on lots of other third-party packages (robot_localization, nav2, etc). The dependencies of each package are listed in each package's **package.xml** file, and can be automatically installed by running the ```rosdep``` command in the simulation setup instructions.

* **imprimis_hardware_platform**: Contains all configuration files for hardware and sensors, and implements the hardware interface. Has launch files that start up all the hardware, either real or simulated.
* **imprimis_description**: Describes the robot with URDF files and configures the simulated robot's sensors. Has launch files to view the URDFs in rviz.
* **imprimis_navigation**: Contains all configuration and launch files for localization and nav2. Has a few nodes that assist with the navigation process, but is mainly filled with config files for nav2 and localization.
* **imprimis_perception**: Contains all nodes that take raw sensor data and turn it into PointCloud data which is then fed to the navigation system. Also has a launch file to start up all perception nodes.
* **um7**: Third-party driver for the um7 IMU.
* **serial-ros2**: Third-party serial library for microcontroller communications.
* **utils**: Package of miscellaneous custom nodes that perform helpful functions in the software stack.
* **SLAM_Packages**: A folder containing a few third-party packages used for global localization with the lidar. These packages are mainly used for testing indoors when there is no GPS signal; it will not be used in the competition, as we will use the GPS for global localization there.

# Install Instructions for Simulation on your own computer
If you want to run a full simulation of the robot's software on your own computer, see these following steps.

  1. Ensure you are running Ubuntu 24.04 and [install ROS2 Jazzy.](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html). Refer to the [ROS learning guide](https://docs.google.com/document/d/1P0E2MpprfE0GaCN3jhLeON7xKa2E6FdJSoLXZ9IfETE/edit?tab=t.0#heading=h.1g0v75y8rcuq) if you are unsure how to do this. It is also HIGHLY recommended to go through this entire guide before installing this simulation - it will make a whole lot more sense if you know a bit about ROS first.
  2. Source ROS: ```source /opt/ros/jazzy/setup.bash``` 
  3. Clone this repository: ```git clone https://github.com/Hyperloop-VCU/imprimis.git```
  4. Navigate to workspace root: ```cd imprimis/imprimis_ws```
  5. Install ROS dependencies: ```rosdep install --from-paths src --ignore-src -r -y```
  6. Add the following line to the file ~/.bashrc: ```export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp```
  7. Build workspace: ```colcon build```
  8. Source workspace: ```source install/setup.bash```
  9. Change permissions for gazebo fixer script: ```chmod 777 ./fix_gazebo.bash```
  10. Run the gazebo fixer script: ```./fix_gazebo.bash```

# Using the Simulated Robot
## Hardware
To launch the simulated robot's hardware, run the following command: 
* ```ros2 launch imprimis_hardware_platform imprimis_sim.launch.py ui_type:=rviz```  

This will make the gazebo simulation and rviz appear. If you say ui_type:=foxglove instead of rviz, you can start up foxglove studio and visualize data there, instead of rviz. In addition to simulating the igvc course, gazebo will also simulate the entire suite of sensors on our robot - gps, lidar, imu, wheel encoders, and depth cameras. Data from these simulated sensors is published to various ros topics; you can see them in rviz or type "ros2 topic list" in the terminal to get the full list of them.

To stop the software, use "ctrl-C" on the keyboard while clicked into the terminal. This applies to every other command here as well - "ctrl-C" in the terminal is the universal exit button.

## Localization system
To launch the simulated robot hardware and localization system, run the following command:
* ```ros2 launch imprimis_navigation localization.launch.py hardware_type:=simulated ui_type:=rviz```

The localization system is responsible for accurately determining where the robot is located relative to its surroundings. It has two main parts, **global** and **local** localization.
* **Global localization** is responsible for providing a **globally consistent** position estimate of the robot. No matter how long the robot drives for, this position estimate will not drift over time and stay correct. However, it does tend to be jittery and jumpy, constantly dancing around the true position of the robot, which is why we have local localization as well. Global localization provides the map->odom and map->base_link transforms. It can be done either with a LiDAR (default, best in indoor environments) or by a GPS (add "map_type:=gps" to the launch command, only works outdoors)
* **Local localization**, or "odometry", is responsible for providing a **locally accurate** position estimate of the robot. This position estimate is smooth and will not jump or jitter. However, it does drift over time as integration errors accumulate, which is why we have global localization as well. Local localization provides the odom->base_link transform. It is usually done by by fusing wheel odometry with an accurate IMU, but this IMU-fusion can be discarded by adding "disable_local_ekf:=true" to the launch command. This will provide a very inaccurate position estimate, but is useful for testing.

## Perception system
To launch the perception system, run the following command:
* ```ros2 launch imprimis_perception course_perception.launch.py```

The perception launch expects the localization system and relevant sensor drivers to already be running, and won't output anything on its own or open up rviz. It only publishes to a few key ROS topics, that the navigation system will read from.
The perception system is responsible for turning the raw data from the lidar and cameras into "PointCloud2" messages which tell navigation exactly where obstacles are located relative to the robot. It has three main parts:
* **Obstacle detection**, implemented in ramp_detector.py, takes the lidar data, cleans it up a bit, and hands it to navigation. This node doesn't need to do too much work, as the lidar tells us exactly where physical obstacles are located in space. It only does a few mathematical corrections to account for robot tilt and other data inconsistencies, and outputs a clean PointCloud2 message on "velodyne_points_nav" representing physical obstacles in the world.
* **Lane detection**, implemented in lane_mapper.py, implements the full pipeline which takes raw image/depth data from the cameras and outputs a clean PointCloud2 message on "perception/lane_points" that outline where the lanes are located in the world. These lane points are treated the exact same as physical obstacles by the navigation system.
* **Pothole detection**, also implemented in lane_mapper.py, implements a similar pipeline to lane detection and outputs a PointCloud2 message on "perception/pothole_points". The difference is that it detects circular white "potholes" on the ground instead of lane lines.

## Navigation system

You can launch the navigation system, localization system, and hardware with the following command:
* ```ros2 launch imprimis_navigation basic_nav.launch.py hardware_type:=simulated```

**The navigation system performs the following functions:**
* **Managing goals**: Given navigation goals published on the "goal_pose" topic, convert them into the appropriate coordinate frame and feed them to the core navigation algorithm (planner and controller servers).
* **Creating the costmap**: Create a 2-D map showing where obstacles are using the pointcloud data from perception, and place the robot on that map using the robot positioning from localization.
* **Computing the path**: Given the costmap, compute the path the robot must take from its current position/rotation (pose) to its goal pose. This is done by the nav2 planner server.
* **Following the path**: Given the costmap, robot position estimates, and path, compute the command velocities needed to follow the path and reach the goal without hitting obstacles in the costmap. This is done by the nav2 controller server.

To make the robot navigate in a straight line to a goal position, you can use the "2D goal pose" tool in rviz to place a goal coordinate relative to the robot's own coordinate frame.

![navigation in rviz w/o perception](.images/navigation_no_perception_demo.mp4)

Notice that the robot doesn't know where any obstacles are - that's because we haven't run the perception system alongside it! We have another launch file to do exactly that, detailed in the next section.

## Navigation + perception system
To launch the robot hardware, localization system, and perception/navigation system, run the following command:
* ```ros2 launch imprimis_navigation sim_mission.launch.py```


This launch file starts both navigation and perception, as well as a custom GUI used for interacting with the simulated robot. Using the "Send IGVC Waypoints" button on the GUI, you can give the robot a set of goal coordinates similar to IGVC, and have it autonomously navigate to those points while avoiding obstacles in the simulated world. The robot can be driven manually using WASD, and reset to its starting position using the "reset" button.

You can also give it goals manually using rviz, exactly as you did before.

![navigation in rviz w/ perception](.images/navigation_perception_demo.mp4)
![giving preset goals using the GUI](.images/gui_demo.mp4)

# Launch Files
As you have seen in previous sections, we use ROS2 python launch files to handle robot startup. It is a hierarchical process:
* Running the navigation launch file will start up navigation-specific nodes AND run the localization launch file.
* Running the localization launch file will start up localization-specific nodes AND run the real hardware launch file, or simulated hardware launch file.

Both hardware and simulated launch files are the "lowest layer" of launching. These launch files both allow the disabling of LiDAR, IMU, GPS, and/or Cameras. You can disable any sensor with a "use_{sensor}=false". For example, to start up all simulated hardware except the LiDAR and GPS:
* ```ros2 launch imprimis_hardware_platform imprimis_sim.launch.py use_lidar:=false use_gps:=false```

Additionally, you can add "ui_type:=foxglove" to any launch file to use foxglove instead of rviz for visualization. Every launch file will use rviz by default. View the source code for navigation, localization, real hardware, and simulated hardware launch files for more details on launch arguments.

# Aliases
There is a hidden file in imprimis_ws called ".bash_aliases" that defines shortcuts to all these long commands. You can add the following to your ~/.bashrc file to define all these aliases automatically:

```
if [ -f ~/Desktop/imprimis/imprimis_ws/.bash_aliases ]; then
   . ~/Desktop/imprimis/imprimis_ws/.bash_aliases
fi
```

Note the ~/Desktop/ path. If you downloaded this somewhere other than your desktop, you'll need to modify those paths. The aliases and shortcuts themselves are all defined in the .bash_aliases file; view the file to learn what they are.
