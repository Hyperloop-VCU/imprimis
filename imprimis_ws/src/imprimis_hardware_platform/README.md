# imprimis_hardware_platform
This package has the configuration files for ros2 control and implements Imprimis's custom ros2 control hardware interface. It's based on [example #2](https://github.com/ros-controls/ros2_control_demos/tree/master/example_2) in the ros2 control examples.

## launch
The launch directory contains the ros2 launch file ```imprimis_hardware.launch.py```. This starts up everything necessary for the robot to drive via commands from its main PC, and the Velodyne Lidar. Launch arguments are below:
  * gui: If true, starts up Rviz. Defaults to true.
  * publish_odom_tf: If true, publishes odometry directly from the diff drive controller. This must be false when running ekf nodes, or the unfiltered and filtered odometries will conflict. Defaults to true.
  * use_controller: If true, launches teleop_twist_joy and maps its output to the controller's cmd_vel. Defaults to false.
  * hardware_type: real, fake, or simulated. Defaults to real. See below:

  #### Real hardware
  Launches imprimis with all real hardware. Connect to board A and the real lidar, and uses the real hardware interface in the hardware directory.
  #### Fake hardware
  Launches imprimis without the lidar and uses a mock hardware interface that reports wheel velocities exactly equal to the controller's setpoints.
  #### Simulated hardware
  Launches imprimis in a Gazebo simulation. In this mode, the controller manager is internal to Gazebo and not launched in this file. The simulated lidar publishes to the same topic as the real lidar without needing any driver/parser nodes. This does not work with the GPS or the GPIO controller currently (will be         fixed soon)

  
## config
The config directory has two configuration files for the diff drive controller: one for real hardware, and one for gazebo-simulated hardware. These files define the controller parameters (update rate, acceleration limits, velocity limits, etc) and also list the GPIO state interfaces. In the simulated config, use_sim_time is true.

## hardware
This contains the C++ headers and implementations for the hardware interface and custom serial communication with board A. The diff drive controller calls the hardware interface read() method, then the write() method, then waits and calls the two functions again.
