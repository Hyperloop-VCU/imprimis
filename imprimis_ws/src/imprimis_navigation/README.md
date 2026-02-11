# imprimis_navigation
This package contains configs for local/global ekf nodes, navsat_transform_node, gps_nav_bridge, and sim_gps_from_odom. It also has launch files for localization and navigation.

## config
This folder has all the aformentioned config files. 

## localization.launch.py
This starts up all necessary nodes to achieve fused state estimation of the robot. Launch arguments are below:
  * hardware_type: real or fake (simulated will be added soon). Passed down to imprimis_hardware.launch.py. Defaults to real. 
  * use_controller: Passed down to imprimis_hardware.launch.py. Defaults to false.
  * use_gpio: If true, gps_nav_bridge will read GPS data from the GPIO controller state interfaces. If false, the GPS data source is the Arduino over USB. Defaults to false.
  * use_fake_gps: If true, gps_nav_bridge will not publish /gps/fix or read any GPS data on its own. Instead, /gps/fix will be provided by sim_gps_from_odom. Defaults to false.

## basic_nav.launch.py
This starts up all necessary nodes for basic waypoint navigation. TODO
