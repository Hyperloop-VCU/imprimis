# imprimis_description
This package has the URDF of the robot. The URDF describes the dimensions of the robot, sensor and wheel positions, physics properties for simulation, and a visual model for visualization. It also defines which joints are controlled by the diff drive controller, and the state interfaces published by the GPIO controller.

## launch
This folder has ```view_robot.launch.py```, which starts up rviz to visualize the URDF in a vacuum.

## meshes
This folder will contain all custom 3D meshes for the visual model of the robot.

## rviz
This folder contains rviz config files for visualization of the URDF.

## urdf
This folder contains the actual URDF description. It's split across multiple files via xacro macros. The main file is ```diffbot.urdf.xacro```, which includes macros defined in the other files.
