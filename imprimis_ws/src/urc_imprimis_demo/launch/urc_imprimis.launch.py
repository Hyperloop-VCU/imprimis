from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Declare all arguments
    declared_arguments = [
        DeclareLaunchArgument(
            name="hardware_type",
            default_value="real",
            choices="real, fake, simulated",
            description="Real runs all the real robot hardware, fake runs no sensor drivers and a minimal ros2 control hardware interface for the motors, simulated runs a full gazebo sim with simulated motors, sensors, and a full physics world."
        )
    ]

    # URC Driver node (just a bno055 for this demo). This node would run on the middle man in all cases.
    urc_driver_node = Node(
        package = 'bno055',
        executable = 'bno055',
        parameters = [os.path.join(get_package_share_directory("urc_imprimis_demo"), "config", "bno055.yaml")],
    )

    # URC translator node. This node would also run on the middle man in all cases.
    urc_imprimis_translator_node = Node(
        package="urc_imprimis_demo",
        executable="urc_imprimis_translator_node"
    )

    # Robot driver nodes, this would run on the actual robot hardware OR the MM depending on the robot.
    # For the wx200, it would run on the MM, but for Imprimis, it would run on Imprimis's PC.

    # It's the user's responsibility to ensure that the robot's PC is up and running with all required ROS nodes.
    # The MM shouldn't have to "remote-start" the robot's PC, all that stuff would already be running.
    robot_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("imprimis_hardware_platform"), 'launch', 'imprimis_hardware.launch.py'])),
        launch_arguments={
            "hardware_type": LaunchConfiguration("hardware_type"),
            "use_controller": "true"
        }.items()
    )

    
    return LaunchDescription(declared_arguments + [
        urc_driver_node,
        urc_imprimis_translator_node,
        robot_driver_launch
    ])