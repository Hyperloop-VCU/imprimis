import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """

    """
    declared_arguments = [
        DeclareLaunchArgument("world"),
        DeclareLaunchArgument("show_sim", default_value="false"), # you may change this to true if seeing the simulation will help you.
        DeclareLaunchArgument("ui_type", default_value="rviz", choices=("foxglove", "rviz", "none")), # you may change this to rviz if rviz will help you.
        DeclareLaunchArgument("top_speed", default_value="0.5", description="Speed on clear stretches, m/s. 2.1 is 4.7 mph; the IGVC limit of 5 mph is 2.235."),
    ]

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare("imprimis_navigation"), "launch", "basic_nav.launch.py"])]),
        launch_arguments={
            "hardware_type": "simulated",
            "world": LaunchConfiguration("world"),
            "nav2_params": "Course2027",
            "show_sim": "false", # if you really want to see the simulation, feel free to change this to "true"
            "ui_type": "none", # if you really want to see rviz, feel free to change this to "rviz"
            "use_recoveries": "true",
            "nav_mode": "outdoor"
        }.items(),
    )

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare("imprimis_perception"), "launch", "course_perception.launch.py"])]),
        launch_arguments={"imu_yaw_in_base": "3.14159", "do_lanedetection": "false"}.items(),
    )

    # Course files are read from the source folder, like the other configuration in this repository.
    # These nodes are deliberately NOT given use_sim_time. They take the time from the stamps on the
    # sensor messages, which spares each of them several hundred clock messages a second.
    def mission_nodes(context):
        return [
            Node(
                package="imprimis_navigation",
                executable="speed_governor",
                output="screen",
                parameters=[{"top_speed": ParameterValue(LaunchConfiguration("top_speed"), value_type=float)}],
            )
        ]

    return LaunchDescription(declared_arguments + [navigation, perception, OpaqueFunction(function=mission_nodes)])
