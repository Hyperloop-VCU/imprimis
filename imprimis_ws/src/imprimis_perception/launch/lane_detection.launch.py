from launch import LaunchDescription
from launch.substitutions import PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, SetEnvironmentVariable, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
import yaml

def generate_launch_description():

    # Declare and initialize arguments
    declared_arguments = []
    """
    declared_arguments.append(
        DeclareLaunchArgument(
            "name",
            default_value="true",
            choices=("true", "false"),
            description="description",
        )
    )
    
    name = LaunchConfiguration("name")
    """

    # Camera driver
    # The GroupAction with forwarding=False and scoped=True prevents the camera launch file from seeing this launch file's arguments.
    # We don't want to tell the camera "publish_odom_tf=false".
    cam_node = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])]),
                launch_arguments={
                    #'pointcloud.enable': 'false',
                    #'diagnostics_period': '1.0',
                    #'log_level': 'error',
                    #'camera_namespace': 'cameras',
                    #'camera_name': 'd455',
                    #'serial_no': '_234322304110',
                    #'device_type': 'd455'
                }.items(),
            )
        ],
        scoped=True,
        forwarding=False,
    )

    lanedetection_node = Node(
        package="imprimis_perception",
        executable="lane_detection",
    )

    # ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 --frame-id front_link --child-frame-id camera_link
    stp_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--frame-id", "front_link", "--child-frame-id", "camera_link", "--x", "0", "--y", "0", "--z", "0", "--roll", "0", "--pitch", "0", "--yaw", "0"],
    )

    return LaunchDescription(declared_arguments + [
        cam_node,
        lanedetection_node,
        stp_node
    ])