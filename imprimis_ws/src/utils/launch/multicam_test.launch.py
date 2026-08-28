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
            "publish_odom_tf",
            default_value="true",
            choices=("true", "false"),
            description="If false, prevents the diff drive controller broadcasting the odom->base_link transform from wheel odometry. This MUST be false if there is another node broadcasting odom -> base_link (e.g. the local EKF).",
        )
    )
    
    use_imu = LaunchConfiguration("use_imu")
    """

    # Camera driver
    # The GroupAction with forwarding=False and scoped=True prevents the camera launch file from seeing this launch file's arguments.
    # We don't want to tell the camera "publish_odom_tf=false".
    d455 = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])]),
                launch_arguments={
                    'pointcloud.enable': 'true',
                    'diagnostics_period': '1.0',
                    'log_level': 'error',
                    'camera_namespace': 'cameras',
                    'camera_name': 'd455',
                    'serial_no': '_234322304110',
                    'device_type': 'd455'
                }.items(),
            )
        ],
        scoped=True,
        forwarding=False,
    )

    d435i = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])]),
                launch_arguments={
                    'pointcloud.enable': 'true',
                    'diagnostics_period': '1.0',
                    'log_level': 'error',
                    'camera_namespace': 'cameras',
                    'camera_name': 'd435i',
                    'serial_no': '_923322073287',
                    'device_type': 'd435i'
                }.items(),
            )
        ],
        scoped=True,
        forwarding=False,
    )

    
    d435_1 = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])]),
                launch_arguments={
                    'pointcloud.enable': 'true',
                    'diagnostics_period': '1.0',
                    'log_level': 'error',
                    'camera_namespace': 'cameras',
                    'camera_name': 'd435_1',
                    'serial_no': '_819112070324',
                    'device_type': 'd435'
                }.items(),
            )
        ],
        scoped=True,
        forwarding=False,
    )

    d435_2 = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])]),
                launch_arguments={
                    'pointcloud.enable': 'true',
                    'diagnostics_period': '1.0',
                    'log_level': 'error',
                    'camera_namespace': 'cameras',
                    'camera_name': 'd435_2',
                    'serial_no': '_827312072151',
                    'device_type': 'd435'
                }.items(),
            )
        ],
        scoped=True,
        forwarding=False,
    )


    # foxglove bridge
    foxglove_launch_include = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml'])]),
    )

    return LaunchDescription(declared_arguments + [
        d455,
        d435i,
        d435_1,
        d435_2,
        foxglove_launch_include
    ])