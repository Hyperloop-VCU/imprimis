from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Ramp detector and the lane pipeline (single_cam: lane_segmentation and lane_projection).

    imu_yaw_in_base is how far the IMU's axes are turned inside base_link. The simulated IMU is
    mounted half a circle around (3.14159). Measure it on the real robot before using this there.
    """
    declared_arguments = [
        DeclareLaunchArgument("imu_yaw_in_base", default_value="3.14159"),
        DeclareLaunchArgument("detect_potholes", default_value="true",
                              description="Look for potholes (painted white circles) and mark them in the cost maps."),
        DeclareLaunchArgument("do_lanedetection", default_value="false",
                              description="If true, starts the lane pipeline (single_cam); if false, doesn't."),
    ]
    # No use_sim_time here on purpose: the ramp detector reads the time from the sensor message stamps.
    ramp_detector = Node(
        package="imprimis_perception",
        executable="ramp_detector",
        parameters=[{"imu_yaw_in_base": ParameterValue(LaunchConfiguration("imu_yaw_in_base"), value_type=float)}],
    )
    lanes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("imprimis_perception"), "launch", "single_cam.launch.py"])),
        launch_arguments={
            "use_hardware": "false",
            "use_nav": "false",
            "standalone_costmap": "false",
            "detect_potholes": LaunchConfiguration("detect_potholes"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("do_lanedetection")),
    )
    return LaunchDescription(declared_arguments + [ramp_detector, lanes])
