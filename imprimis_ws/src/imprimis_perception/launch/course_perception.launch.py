from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Ramp detector and lane mapper, the two perception nodes the cost maps read.

    imu_yaw_in_base is how far the IMU's axes are turned inside base_link. The simulated IMU is
    mounted half a circle around (3.14159). Measure it on the real robot before using this there.
    """
    declared_arguments = [
        DeclareLaunchArgument("imu_yaw_in_base", default_value="3.14159"),
        DeclareLaunchArgument("use_depth", default_value="true",
                              description="Place lane pixels with the depth image. If false, a flat ground plane is assumed."),
        DeclareLaunchArgument("detect_potholes", default_value="true",
                              description="Look for potholes (painted white circles) and mark them in the cost maps."),
        DeclareLaunchArgument("detect_drop_offs", default_value="true",
                              description="Also look for real holes, where the depth camera sees the ground farther away than flat ground would be."),
    ]
    # No use_sim_time here on purpose: both nodes read the time from the sensor message stamps.
    common = {
        "imu_yaw_in_base": ParameterValue(LaunchConfiguration("imu_yaw_in_base"), value_type=float),
    }

    ramp_detector = Node(
        package="imprimis_perception",
        executable="ramp_detector",
        parameters=[common],
    )
    lane_mapper = Node(
        package="imprimis_perception",
        executable="lane_mapper",
        parameters=[common, {"use_depth": ParameterValue(LaunchConfiguration("use_depth"), value_type=bool),
                             "detect_potholes": ParameterValue(LaunchConfiguration("detect_potholes"), value_type=bool),
                             "detect_drop_offs": ParameterValue(LaunchConfiguration("detect_drop_offs"), value_type=bool)}],
    )
    return LaunchDescription(declared_arguments + [ramp_detector, lane_mapper])
