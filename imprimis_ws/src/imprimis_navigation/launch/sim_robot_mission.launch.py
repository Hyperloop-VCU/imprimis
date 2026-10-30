import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("nav2_params", default_value="Course2027"),
        DeclareLaunchArgument("ui_type", default_value="rviz", choices=("foxglove", "rviz", "none")),
        DeclareLaunchArgument("gui", default_value="true", choices=("true", "false"),
                              description="Open the sim control window (WASD driving, reset, send IGVC waypoints)."),
        DeclareLaunchArgument("top_speed", default_value="0.5",
                              description="Speed on clear stretches, m/s. 0.5 is walking pace. The wheel controller caps everything at 2.0."),
        DeclareLaunchArgument("imu_yaw_in_base", default_value="3.14159",
                              description="How the IMU is turned inside base_link. The simulated IMU is mounted half a circle around."),
        DeclareLaunchArgument("world", default_value="igvc2027",
                              description="World file used for simulation (excluding the .sdf). It must be located in imprimis_hardware_platform/worlds."),
        DeclareLaunchArgument("show_sim", default_value="true",
                              description="If false, runs Gazebo in headless mode (no GUI, just server). If true, runs the GUI like normal."),
    ]

    notice = LogInfo(msg="sim_robot_mission.launch.py: robot_mission.launch.py's mission, in simulation, through the "
                         "lane_segmentation / lane_projection pipeline instead of lane_mapper.")

    # basic_nav.launch.py starts Gazebo and localization underneath it. use_cams stays true here:
    # in sim the camera is bridged by Gazebo itself under the topic names the URDF frames expect
    # (see gz_bridge.yaml), so no separate camera driver needs to be started, unlike on real hardware.
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare("imprimis_navigation"), "launch", "basic_nav.launch.py"])]),
        launch_arguments={
            "hardware_type": "simulated",
            "world": LaunchConfiguration("world"),
            "nav2_params": LaunchConfiguration("nav2_params"),
            "show_sim": LaunchConfiguration("show_sim"),
            "ui_type": LaunchConfiguration("ui_type"),
            "use_cams": "true",
            "use_recoveries": "true",
        }.items(),
    )

    # None of the nodes below is given use_sim_time: they take their time from the stamps of the
    # sensor messages, same as robot_mission.launch.py does on real hardware.
    imu_yaw = ParameterValue(LaunchConfiguration("imu_yaw_in_base"), value_type=float)
    ramp_detector = Node(
        package="imprimis_perception",
        executable="ramp_detector",
        output="screen",
        parameters=[{
            "imu_yaw_in_base": imu_yaw,
        }],
    )
    # The simulated camera stamps its images with "front_link" directly -- there is no RealSense
    # driver in sim to publish camera_color_optical_frame like there is on the real robot, so this
    # supplies it: the standard link-to-optical-frame rotation (roll -90, yaw -90) every ROS camera
    # driver uses, so lane_projection's TF lookups resolve exactly like they do on real hardware.
    camera_optical_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--roll", "-1.5707963", "--pitch", "0", "--yaw", "-1.5707963",
                   "--frame-id", "front_link", "--child-frame-id", "camera_color_optical_frame"],
    )

    # lane_segmentation + lane_projection replace lane_mapper: segmentation finds the white tape in
    # the color image, projection places it on the ground via TF and vetoes points near a LiDAR-seen
    # obstacle.
    lane_segmentation = Node(
        package="imprimis_perception",
        executable="lane_segmentation",
        output="screen",
        parameters=[{
            "image_topic": "cameras/front/color/image_raw",
        }],
    )
    lane_projection = Node(
        package="imprimis_perception",
        executable="lane_projection",
        output="screen",
        parameters=[{
            "camera_info_topic": "cameras/front/color/camera_info",
            "detect_potholes": True,
        }],
    )
    speed_governor = Node(
        package="imprimis_navigation",
        executable="speed_governor",
        output="screen",
        parameters=[{"top_speed": ParameterValue(LaunchConfiguration("top_speed"), value_type=float)}],
    )
    # the simulator's own control window (WASD driving, reset, send IGVC waypoints) -- there is no
    # "control_gui" executable in this workspace, that was a leftover from robot_mission.launch.py
    sim_control_gui = Node(
        package="imprimis_navigation",
        executable="sim_control_gui",
        output="screen",
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    return LaunchDescription(declared_arguments + [
        notice, navigation, camera_optical_tf, ramp_detector, lane_segmentation, lane_projection,
        speed_governor, sim_control_gui
    ])
