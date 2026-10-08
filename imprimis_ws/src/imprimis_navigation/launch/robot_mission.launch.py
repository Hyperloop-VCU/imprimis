import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("course_file", default_value="",
                              description="Course file of the place the robot is at (finish line, checkpoints, GPS datum). Empty: mode switching only, no laps."),
        DeclareLaunchArgument("nav2_params", default_value="Course2027"),
        DeclareLaunchArgument("ui_type", default_value="rviz", choices=("foxglove", "rviz", "none")),
        DeclareLaunchArgument("gui", default_value="false", choices=("true", "false"),
                              description="Open the control window on the robot's own screen. Usually it runs on a laptop instead."),
        DeclareLaunchArgument("top_speed", default_value="0.5",
                              description="Speed on clear stretches, m/s. 0.5 is walking pace. The wheel controller caps everything at 2.0."),
        DeclareLaunchArgument("imu_yaw_in_base", default_value="0.0",
                              description="How the IMU is turned inside the robot, radians. NOT MEASURED YET."),
        DeclareLaunchArgument("color_fov", default_value="1.204",
                              description="Horizontal field of view of the color picture, radians (69 degrees for a D435i; check camera_info)."),
    ]
    course_file = LaunchConfiguration("course_file")

    notice = LogInfo(msg="robot_mission.launch.py is UNTESTED ON HARDWARE. Hand controller on first; red button within reach; "
                         "the motors obey this software only while the controller's switch is in AUTONOMOUS.")

    # The team's navigation launch file starts the hardware and the localization underneath it.
    # use_cams stays false there: that switch starts all four cameras under other names (d455, d435i, ...),
    # whose frames the robot model does not have. One camera is started below instead.
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare("imprimis_navigation"), "launch", "basic_nav.launch.py"])]),
        launch_arguments={
            "hardware_type": "real",
            "nav2_params": LaunchConfiguration("nav2_params"),
            "ui_type": LaunchConfiguration("ui_type"),
            "nav_mode": "indoor",
            "use_cams": "false",
            "use_recoveries": "true",
        }.items(),
    )

    # One RealSense, in the name space "cameras" with the name "front": the robot model calls its camera
    # "front" (front_link), and the driver then adds the optical frames below that link. Depth is aligned to
    # the color picture, because the lane mapper looks up the depth of a color pixel at the same pixel.
    # The group keeps this launch file's own arguments away from the camera's launch file.
    camera = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"])]),
                launch_arguments={
                    "camera_namespace": "cameras",
                    "camera_name": "front",
                    #"serial_no": LaunchConfiguration("camera_serial"),
                    "align_depth.enable": "true",
                    "enable_sync": "true",
                    "pointcloud.enable": "false",
                    "log_level": "warn",
                }.items(),
            )
        ],
        scoped=True,
        forwarding=False,
        condition=IfCondition(LaunchConfiguration("use_camera")),
    )

    # None of the nodes below is given use_sim_time: on the robot the clock is the computer's clock, and the
    # nodes take their time from the stamps of the sensor messages in any case.
    imu_yaw = ParameterValue(LaunchConfiguration("imu_yaw_in_base"), value_type=float)
    ramp_detector = Node(
        package="imprimis_perception",
        executable="ramp_detector",
        output="screen",
        parameters=[{
            "imu_yaw_in_base": imu_yaw,
            "depth_topic": "cameras/front/depth/image_rect_raw",
            "depth_in_optical_frame": True,       # a real RealSense stamps its pictures with the optical frame
            "camera_offset_x": 0.0,               # the simulated sensor sits 0.05 m ahead of its link; the real one does not
            "horizontal_fov": 1.52,               # of the DEPTH picture: 87 degrees for the D435 family (maker's figure)
        }],
    )
    lane_mapper = Node(
        package="imprimis_perception",
        executable="lane_mapper",
        output="screen",
        parameters=[{
            "imu_yaw_in_base": imu_yaw,
            "image_topic": "cameras/front/color/image_raw",
            "depth_topic": "cameras/front/aligned_depth_to_color/image_raw",
            "image_in_optical_frame": True,
            "camera_offset_x": 0.0,
            "horizontal_fov": ParameterValue(LaunchConfiguration("color_fov"), value_type=float),
            "use_depth": True,
            "detect_potholes": True,
            "detect_drop_offs": False,            # proven only on made-up pictures; real depth has holes of its own
        }],
    )
    speed_governor = Node(
        package="imprimis_mission",
        executable="speed_governor",
        output="screen",
        parameters=[{"top_speed": ParameterValue(LaunchConfiguration("top_speed"), value_type=float)}],
    )
    control_gui = Node(
        package="imprimis_mission",
        executable="control_gui",
        output="screen",
        parameters=[{"course_file": course_file, "speed_boost": 1.0, "speed_normal": 0.5}],
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    return LaunchDescription(declared_arguments + [notice, navigation, camera, ramp_detector, lane_mapper, speed_governor])
