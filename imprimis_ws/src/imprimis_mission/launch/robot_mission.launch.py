"""The lap software on the PHYSICAL robot.

    UNTESTED ON HARDWARE. This file was written at a desk, from the team's hardware launch file and from the
    simulator launch file beside it (sim_mission.launch.py). It has been started only on a computer with no
    robot attached, where the hardware drivers find nothing and the arming check refuses to drive. Treat the
    first run on the robot as a test of this file: wheels off the ground, motors off, then motors on with a
    hand on the red button.

What it starts
    navigation   the team's basic_nav.launch.py with hardware_type:=real (ros2_control, Board A, LiDAR, IMU, GPS,
                 localization, Nav2 with reverse and recovery)
    camera       ONE RealSense, named "front" like the camera of the robot model, so that its frames match the URDF
    perception   ramp_detector and lane_mapper, set for a real camera
    mission      lap_manager (arming check and GPS boundary ON), speed_governor (walking speed by default),
                 and, if asked for, the control window

What it leaves out, compared with the simulator
    Gazebo, the bridges, the view camera, the course watcher (Blender), and the "true pose" topic. Laps are
    therefore scored with odometry, which is an estimate, and the mouse-look of the control window does nothing.

Before the first run, in this order
    1. Hand controller ON before the motors. Its switch decides: in MANUAL the robot ignores this software
       completely. Only with the switch in AUTONOMOUS do the commands of Nav2 or of the control window reach
       the motors. The red button and the keychain OFF stop the motors whatever this software does.
    2. Device names. Board A must be /dev/ttyUSB0, the IMU /dev/ttyUSB1 and the GPS /dev/ttyACM0 (the team's
       hardware launch file). Plugging in another order swaps the first two.
    3. imu_yaw_in_base. How the IMU is turned inside the robot, in radians. The simulator uses 3.14159. The
       real value is NOT KNOWN; 0.0 here is a placeholder. A wrong value levels the LiDAR points the wrong way.
    4. Wheel numbers. config/diffbot_controllers.yaml has wheel_radius 0.165 and wheel_separation 1.016; the
       robot model has 0.180 and 0.870. Measure, and correct the file that is wrong, or the odometry turns by
       a different angle than the robot.
    5. A course file for the place you are at (course_file). Without a GPS datum in it the arming check refuses
       every automatic run, which is the safe answer. courses/real_course_template.json shows the fields.
    6. Camera. camera_serial picks the unit (the D435i by default). color_fov is the horizontal field of view
       of its COLOR picture in radians; 1.204 is the published figure for a D435i and should be replaced by
       the value in the camera_info topic. The lane mapper has only been run on simulated pictures: its white
       threshold (min_value, max_saturation) will need tuning in daylight.
    7. top_speed stays at 0.5 m/s until laps at that speed are boring.

Typical use
    on the robot:   ros2 launch imprimis_mission robot_mission.launch.py course_file:=/path/to/site.json
    on a laptop on the same network, for the control window:
                    ros2 run imprimis_mission control_gui --ros-args -p course_file:=/path/to/site.json
    on a computer with no robot attached, to see that everything starts (the drivers will report that their
    devices are missing, and an automatic run will be refused):
                    ros2 launch imprimis_mission robot_mission.launch.py use_camera:=false
"""
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
        DeclareLaunchArgument("memory_dir", default_value="~/imprimis_lap_memory_robot",
                              description="Lap records of the REAL robot. Keep it apart from the simulator's lap memory: a simulated route is not valid on real ground."),
        DeclareLaunchArgument("nav2_params", default_value="Course2027"),
        DeclareLaunchArgument("ui_type", default_value="rviz", choices=("foxglove", "rviz", "none")),
        DeclareLaunchArgument("gui", default_value="false", choices=("true", "false"),
                              description="Open the control window on the robot's own screen. Usually it runs on a laptop instead."),
        DeclareLaunchArgument("top_speed", default_value="0.5",
                              description="Speed on clear stretches, m/s. 0.5 is walking pace. The wheel controller caps everything at 2.0."),
        DeclareLaunchArgument("imu_yaw_in_base", default_value="0.0",
                              description="How the IMU is turned inside the robot, radians. NOT MEASURED YET."),
        DeclareLaunchArgument("use_camera", default_value="true", choices=("true", "false")),
        DeclareLaunchArgument("camera_serial", default_value="_923322073287",
                              description="Serial number of the RealSense to use, with a leading underscore. The default is the D435i."),
        DeclareLaunchArgument("color_fov", default_value="1.204",
                              description="Horizontal field of view of the color picture, radians (69 degrees for a D435i; check camera_info)."),
        DeclareLaunchArgument("arming_check", default_value="true", choices=("true", "false"),
                              description="Refuse an automatic run unless GPS, lane lines and sensors say the robot is on the start straight of this course."),
        DeclareLaunchArgument("boundary_check", default_value="true", choices=("true", "false"),
                              description="Stop an automatic run when GPS puts the robot outside the course boundary."),
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

    lap_manager = Node(
        package="imprimis_mission",
        executable="lap_manager",
        output="screen",
        parameters=[{
            "course_file": course_file,
            "memory_dir": LaunchConfiguration("memory_dir"),
            "autostart": False,                   # on the robot a run starts only when a person asks for it
            "exit_when_done": False,
            "arming_check": ParameterValue(LaunchConfiguration("arming_check"), value_type=bool),
            "arming_require_gps": True,
            "boundary_check": ParameterValue(LaunchConfiguration("boundary_check"), value_type=bool),
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
                                                   #lap_manager, speed_governor, control_gui])
