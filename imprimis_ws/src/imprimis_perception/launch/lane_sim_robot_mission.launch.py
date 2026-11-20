"""Sim robot mission with the lane pipeline (lane_segmentation -> lane_projection) and Lane2027."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

CAMERA = 'front'
OPTICAL_FRAME = 'd435i_color_optical_frame'
RAW_MASK = '/perception/front_link_frame/lane_mask'
MASK = f'/perception/{CAMERA}/lane_mask'

# Gazebo stamps the camera with front_link (a body frame); lane_projection needs an optical frame.
# Remove this relay once gz_frame_id in gazebo_info.xacro is set to the optical frame.
MASK_RELAY = f'''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
rclpy.init()
node = Node('sim_lane_mask_relay')
pub = node.create_publisher(CompressedImage, '{MASK}/compressed', 5)
def relay(msg):
    msg.header.frame_id = '{OPTICAL_FRAME}'
    pub.publish(msg)
node.create_subscription(CompressedImage, '{RAW_MASK}/compressed', relay, 5)
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
'''


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("nav2_params", default_value="Lane2027"),
        DeclareLaunchArgument("ui_type", default_value="rviz", choices=("foxglove", "rviz", "none")),
        DeclareLaunchArgument("gui", default_value="true", choices=("true", "false")),
        DeclareLaunchArgument("top_speed", default_value="0.5"),
        DeclareLaunchArgument("imu_yaw_in_base", default_value="3.14159"),
        DeclareLaunchArgument("world", default_value="igvc2027"),
        DeclareLaunchArgument("show_sim", default_value="true"),
    ]

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
            [FindPackageShare("imprimis_navigation"), "launch", "basic_nav.launch.py"])]),
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

    ramp_lanes = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-world", "default", "-name", "ramp_lanes", "-x", "-6.386", "-y", "0", "-z", "0",
                   "-file", PathJoinSubstitution(
                       [FindPackageShare("imprimis_perception"), "sim", "ramp_lanes", "model.sdf"])],
        output="screen",
        condition=LaunchConfigurationEquals("world", "igvc2027"),
    )

    ramp_detector = Node(
        package="imprimis_perception",
        executable="ramp_detector",
        output="screen",
        parameters=[{"imu_yaw_in_base": ParameterValue(LaunchConfiguration("imu_yaw_in_base"), value_type=float)}],
    )

    camera_optical_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="sim_camera_optical_tf",
        arguments=["--roll", "-1.5707963", "--yaw", "-1.5707963",
                   "--frame-id", "front_link", "--child-frame-id", OPTICAL_FRAME],
    )

    lane_segmentation = Node(
        package="imprimis_perception",
        executable="lane_segmentation",
        name="lane_segmentation",
        output="screen",
        parameters=[{
            "image_topic": f"/cameras/{CAMERA}/color/image_raw",
            "mask_topic": RAW_MASK,
            "raw_mask_topic": f"/perception/{CAMERA}/lane_mask_raw",
        }],
    )
    mask_relay = ExecuteProcess(
        cmd=[FindExecutable(name="python3"), "-c", MASK_RELAY],
        name="sim_lane_mask_relay",
        output="screen",
    )
    lane_projection = Node(
        package="imprimis_perception",
        executable="lane_projection",
        name="lane_projection",
        output="screen",
        parameters=[{
            "camera_names": [CAMERA],
            "detect_potholes": True,
        }],
    )

    speed_governor = Node(
        package="imprimis_navigation",
        executable="speed_governor",
        output="screen",
        parameters=[{"top_speed": ParameterValue(LaunchConfiguration("top_speed"), value_type=float)}],
    )
    sim_control_gui = Node(
        package="imprimis_navigation",
        executable="sim_control_gui",
        output="screen",
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    return LaunchDescription(declared_arguments + [
        navigation, ramp_lanes, camera_optical_tf, ramp_detector, lane_segmentation, mask_relay, lane_projection,
        speed_governor, sim_control_gui,
    ])
