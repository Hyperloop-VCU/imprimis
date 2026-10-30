"""Start one camera's segmentation node on the Pi attached to that camera."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    camera_name = LaunchConfiguration('camera_name')
    image_topic = LaunchConfiguration('image_topic')
    mask_topic = LaunchConfiguration('mask_topic')
    raw_mask_topic = LaunchConfiguration('raw_mask_topic')
    process_rate = LaunchConfiguration('process_rate_hz')
    publish_raw = LaunchConfiguration('publish_raw_mask')
    processing_width = LaunchConfiguration('processing_width')

    segmenter = Node(
        package='imprimis_perception',
        executable='lane_segmentation',
        name=['lane_segmentation_', camera_name],
        output='screen',
        parameters=[{
            'image_topic': image_topic,
            'mask_topic': mask_topic,
            'raw_mask_topic': raw_mask_topic,
            'process_rate_hz': ParameterValue(process_rate, value_type=float),
            'publish_raw_mask': ParameterValue(publish_raw, value_type=bool),
            'processing_width': ParameterValue(processing_width, value_type=int),
        }],

        realsense = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
            PathJoinSubstitution([
            FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'
        ])
    ),
    launch_arguments={
        'camera_name': 'camera',
        'camera_namespace': 'camera',
    }.items(),
)
    )
    return LaunchDescription([
        DeclareLaunchArgument('camera_name', default_value='front'),
        DeclareLaunchArgument('image_topic', default_value='/camera/camera/color/image_raw'),
        DeclareLaunchArgument('mask_topic', default_value='/perception/front/lane_mask'),
        DeclareLaunchArgument('raw_mask_topic', default_value='/perception/front/lane_mask_raw'),
        DeclareLaunchArgument('process_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('publish_raw_mask', default_value='false'),
        DeclareLaunchArgument('processing_width', default_value='640',
                              description='Maximum mask width sent over the network; zero keeps native resolution.'),
        segmenter,
    ])
