"""Single-host four-camera bring-up/test of segmentation and projection.

For cameras on multiple Pis, start lane_segmentation.launch.py once per camera
and lane_projection.launch.py once on the receiving laptop. Start camera drivers
and publish calibrated camera-to-base TF separately in either arrangement.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


CAMERAS = ('d455', 'd435i', 'd435_1', 'd435_2')


def generate_launch_description():
    base_frame = LaunchConfiguration('base_frame')
    process_rate = LaunchConfiguration('process_rate_hz')
    publish_raw = LaunchConfiguration('publish_raw_mask')
    processing_width = LaunchConfiguration('processing_width')
    mask_topics = [f'perception/{name}/lane_mask' for name in CAMERAS]
    camera_info_topics = [f'cameras/{name}/color/camera_info' for name in CAMERAS]

    segmentation_nodes = []
    for name in CAMERAS:
        segmentation_nodes.append(Node(
            package='imprimis_perception',
            executable='lane_segmentation',
            name=f'lane_segmentation_{name}',
            output='screen',
            parameters=[{
                'image_topic': f'cameras/{name}/color/image_raw',
                'mask_topic': f'perception/{name}/lane_mask',
                'raw_mask_topic': f'perception/{name}/lane_mask_raw',
                'process_rate_hz': ParameterValue(process_rate, value_type=float),
                'publish_raw_mask': ParameterValue(publish_raw, value_type=bool),
                'processing_width': ParameterValue(processing_width, value_type=int),
            }],
        ))

    projection_node = Node(
        package='imprimis_perception',
        executable='lane_projection',
        output='screen',
        parameters=[{
            'camera_names': list(CAMERAS),
            'mask_topics': mask_topics,
            'camera_info_topics': camera_info_topics,
            'base_frame': base_frame,
            'output_topic': 'perception/lane_points',
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('process_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('publish_raw_mask', default_value='false'),
        DeclareLaunchArgument('processing_width', default_value='640',
                              description='Maximum mask width; zero keeps native resolution.'),
        *segmentation_nodes,
        projection_node,
    ])
