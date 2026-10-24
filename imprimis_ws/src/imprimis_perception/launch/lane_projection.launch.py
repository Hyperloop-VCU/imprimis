"""Start the four-camera projector on the laptop that receives masks from Pis."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


CAMERAS = ('d455', 'd435i', 'd435_1', 'd435_2')


def generate_launch_description():
    base_frame = LaunchConfiguration('base_frame')
    ground_z = LaunchConfiguration('ground_z')
    output_z = LaunchConfiguration('output_z')
    mask_topics = [f'/perception/{name}/lane_mask' for name in CAMERAS]
    camera_info_topics = [f'/cameras/{name}/color/camera_info' for name in CAMERAS]
    projector = Node(
        package='imprimis_perception',
        executable='lane_projection',
        output='screen',
        parameters=[{
            'camera_names': list(CAMERAS),
            'mask_topics': mask_topics,
            'camera_info_topics': camera_info_topics,
            'base_frame': base_frame,
            'ground_z': ParameterValue(ground_z, value_type=float),
            'output_z': ParameterValue(output_z, value_type=float),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('ground_z', default_value='-0.1277'),
        DeclareLaunchArgument('output_z', default_value='0.5'),
        projector,
    ])
