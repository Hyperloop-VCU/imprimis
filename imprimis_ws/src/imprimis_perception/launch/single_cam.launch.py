from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_dir = PathJoinSubstitution([FindPackageShare('imprimis_perception'), 'launch'])

    segmentation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([launch_dir, 'lane_segmentation.launch.py'])),
        launch_arguments={'camera_name': 'd435i'}.items(),
    )
    projection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([launch_dir, 'lane_projection.launch.py'])),
    )

    return LaunchDescription([segmentation, projection])
