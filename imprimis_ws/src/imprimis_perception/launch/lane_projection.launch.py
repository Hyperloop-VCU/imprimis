from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# camera name (the driver's camera_name) -> its link in the URDF
CAMERAS = {
    'd435i': 'front_link',
}


def generate_launch_description():
    mounts = [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'{name}_mount',
            arguments=['--frame-id', urdf_link, '--child-frame-id', f'{name}_link'],
        )
        for name, urdf_link in CAMERAS.items()
    ]
    projector = Node(
        package='imprimis_perception',
        executable='lane_projection',
        output='screen',
        parameters=[{
            'camera_names': list(CAMERAS),
            'base_frame': LaunchConfiguration('base_frame'),
            'output_height': ParameterValue(LaunchConfiguration('output_height'), value_type=float),
            'detect_potholes': ParameterValue(LaunchConfiguration('detect_potholes'), value_type=bool),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('output_height', default_value='0.0',
                              description='Meters above the ground the lane points are published at. '
                                          '0.0 for Lane2027.yaml; about 0.62 for Course2027.yaml.'),
        DeclareLaunchArgument('detect_potholes', default_value='true'),
        *mounts,
        projector,
    ])
