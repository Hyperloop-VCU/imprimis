from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    height = LaunchConfiguration('height')
    tilt = LaunchConfiguration('tilt')

    # base_link sits on the ground under the tripod; odom lets the costmap and rviz configs that use it run
    odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tripod_odom',
        arguments=['--frame-id', 'odom', '--child-frame-id', 'base_footprint'],
    )
    ground = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tripod_ground',
        arguments=['--frame-id', 'base_footprint', '--child-frame-id', 'base_link'],
    )
    mount = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tripod_mount',
        arguments=['--z', height, '--pitch', PythonExpression([tilt, ' * 3.141592653589793 / 180']),
                   '--frame-id', 'base_link', '--child-frame-id', 'front_link'],
    )
    lanes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('imprimis_perception'), 'launch', 'single_cam.launch.py'])),
        launch_arguments={'standalone_costmap': 'true'}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('height', default_value='0.686', description='Camera height above the ground, m.'),
        DeclareLaunchArgument('tilt', default_value='12.0',
                              description='Degrees the camera looks down, relative to the ground, not gravity.'),
        odom, ground, mount, lanes,
    ])
