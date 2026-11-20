from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_dir = PathJoinSubstitution([FindPackageShare('imprimis_perception'), 'launch'])
    use_hardware = LaunchConfiguration('use_hardware')
    use_nav = LaunchConfiguration('use_nav')
    nav_mode = LaunchConfiguration('nav_mode')
    record = LaunchConfiguration('record')

    segmentation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([launch_dir, 'lane_segmentation.launch.py'])),
        launch_arguments={'camera_name': 'd435i'}.items(),
    )
    projection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([launch_dir, 'lane_projection.launch.py'])),
    )

    # use_cams stays false in both: their camera driver would fight the one lane_segmentation starts
    # basic_nav already starts imprimis_hardware, so hardware on its own only runs without nav
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('imprimis_hardware_platform'), 'launch', 'imprimis_hardware.launch.py'])),
        launch_arguments={
            'use_gps': PythonExpression(["'true' if '", nav_mode, "' == 'outdoor' else 'false'"]),
            'use_cams': 'false',
        }.items(),
        condition=IfCondition(PythonExpression(["'", use_hardware, "' == 'true' and '", use_nav, "' != 'true'"])),
    )
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('imprimis_navigation'), 'launch', 'basic_nav.launch.py'])),
        launch_arguments={
            'nav_mode': nav_mode,
            'nav2_params': 'Lane2027',
            'use_cams': 'false',
        }.items(),
        condition=IfCondition(use_nav),
    )
    costmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([launch_dir, 'lane_costmap.launch.py'])),
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration('standalone_costmap'), "' == 'true' and '", use_nav, "' != 'true'"])),
    )
    recorder = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([launch_dir, 'lane_bag_record.launch.py'])),
        launch_arguments={
            'bag': record,
            'duration': LaunchConfiguration('duration'),
            'iteration': LaunchConfiguration('iteration'),
            'camera_name': 'd435i',
        }.items(),
        condition=IfCondition(PythonExpression(["'", record, "' != ''"])),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_hardware', default_value='false', choices=('true', 'false'),
                              description='Start imprimis_hardware (drive, LiDAR, IMU, GPS).'),
        DeclareLaunchArgument('use_nav', default_value='false', choices=('true', 'false'),
                              description='Start basic_nav with Lane2027. It starts the hardware itself.'),
        DeclareLaunchArgument('nav_mode', default_value='indoor', choices=('indoor', 'outdoor'),
                              description='indoor: SLAM map frame, no GPS. outdoor: GPS map frame.'),
        DeclareLaunchArgument('standalone_costmap', default_value='false', choices=('true', 'false'),
                              description='Start a standalone Lane2027 local costmap when nav is not running.'),
        DeclareLaunchArgument('record', default_value='',
                              description='Bag name to record a lane bag under lane_iterations/<iteration>/RealBags; '
                                          'empty to not record.'),
        DeclareLaunchArgument('duration', default_value='30', description='Seconds to record.'),
        DeclareLaunchArgument('iteration', default_value='1', description='lane_iterations folder to record into.'),
        segmentation, projection, hardware, nav, costmap, recorder,
    ])
