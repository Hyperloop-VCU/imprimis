from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import UnlessCondition, IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PythonExpression
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    # Declare and initialize arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "map_type",
            default_value="lidar",
            choices=("lidar", "gps", "fake"),
            description="If lidar, the map frame is generated from SLAM. If gps, the map frame is generated from fusing local odom with GPS. If fake, map is identical to odom."
        )
    )
    map_type = LaunchConfiguration("map_type")

    nav_config_src_dir = PathJoinSubstitution([FindPackageShare("imprimis_navigation"), '../../../../src/imprimis_navigation/config'])

    # Helper node to wait for odom -> base_link
    wait_for_odom_tf = Node(
        package="utils",
        executable="wait_for_tf",
        parameters=[{
            "source_frame": "odom",
            "target_frame": "base_link",
        }]
    )

    # SLAM-based map frame
    slam_config = PathJoinSubstitution([nav_config_src_dir, "SLAM.yaml"])
    SLAM_scanmatcher = RegisterEventHandler(OnProcessExit(
        target_action=wait_for_odom_tf,
        on_exit=[Node(
            package='scanmatcher',
            executable='scanmatcher_node',
            parameters=[slam_config],
            remappings=[('/input_cloud','/velodyne_points'), ("imu", "imu/data")],
            output='screen',
            arguments=["--ros-args", "--log-level", "info"],
            condition=IfCondition(PythonExpression(["'", map_type, "' == 'lidar'"]))
        )],
    ))

    # Faked map frame
    map_faker = RegisterEventHandler(OnProcessExit(
        target_action=wait_for_odom_tf,
        on_exit=[Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["--frame-id", "map", "--child-frame-id", "odom", "--x", "0", "--y", "0", "--z", "0", "--roll", "0", "--pitch", "0", "--yaw", "0"],
            condition=IfCondition(PythonExpression(["'", map_type, "' == 'fake'"]))
        )],
    ))

    # GPS-based map frame
    navsat_transform_config = PathJoinSubstitution([nav_config_src_dir, "navsat_transform.yaml"])
    global_ekf_config = PathJoinSubstitution([nav_config_src_dir, "Global_EKF.yaml"])
    navsat_transform = RegisterEventHandler(OnProcessExit(
        target_action=wait_for_odom_tf,
        on_exit=[Node(
            package="robot_localization",
            executable="navsat_transform_node",
            parameters=[navsat_transform_config],
            remappings=[('odometry/filtered', 'odometry/filtered/global'), ('imu', 'imu/data')],
            condition=IfCondition(PythonExpression(["'", map_type, "' == 'gps'"]))
        )],
    ))
    global_ekf = RegisterEventHandler(OnProcessExit(
        target_action=wait_for_odom_tf,
        on_exit=[Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_global",
            parameters=[global_ekf_config],
            remappings=[('/odometry/filtered', '/odometry/filtered/global')],
            arguments=["--ros-args", "--log-level", "warn"],
            condition=IfCondition(PythonExpression(["'", map_type, "' == 'gps'"]))
        )],
    ))
    gps_monitor = Node(
        package="utils",
        executable="gps_monitor",
        condition=IfCondition(PythonExpression(["'", map_type, "' == 'gps'"]))
    )

    return LaunchDescription(declared_arguments + [
        # If map_type is lidar
        SLAM_scanmatcher,
        wait_for_odom_tf,

        # If map_type is gps
        navsat_transform,
        global_ekf,
        gps_monitor,

        # If map_type is fake
        map_faker
    ])
