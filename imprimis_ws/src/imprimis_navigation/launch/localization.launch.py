from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PythonExpression
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    # Declare and initialize arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "hardware_type",
            default_value="real",
            choices=("real", "simulated"),
            description="Choose between real or simulated hardware.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_controller",
            default_value="false",
            description="Whether or not to start up the logitech controller input node.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "disable_local_ekf",
            default_value="false",
            description="If false, a local EKF node fuses wheel odom with other local odom sources. If true, wheel odom is the sole local odom source.",
        )
    )
    hardware_type = LaunchConfiguration("hardware_type")
    use_controller = LaunchConfiguration("use_controller")
    disable_local_ekf = LaunchConfiguration("disable_local_ekf")

    # Get robot localization nodes config file
    roboloco_config = PathJoinSubstitution(
        [
            FindPackageShare("imprimis_navigation"),
            "config",
            "localization_nodes_config.yaml",
        ]
    )

    # Declare lidar SLAM launch include. This is ran after odom->base_link is available
    lidar_SLAM_launch_source = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('lidarslam'),
            'launch',
            'lidarslam.launch.py'
        ]),
    ) 

    # hardware (real or simulated)
    imprimis_hardware_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('imprimis_hardware_platform'),
                'launch',
                'imprimis_hardware.launch.py'
            ])
        ]),
        launch_arguments={
                'hardware_type': hardware_type,
                'use_controller': use_controller,
                'publish_odom_tf': disable_local_ekf
                }.items(),
    )
    
    # Local EKF node for local odom fusion from multiple sources
    local_ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local",
        parameters=[roboloco_config, {"use_sim_time": PythonExpression(["'", hardware_type, "' == 'simulated'"])}],
        remappings=[('/odometry/filtered', '/odometry/filtered/local')],
        arguments=["--ros-args", "--log-level", "warn"],
        condition=UnlessCondition(disable_local_ekf)
    )

    # Helper node to wait for odom -> base_link and exit once it's available
    wait_for_odom_tf = Node(
        package="utils",
        executable="wait_for_tf",
        parameters=[{
            "source_frame": "odom",
            "target_frame": "base_link"
        }]
    )

    # Lidar SLAM, after hardware/EKF has made odom -> base_link available
    lidar_SLAM_launch = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_odom_tf,
            on_exit=[
                lidar_SLAM_launch_source
            ]
        )
    )
    navsat_transform_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        parameters=[roboloco_config, {"use_sim_time": PythonExpression(["'", hardware_type, "' == 'simulated'"])}],
        remappings=[("/imu", "/imu/data"), ('/odometry/filtered', '/odometry/filtered/global')],
        arguments=["--ros-args", "--log-level", "warn"],
    )
   

    return LaunchDescription(declared_arguments + [
        imprimis_hardware_launch_include,
        local_ekf_node,
        wait_for_odom_tf,
        lidar_SLAM_launch
        #navsat_transform_node,
    ])


"""
Old global EKF node
global_ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_global",
        parameters=[roboloco_config, {"use_sim_time": PythonExpression(["'", hardware_type, "' == 'simulated'"])}],
        remappings=[('/odometry/filtered', '/odometry/filtered/global')]
    )
"""