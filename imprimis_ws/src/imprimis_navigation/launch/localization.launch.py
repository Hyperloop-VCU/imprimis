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
    declared_arguments.append(
        DeclareLaunchArgument(
            "world",
            default_value="warehouse",
            description="World file used for simulation (excluding the .sdf). It must be located in imprimis_hardware_platform/worlds",
        )
    )
    hardware_type = LaunchConfiguration("hardware_type")
    use_controller = LaunchConfiguration("use_controller")
    disable_local_ekf = LaunchConfiguration("disable_local_ekf")
    world = LaunchConfiguration("world")

    nav_config_src_dir = PathJoinSubstitution([FindPackageShare("imprimis_navigation"), '../../../../src/imprimis_navigation/config'])

    # Lidar SLAM. This is ran after odom->base_link is available
    slam_config = PathJoinSubstitution([nav_config_src_dir, "SLAM.yaml"])
    SLAM_scanmatcher_node = Node(
        package='scanmatcher',
        executable='scanmatcher_node',
        parameters=[slam_config, {"use_sim_time": PythonExpression(["'", hardware_type, "' == 'simulated'"])}],
        remappings=[('/input_cloud','/velodyne_points'), ("imu", "imu/data")],
        output='screen',
        arguments=["--ros-args", "--log-level", "info"],
    )

    # hardware (real or simulated)
    imprimis_hardware_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('imprimis_hardware_platform'), 'launch', 'imprimis_hardware.launch.py'])]),
        launch_arguments={
            'hardware_type': hardware_type,
            'use_controller': use_controller,
            'publish_odom_tf': disable_local_ekf,
            'world': world
        }.items(),
    )
    
    # Local EKF node for local odom fusion from multiple sources
    local_ekf_config = PathJoinSubstitution([ nav_config_src_dir, "Local_EKF.yaml"])
    local_ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local",
        parameters=[local_ekf_config, {"use_sim_time": PythonExpression(["'", hardware_type, "' == 'simulated'"])}],
        remappings=[('/odometry/filtered', '/odometry/filtered/local')],
        arguments=["--ros-args", "--log-level", "warn"],
        condition=UnlessCondition(disable_local_ekf)
    )

    # Helper node to wait for odom -> base_link
    wait_for_odom_tf_and_lidar = Node(
        package="utils",
        executable="wait_for_tf",
        parameters=[{
            "source_frame": "odom",
            "target_frame": "base_link",
        }]
    )

    # Lidar SLAM, after hardware/EKF has made odom -> base_link available
    SLAM_scanmatcher_run = RegisterEventHandler(OnProcessExit(
        target_action=wait_for_odom_tf_and_lidar,
        on_exit=[SLAM_scanmatcher_node] 
    ))

    return LaunchDescription(declared_arguments + [
        imprimis_hardware_launch_include,
        local_ekf_node,
        wait_for_odom_tf_and_lidar,
        SLAM_scanmatcher_run
    ])