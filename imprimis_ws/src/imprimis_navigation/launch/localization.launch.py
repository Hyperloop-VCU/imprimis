from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Declare and initialize arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "hardware_type",
            default_value="real",
            choices=("real", "fake"),
            description="Choose between real or fake hardware.",
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
            "use_fake_gps",
            default_value="false",
            description="Whether or not to use a fake gps, simulated from /odometry/filtered."
        )
    )
    hardware_type = LaunchConfiguration("hardware_type")
    use_controller = LaunchConfiguration("use_controller")
    use_fake_gps = LaunchConfiguration("use_fake_gps")

    # hardware (real or fake)
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
                'publish_odom_tf': 'false',
                }.items(),
        )
    
    # local EKF
    local_ekf_params_file = PathJoinSubstitution(
        [
            FindPackageShare("imprimis_navigation"),
            "config",
            "local_ekf_config.yaml",
        ]
    )
    local_ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        parameters=[local_ekf_params_file],
    )

    # t265
    t265_driver_node = Node(
        package="t265_ros_driver",
        executable="main"
    )

    # GPS (real or fake, only one of these two is ever launched)
    fake_gps_node = Node(
        package="sim_gps_from_odom",
        executable="sim_gps_from_odom",
        condition=IfCondition(use_fake_gps),
    )
    real_gps_node = Node(
        package="gps_nav_bridge",
        executable="gps_nav_bridge",
        condition=UnlessCondition(use_fake_gps),
    )
    
    # navsat transform node
    navsat_transform_params_file = PathJoinSubstitution(
        [
            FindPackageShare("imprimis_navigation"),
            "config",
            "navsat_transform_params.yaml",
        ]
    )
    navsat_transform_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        parameters=[navsat_transform_params_file]
    )

    # global EKF
    global_ekf_params_file = PathJoinSubstitution(
        [
            FindPackageShare("imprimis_navigation"),
            "config",
            "global_ekf_config.yaml",
        ]
    )
    global_ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        parameters=[global_ekf_params_file],
    )

    # IMU
    bno055_params_file = PathJoinSubstitution(
        [
            FindPackageShare("imprimis_navigation"),
            "config",
            "bno055_params.yaml",
        ]
    )
    bno055_node = Node(
        package="bno055",
        executable="bno055",
        parameters=[bno055_params_file]
    )

    return LaunchDescription(declared_arguments + [
        imprimis_hardware_launch_include,
        local_ekf_node,
        t265_driver_node,
        real_gps_node,
        fake_gps_node,
        #global_ekf_node,
        navsat_transform_node,
        bno055_node
    ])