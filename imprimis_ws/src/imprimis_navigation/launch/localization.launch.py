from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PythonExpression


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
    hardware_type = LaunchConfiguration("hardware_type")
    use_controller = LaunchConfiguration("use_controller")

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
                'publish_odom_tf': 'false'
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
        name="ekf_local",
        parameters=[local_ekf_params_file, {"use_sim_time": PythonExpression(["'", hardware_type, "' == 'simulated'"])}],
        remappings=[('/odometry/filtered', '/odometry/filtered/local')]
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
        parameters=[navsat_transform_params_file, {"use_sim_time": PythonExpression(["'", hardware_type, "' == 'simulated'"])}],
        remappings=[("/imu", "/imu/data"), ('/odometry/filtered', '/odometry/filtered/local'), ('/gps/fix', '/gps/fix')],
        arguments=["--ros-args", "--log-level", "warn"],
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
        name="ekf_global",
        parameters=[global_ekf_params_file, {"use_sim_time": PythonExpression(["'", hardware_type, "' == 'simulated'"])}],
        remappings=[('/odometry/filtered', '/odometry/filtered/global')]
    )
   

    return LaunchDescription(declared_arguments + [
        # always
        imprimis_hardware_launch_include,
        local_ekf_node,
        global_ekf_node,
        navsat_transform_node,
    ])