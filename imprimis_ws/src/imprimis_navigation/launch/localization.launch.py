from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
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
    hardware_type = LaunchConfiguration("hardware_type")
    use_controller = LaunchConfiguration("use_controller")

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
    


    ekf_params_file = PathJoinSubstitution(
        [
            FindPackageShare("imprimis_navigation"),
            "config",
            "ekf_config.yaml",
        ]
    )
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        parameters=[ekf_params_file],
    )
    t265_driver_node = Node(
        package="t265_ros_driver",
        executable="main"
    )



    return LaunchDescription(declared_arguments + [
        imprimis_hardware_launch_include,
        ekf_node,
        t265_driver_node
    ])