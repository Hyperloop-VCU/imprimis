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
            "use_gpio",
            default_value="true",
            description="Whether or not to get GPS data from Board A or Serial",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_gps",
            default_value="false",
            description="Whether or not to use a fake gps, simulated from /odometry/filtered/local."
        )
    )
    hardware_type = LaunchConfiguration("hardware_type")
    use_controller = LaunchConfiguration("use_controller")
    use_fake_gps = LaunchConfiguration("use_fake_gps")
    use_gpio = LaunchConfiguration("use_gpio")

    # when use_fake_gps = false, choose between gpio vs serial
    use_gpio_condition = IfCondition(
    PythonExpression([
        "'", use_fake_gps, "' == 'false' and '", use_gpio, "' == 'true'"
    ])
)

    use_serial_condition = IfCondition(
    PythonExpression([
        "'", use_fake_gps, "' == 'false' and '", use_gpio, "' == 'false'"
    ])
)


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
        parameters=[local_ekf_params_file],
        remappings=[('/odometry/filtered', '/odometry/filtered/local')]
    )

    # t265
    t265_driver_node = Node(
        package="t265_ros_driver",
        executable="main"
    )

    # gps param files
    gps_nav_bridge_sim_params = PathJoinSubstitution(
        [
            FindPackageShare("imprimis_navigation"),
            "config",
            "gps_nav_bridge",
            "gps_nav_bridge_sim.yaml",
        ]
    )

    gps_nav_bridge_serial_params = PathJoinSubstitution(
        [
            FindPackageShare("imprimis_navigation"),
            "config",
            "gps_nav_bridge",
            "gps_nav_bridge_serial_auto.yaml",
        ]
    )

    sim_gps_params_file = PathJoinSubstitution(
        [
            FindPackageShare("imprimis_navigation"),
            "config",
            "sim_gps_from_odom.yaml",
        ]
    )
    gps_nav_bridge_gpio_params = PathJoinSubstitution(
        [
            FindPackageShare("imprimis_navigation"),
            "config",
            "gps_nav_bridge",
            "gps_nav_bridge_gpio_auto.yaml",
        ]
    )

    # fake gps
    fake_gps_node = Node(
        package="sim_gps_from_odom",
        executable="sim_gps_from_odom",
        name="sim_gps_from_odom",
        parameters=[sim_gps_params_file],
        condition=IfCondition(use_fake_gps),
    )

    # gps_nav_bridge for sim gps
    gps_nav_bridge_sim_node = Node(
        package="gps_nav_bridge",
        executable="gps_nav_bridge",
        name="gps_nav_bridge",
        parameters=[gps_nav_bridge_sim_params],
        condition=IfCondition(use_fake_gps),
    )

    # gps_nav_bridge for real gps
    gps_nav_bridge_serial_node = Node(
        package="gps_nav_bridge",
        executable="gps_nav_bridge",
        name="gps_nav_bridge",
        parameters=[gps_nav_bridge_serial_params],
        condition=use_serial_condition,
    )

    # gps_nav_bridge for gps data from board a (gpio)
    gps_nav_bridge_gpio_node = Node(
        package="gps_nav_bridge",
        executable="gps_nav_bridge_gpio",
        name="gps_nav_bridge_gpio",
        parameters=[gps_nav_bridge_gpio_params],
        condition=use_gpio_condition,
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
        parameters=[navsat_transform_params_file],
        remappings=[('/imu', '/bno055/imu'), ('/odometry/filtered', '/odometry/filtered/local'), ('/gps/fix', '/gps/fix')],
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
        parameters=[global_ekf_params_file],
        remappings=[('/odometry/filtered', '/odometry/filtered/global')]
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
    
    """
    I'm keeping this here because Nav2 isnt launched here.
    Nav2 usually handles the conversion so this will be ommitted
    in the basic_nav launch file. 
    """
    # cmd_vel twist to twist stamped 
    twist_to_stamped_params_file = PathJoinSubstitution(
        [
            FindPackageShare("imprimis_navigation"),
            "config",
            "twist_to_stamped.yaml",
        ]
    )
    twist_to_stamped_node = Node(
        package="twist_to_stamped",
        executable="twist_to_stamped",
        name="twist_to_stamped",
        parameters=[twist_to_stamped_params_file],
    )

    return LaunchDescription(declared_arguments + [
        imprimis_hardware_launch_include,
        local_ekf_node,
        t265_driver_node,
        gps_nav_bridge_sim_node,
        gps_nav_bridge_serial_node,
        gps_nav_bridge_gpio_node,
        fake_gps_node,
        global_ekf_node,
        navsat_transform_node,
        bno055_node,
        twist_to_stamped_node,
    ])