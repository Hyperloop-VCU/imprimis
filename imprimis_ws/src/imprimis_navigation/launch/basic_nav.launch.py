from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "hardware_type",
            default_value="real",
            choices=("real", "simulated"),
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
            "world",
            default_value="warehouse",
            description="World file used for simulation (excluding the .sdf). It must be located in imprimis_hardware_platform/worlds",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "nav2_params",
            default_value="SmacHybrid_RPP_1",
            description="Filename of the nav2 parameters YAML (excluding the .yaml). It must be located in imprimis_navigation/config/nav2",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "disable_local_ekf",
            default_value='false',
            description="If false, a local EKF node fuses wheel odom with other local odom sources. If true, wheel odom is the sole local odom source.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "autostart_nav2",
            default_value="true",
            description="Autostart Nav2 lifecycle nodes.",
        )
    )

    hardware_type = LaunchConfiguration("hardware_type")
    use_controller = LaunchConfiguration("use_controller")
    autostart_nav2 = LaunchConfiguration("autostart_nav2")
    disable_local_EKF = LaunchConfiguration("disable_local_ekf")
    nav2_params = LaunchConfiguration("nav2_params")
    world = LaunchConfiguration("world")

    nav_config_src_dir = PathJoinSubstitution([FindPackageShare("imprimis_navigation"), '../../../../src/imprimis_navigation/config'])

    # hardware and localization (real or simulated)
    localization_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare("imprimis_navigation"), "launch", "localization.launch.py"])]),
        launch_arguments={
            "hardware_type": hardware_type,
            "use_controller": use_controller,
            "disable_local_EKF": disable_local_EKF,
            "world": world
        }.items(),
    )

    # Helper node to wait for map -> odom tf and exit once it's available
    wait_for_map_odom_tf = Node(
        package="utils",
        executable="wait_for_tf",
        parameters=[{
            "source_frame": "map",
            "target_frame": "odom"
        }]
    )

    # nav2 navigation stack (planner/controller/bt/costmaps)
    nav2_params_file_path = PathJoinSubstitution([nav_config_src_dir, 'nav2', [nav2_params, '.yaml']])
    nav2_navigation_launch = RegisterEventHandler(OnProcessExit(
        target_action=wait_for_map_odom_tf,
        on_exit=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("imprimis_navigation"), "launch", "nav2_minimal_bringup.launch.py"])),
            launch_arguments={
                "params_file": nav2_params_file_path,
                "autostart": autostart_nav2,
                "use_sim_time": PythonExpression(["'", hardware_type, "' == 'simulated'"])
            }.items(),
        )]
    ))

    # gps_nav_bridge
    gps_nav_bridge_params = PathJoinSubstitution([nav_config_src_dir, "gps_nav_bridge.yaml"])
    gps_nav_bridge_node = Node(
        package="gps_nav_bridge",
        executable="gps_nav_bridge",
        name="gps_nav_bridge",
        parameters=[gps_nav_bridge_params, {"use_sim_time": PythonExpression(["'", hardware_type, "' == 'simulated'"])}],
    )

    return LaunchDescription(declared_arguments + [
        localization_launch_include,
        gps_nav_bridge_node,

        # wait for map -> odom tf before launching nav2 stack
        wait_for_map_odom_tf,

        # nav2 stack
        nav2_navigation_launch
    ])


"""
Old gps nav bridge stuff
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

    # gps_nav_bridge for real gps on serial
    gps_nav_bridge_serial_node = Node(
        package="gps_nav_bridge",
        executable="gps_nav_bridge",
        name="gps_nav_bridge",
        parameters=[gps_nav_bridge_serial_params],
        condition=use_serial_condition,
    )

    # gps_nav_bridge for real gps on board A
    gps_nav_bridge_gpio_node = Node(
        package="gps_nav_bridge",
        executable="gps_nav_bridge_gpio",
        name="gps_nav_bridge_gpio",
        parameters=[gps_nav_bridge_gpio_params],
        condition=use_gpio_condition,
    )
    gps_nav_bridge_serial_params = PathJoinSubstitution(
        [
            FindPackageShare("imprimis_navigation"),
            "config",
            "gps_nav_bridge",
            "gps_nav_bridge_serial_auto.yaml",
        ]
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
declared_arguments.append(
        DeclareLaunchArgument(
            "use_gpio",
            default_value="false",
            description="Whether or not to get GPS data from Board A or Serial",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_gps",
            default_value="true",
            description="Whether or not to use a fake gps, simulated from /odometry/filtered/local."
        )
    )

"""

"""
Old map server stuff

    declared_arguments.append(
        DeclareLaunchArgument(
            "map_yaml",
            default_value="blank.yaml",
            description="Filename of the map YAML. This file must be located in imprimis_navigation/config/nav2",
        )
    )

# map server
    map_yaml_path = PathJoinSubstitution([nav_config_src_dir, 'nav2', map_yaml])
    map_server_node = RegisterEventHandler(OnProcessExit(
        target_action=wait_for_map_odom_tf,
        on_exit=[Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{
                "yaml_filename": map_yaml_path, 
                "use_sim_time": PythonExpression(["'", hardware_type, "' == 'simulated'"])
            }],
        )]
    ))

    # lifecycle manager for map server
    lifecycle_manager_map = RegisterEventHandler(OnProcessExit(
        target_action=wait_for_map_odom_tf,
        on_exit=[Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_map",
            output="screen",
            parameters=[{
                "autostart": True,
                "node_names": ["map_server"],
                "use_sim_time": PythonExpression(["'", hardware_type, "' == 'simulated'"]),
            }],
        )]
    ))
"""