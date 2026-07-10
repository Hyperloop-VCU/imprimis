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
            "map_type",
            default_value="lidar",
            choices=("lidar", "gps", "fake"),
            description="If lidar, the map frame is generated from SLAM. If gps, the map frame is generated from fusing local odom with GPS. If fake, map is identical to odom."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "show_sim",
            default_value="true",
            description="If false, runs Gazebo in headless mode (no GUI, just server). If true, runs the GUI like normal."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "map_yaml",
            default_value="bigger_blank",
            description="Filename of the map YAML (excluding the .yaml). This file must be located in imprimis_navigation/config/nav2",
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
            default_value="SmacHybrid_DWB_2",
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

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_controller_switcher",
            default_value="false",
            description="Enable automatic RPP/MPPI controller switching based on obstacle proximity.",
        )
)
    

    declared_arguments.append(
    DeclareLaunchArgument(
        "track_velocity",
        default_value="false",
        description="Enable velocity tracker node.",
    )
)
    
    
    declared_arguments.append(
    DeclareLaunchArgument(
        "use_waypoints",
        default_value="false",
        description="Send waypoints automatically on launch.",
    )
)
    
    hardware_type = LaunchConfiguration("hardware_type")
    use_controller = LaunchConfiguration("use_controller")
    autostart_nav2 = LaunchConfiguration("autostart_nav2")
    disable_local_EKF = LaunchConfiguration("disable_local_ekf")
    nav2_params = LaunchConfiguration("nav2_params")
    world = LaunchConfiguration("world")
    map_type = LaunchConfiguration("map_type")
    show_sim = LaunchConfiguration("show_sim")

    map_yaml = LaunchConfiguration("map_yaml")
    use_controller_switcher = LaunchConfiguration("use_controller_switcher")
    nav_config_src_dir = PathJoinSubstitution([FindPackageShare("imprimis_navigation"), '../../../../src/imprimis_navigation/config'])
    track_velocity = LaunchConfiguration("track_velocity")
    waypoints_file = PathJoinSubstitution([nav_config_src_dir, 'waypoints.yaml'])
    use_waypoints = LaunchConfiguration("use_waypoints")
    # hardware and localization (real or simulated)
    localization_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare("imprimis_navigation"), "launch", "localization.launch.py"])]),
        launch_arguments={
            "hardware_type": hardware_type,
            "use_controller": use_controller,
            "disable_local_EKF": disable_local_EKF,
            "world": world,
            "map_type": map_type,
            "show_sim": show_sim
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

    # map server
    map_yaml_path = PathJoinSubstitution([nav_config_src_dir, 'nav2', [map_yaml, '.yaml']])
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

    # Controller_switcher
    controller_switcher = RegisterEventHandler(OnProcessExit(
        target_action=wait_for_map_odom_tf,
        on_exit=[Node(
            package="imprimis_navigation",
            executable="controller_switcher.py",
            name="controller_switcher",
            output="screen",
            condition=IfCondition(
                PythonExpression([
                    "'", use_controller_switcher, "' == 'true' or '", nav2_params, "' == 'SmacHybrid_HybridRPPMPPI_1'"
                ])
            ),
        )]
    ))

    # map_goal_to_odom
    map_goal_to_odom_params = PathJoinSubstitution([nav_config_src_dir, "map_goal_to_odom.yaml"])
    map_goal_to_odom = RegisterEventHandler(OnProcessExit(
        target_action=wait_for_map_odom_tf,
        on_exit=[Node(
            package="map_goal_to_odom",
            executable="map_goal_to_odom",
            name="map_goal_to_odom",
            parameters=[map_goal_to_odom_params, {"use_sim_time": PythonExpression(["'", hardware_type, "' == 'simulated'"])}],
        )]
    ))
    
    velocity_tracker_node = Node(
        package="imprimis_navigation",
        executable="velocity_tracker.py",
        name="velocity_tracker",
        output="screen",
        condition=IfCondition(track_velocity),
    )
    
    waypoint_sender = RegisterEventHandler(OnProcessExit(
    target_action=wait_for_map_odom_tf,
    on_exit=[Node(
        package="imprimis_navigation",
        executable="waypoint_sender.py",
        name="waypoint_sender",
        output="screen",
        parameters=[{"waypoints_file": waypoints_file}],
        condition=IfCondition(use_waypoints),
    )]
))
    return LaunchDescription(declared_arguments + [
        localization_launch_include,

        # wait for map -> odom tf before starting anything else
        wait_for_map_odom_tf,

        # goal converter node
        map_goal_to_odom,

        # nav2 stack
        map_server_node,
        lifecycle_manager_map,
        nav2_navigation_launch,
        controller_switcher,
        velocity_tracker_node,
        waypoint_sender
    ])