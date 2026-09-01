from launch import LaunchDescription
from launch.substitutions import PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, SetEnvironmentVariable, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
import yaml

def generate_launch_description():

    # Declare and initialize arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "publish_odom_tf",
            default_value="true",
            choices=("true", "false"),
            description="If false, prevents the diff drive controller broadcasting the odom->base_link transform from wheel odometry. This MUST be false if there is another node broadcasting odom -> base_link (e.g. the local EKF).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_controller",
            default_value="false",
            choices=("true", "false"),
            description="Whether or not to start up the logitech controller input node",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "lidar_rpm",
            default_value="600.0",
            choices=("300.0", "600.0", "1200.0"),
            description="Velodyne LIDAR RPM. Keep at 600.0"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_cams",
            default_value="false",
            choices=("true", "false"),
            description="Whether or not to start up the intel depth cameras.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_lidar",
            default_value="true",
            choices=("true", "false"),
            description="Whether or not to start up the nodes reading from the VLP16 lidar.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_gps",
            default_value="true",
            choices=("true", "false"),
            description="Whether or not to start up the GPS driver.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_imu",
            default_value="true",
            choices=("true", "false"),
            description="Whether or not to start up the IMU driver.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ui_type",
            default_value="foxglove",
            choices=("foxglove", "rviz", "none"),
            description="Use foxglove_bridge + (separate) foxglove studio, rosbridge + (separate) gps goal input + rviz, or no bridges / UIs.",
        )
    )
    
    use_controller = LaunchConfiguration("use_controller")
    publish_odom_tf = LaunchConfiguration("publish_odom_tf")
    lidar_rpm = LaunchConfiguration("lidar_rpm")
    use_cams = LaunchConfiguration("use_cams")
    use_lidar = LaunchConfiguration("use_lidar")
    use_gps = LaunchConfiguration("use_gps")
    use_imu = LaunchConfiguration("use_imu")
    ui_type = LaunchConfiguration("ui_type")

    # Get package directories from the workspace source folder
    # This allows us to pass params files to nodes from SOURCE, not from install, preventing us from needing to rebuild every time we change parameters
    hardware_src_dir = PathJoinSubstitution([FindPackageShare("imprimis_hardware_platform"), '../../../../src/imprimis_hardware_platform'])
    hardware_src_dir_os = os.path.join(get_package_share_directory("imprimis_hardware_platform"), '../../../../src/imprimis_hardware_platform')
    description_src_dir = PathJoinSubstitution([hardware_src_dir, '../imprimis_description'])
    description_src_dir_os = os.path.join(hardware_src_dir_os, '../imprimis_description')

    # Get URDF via xacro and pass arguments to it
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([description_src_dir, "urdf", "diffbot.urdf.xacro"]),
        " ",
        "hardware_type:=",
        "real",
        " publish_odom_tf:=",
        publish_odom_tf
    ])
    robot_description = {"robot_description": robot_description_content}

    
    # controller manager
    controllers_config = PathJoinSubstitution([hardware_src_dir, "config", 'diffbot_controllers.yaml'])
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_config, {"enable_odom_tf": publish_odom_tf}],
        output="both",
        remappings=[("~/robot_description", "/robot_description")],
        arguments=["--ros-args", "--log-level", "info"]
    )

    # robot state publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": False}],
        arguments=["--ros-args", "--log-level", "warn"]
    )

    # rviz
    rviz_config_file = PathJoinSubstitution([description_src_dir, "rviz", "diffbot.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file, "--ros-args", "--log-level", "warn"],
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(PythonExpression(["'", ui_type, "' == 'rviz'"]))
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        parameters=[{"use_sim_time": False}],
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", "--ros-args", "--log-level", "warn"],
    )

    # Diff drive controller spawner
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        parameters=[{"use_sim_time": False}],
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager", "--ros-args", "--log-level", "warn"],
    )

    # GPIO controller spawner
    gpio_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        parameters=[{"use_sim_time": False}],
        arguments=["gpio_controller", "--controller-manager", "/controller_manager", "--ros-args", "--log-level", "warn"]
    )

    # Velodyne LIDAR driver, parser, and republisher
    lidar_driver_config_file = PathJoinSubstitution([hardware_src_dir, "config", "VLP16-driver-params.yaml"])
    lidar_transform_config_file = os.path.join(hardware_src_dir_os, 'config', 'VLP16-transform-params.yaml')
    with open(lidar_transform_config_file, 'r') as f:
        lidar_transform_config = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']
    lidar_transform_config['calibration'] = os.path.join(hardware_src_dir_os, 'config', 'VLP16-transform-calibration.yaml')

    velodyne_driver_node = Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        output='both',
        parameters=[lidar_driver_config_file, {"rpm": lidar_rpm}],
        condition=IfCondition(use_lidar),
        arguments=["--ros-args", "--log-level", "warn"]
    )
    velodyne_transform_node = Node(
        package='velodyne_pointcloud',
        executable='velodyne_transform_node',
        output='log', # shut up
        parameters=[lidar_transform_config],
        condition=IfCondition(use_lidar),
        arguments=["--ros-args", "--log-level", "error"]
    )
    lidar_delay_fixer = Node(
        package="utils",
        executable="fix_lidar_delay",
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(use_lidar)
    )
    
    # IMU driver, after a delay
    imu_driver = TimerAction(
        period = 2.0,
        actions = [
            Node(
                package="umx_driver",
                executable="um7_driver",
                parameters=[{"port": "/dev/ttyUSB1"}],
                arguments=["--ros-args", "--log-level", "warn"]
            )
        ],
        condition=IfCondition(use_imu)
    )

    # Calibrate IMU after a delay
    imu_calibrator = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call',
                        '/imu/reset',
                        'umx_driver/srv/Um7Reset',
                        '{zero_gyros: true, reset_ekf: true, set_mag_ref: true}'
                    ],
                    output='screen'
                )
        ],
        condition=IfCondition(use_imu)
    )
    gps_nmea_driver = Node(
        package="nmea_navsat_driver",
        executable="nmea_serial_driver",
        condition=IfCondition(use_gps),
        arguments=["--ros-args", "--log-level", "warn"],
        parameters=[{
            'port': '/dev/ttyACM0',
            'baud': 9600,
            'frame_id': 'gps_link'
        }, {"use_sim_time": False}],
        namespace="gps"
    )

    # Camera driver
    """
    # The GroupAction with forwarding=False and scoped=True prevents the camera launch file from seeing this launch file's arguments.
    # We don't want to tell the camera "publish_odom_tf=false".
    camera_launch_include = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])]),
                launch_arguments={
                    'pointcloud.enable': 'true',
                    'diagnostics_period': '1.0',
                    'log_level': 'error',
                    'camera_namespace': 'cameras',
                    'camera_name': 'front'
                }.items(),
            )
        ],
        scoped=True,
        forwarding=False,
        condition=IfCondition(use_cams),
    )
    """

    camera_launch_include = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('utils'), 'launch', 'multicam_test.launch.py'])]),
            )
        ],
        scoped=True,
        forwarding=False,
        condition=IfCondition(use_cams),
    )


    # Controller input
    controller_input_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('teleop_twist_joy'), 'launch', 'teleop-launch.py'])]),
        launch_arguments={
            'joy_config': 'ps3',
            "publish_stamped_twist": 'true',
            'frame': 'base_link',
            'joy_vel': 'diffbot_base_controller/cmd_vel'
        }.items(),
        condition=IfCondition(use_controller)
    )

    # rosbridge
    #ros2 launch rosbridge_server rosbridge_websocket_launch.xml
    rosbridge_launch_include = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml'])]),
        launch_arguments={'port': '9091'}.items(),
        condition=IfCondition(PythonExpression(["'", ui_type, "' == 'rviz'"]))
    )

    # foxglove bridge
    foxglove_launch_include = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml'])]),
        condition=IfCondition(PythonExpression(["'", ui_type, "' == 'foxglove'"]))
    )

    things_to_launch = [
        # Always
        robot_state_pub_node,
        robot_controller_spawner,
        joint_state_broadcaster_spawner,
        controller_manager_node,
        gpio_controller_spawner,

        # Not always
        imu_driver,
        imu_calibrator,
        gps_nmea_driver,
        velodyne_driver_node,
        velodyne_transform_node,
        camera_launch_include,
        controller_input_launch_include,
        lidar_delay_fixer,
        rviz_node,
        rosbridge_launch_include,
        foxglove_launch_include,
    ]

    return LaunchDescription(declared_arguments + things_to_launch)


# Remap wheel odometry topic to mirror ekf output if publish_odom_tf is false, so higher level stuff doesn't have to care
    #odom_remapper = Node(
    #    package="utils",
    #    executable="odom_remapper",
    #    condition=IfCondition(publish_odom_tf),
    #    parameters=[{"use_sim_time": PythonExpression(["'", hardware_type, "' == 'simulated'"])}]
    #)


# Add covariance to simulated GPS
    # gps_covariance_fixer = Node(
    #    package="utils",
    #    executable="gps_add_sim_covariance",
    #    condition=IfCondition(PythonExpression(["'", hardware_type, "' != 'fake'"])),
    #    parameters=[{"use_sim_time": PythonExpression(["'", hardware_type, "' == 'simulated'"])}]
    #)
    