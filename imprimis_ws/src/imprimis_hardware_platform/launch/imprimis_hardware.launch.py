from launch import LaunchDescription
from launch.substitutions import PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
            "show_sim",
            default_value="true",
            choices=("true", "false"),
            description="If false, the simulation will run in headless mode (no GUI). If true, the gazebo GUI will run as usual. Only applicable when hardware_type is simulated."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "publish_odom_tf",
            default_value="true",
            description="If false, prevents the diff drive controller broadcasting the odom->base_link transform from wheel odometry. This MUST be false if there is another node broadcasting odom -> base_link (e.g. the local EKF).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "hardware_type",
            default_value="real",
            choices=("real", "fake", "simulated"),
            description="Choose between real hardware, completely faked hardware, or gazebo-simulated hardware.",
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
            "world",
            default_value="warehouse",
            description="World for gazebo simulation"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "lidar_rpm",
            default_value="600.0",
            choices=("300.0", "600.0", "1200.0"),
            description="Velodyne LIDAR RPM. Only works with real hardware, it's always 600.0 in simulation."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_real_gps",
            default_value="false",
            choices=("true", "false"),
            description="If true, gazebo will not publish the GPS fix topic, and the GPS driver will be started."
        )
    )
    hardware_type = LaunchConfiguration("hardware_type")
    use_controller = LaunchConfiguration("use_controller")
    publish_odom_tf = LaunchConfiguration("publish_odom_tf")
    lidar_rpm = LaunchConfiguration("lidar_rpm")
    show_sim = LaunchConfiguration("show_sim")
    sim_real_gps = LaunchConfiguration("sim_real_gps")

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
        hardware_type,
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
        condition=IfCondition(PythonExpression(["'", hardware_type, "' != 'simulated'"])),
        arguments=["--ros-args", "--log-level", "info"]
    )

    # robot state publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": PythonExpression(["'", hardware_type, "' == 'simulated'"])}],
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
        parameters=[{"use_sim_time": PythonExpression(["'", hardware_type, "' == 'simulated'"])}],
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        parameters=[{"use_sim_time": PythonExpression(["'", hardware_type, "' == 'simulated'"])}],
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", "--ros-args", "--log-level", "warn"],
        
    )

    # Diff drive controller spawner
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        parameters=[{"use_sim_time": PythonExpression(["'", hardware_type, "' == 'simulated'"])}],
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager", "--ros-args", "--log-level", "warn"],
    )

    # GPIO controller spawner
    gpio_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        parameters=[{"use_sim_time": PythonExpression(["'", hardware_type, "' == 'simulated'"])}],
        arguments=["gpio_controller", "--controller-manager", "/controller_manager", "--ros-args", "--log-level", "warn"],
        condition=IfCondition(PythonExpression(["'", hardware_type, "' != 'simulated'"]))
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
        condition=IfCondition(PythonExpression(["'", hardware_type, "' == 'real'"])),
        arguments=["--ros-args", "--log-level", "warn"]
    )
    velodyne_transform_node = Node(
        package='velodyne_pointcloud',
        executable='velodyne_transform_node',
        output='log', # shut up
        parameters=[lidar_transform_config],
        condition=IfCondition(PythonExpression(["'", hardware_type, "' == 'real'"])),
        arguments=["--ros-args", "--log-level", "error"]
    )
    lidar_delay_fixer = Node(
        package="utils",
        executable="fix_lidar_delay",
        parameters=[{"use_sim_time": PythonExpression(["'", hardware_type, "' == 'simulated'"])}]
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
        condition=IfCondition(PythonExpression(["'", hardware_type, "' == 'real'"]))
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
        condition=IfCondition(PythonExpression(["'", hardware_type, "' == 'real'"]))
    )

    # GPS driver
    gps_driver = Node(
        package="nmea_navsat_driver",
        executable="nmea_serial_driver",
        condition=IfCondition(PythonExpression(["'", hardware_type, "' == 'real' or ('", hardware_type, "' == 'simulated' and '", sim_real_gps, "' == 'true')"])),
        arguments=["--ros-args", "--log-level", "warn"],
        parameters=[{
            'port': '/dev/ttyACM0',
            'baud': 9600,
            'frame_id': 'gps_link'
        }, {"use_sim_time": PythonExpression(["'", hardware_type, "' == 'simulated'"])}],
        namespace="gps",
        remappings=[("fix", "fix_no_cov")]
    )

    # Gazebo simulation
    gazebo_launch_include = IncludeLaunchDescription(PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
        launch_arguments={
            'gz_args': ['-v0 -r ', LaunchConfiguration("world"), '.sdf'], 
            "on_exit_shutdown": "true"
        }.items(),
        condition=IfCondition(PythonExpression(["'", hardware_type, "' == 'simulated' and '", show_sim, "' == 'true'"]))
    )
    gazebo_no_gui_launch_include = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
        launch_arguments={
            'gz_args': ['--headless-rendering -s -v0 -r ', LaunchConfiguration("world"), '.sdf'], 
            "on_exit_shutdown": "true"
        }.items(),
        condition=IfCondition(PythonExpression(["'", hardware_type, "' == 'simulated' and '", show_sim, "' == 'false'"]))
    )

    # Spawn imprimis into Gazebo simulation
    spawn_imprimis_gazebo = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=['-topic', 'robot_description', '-name', 'imprimis', '-z', '0.1', "--ros-args", "--log-level", "warn"],
        output="screen",
        condition=IfCondition(PythonExpression(["'", hardware_type, "' == 'simulated'"]))
    )

    # Bridge Gazebo topics and ROS topics
    gzbridge_config_file = PathJoinSubstitution([hardware_src_dir, 'config', 'gz_bridge.yaml'])
    gzbridge_no_gps_config_file = PathJoinSubstitution([hardware_src_dir, 'config', 'gz_bridge_no_gps.yaml'])
    gzbridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=['--ros-args', '-p', ['config_file:=', gzbridge_config_file], "--ros-args", "--log-level", "warn"],
        condition=IfCondition(PythonExpression(["'", hardware_type, "' == 'simulated' and '", sim_real_gps, "' == 'false'"])),
    )
    gzbridge_no_gps = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=['--ros-args', '-p', ['config_file:=', gzbridge_no_gps_config_file], "--ros-args", "--log-level", "warn"],
        condition=IfCondition(PythonExpression(["'", hardware_type, "' == 'simulated' and '", sim_real_gps, "' == 'true'"])),
    )

    # Add covariance to simulated GPS
    gps_covariance_fixer = Node(
        package="utils",
        executable="gps_add_sim_covariance",
        condition=IfCondition(PythonExpression(["'", hardware_type, "' != 'fake'"])),
        parameters=[{"use_sim_time": PythonExpression(["'", hardware_type, "' == 'simulated'"])}]
    )

    # Set gazebo resource path to include all sourced ros packages
    packages_paths = [os.path.join(p, 'share') for p in os.getenv('AMENT_PREFIX_PATH').split(':')]
    gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(hardware_src_dir_os, 'worlds') + ':',
            os.path.join(hardware_src_dir_os, 'meshes') + ':',
            os.path.join(description_src_dir_os, 'meshes') + ':',
            ':' + ':'.join(packages_paths)])
    
    # Do the same for old ignition variable
    old_sim_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(hardware_src_dir_os, 'worlds') + ':',
            os.path.join(hardware_src_dir_os, 'meshes') + ':',
            os.path.join(description_src_dir_os, 'meshes') + ':',
            ':' + ':'.join(packages_paths)])
    

    # Controller input
    controller_input_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('teleop_twist_joy'), 'launch', 'teleop-launch.py'])]),
        launch_arguments={
            'joy_config': 'xbox',
            "publish_stamped_twist": 'true',
            'frame': 'base_link',
            'joy_vel': 'diffbot_base_controller/cmd_vel'
        }.items(),
        condition=IfCondition(use_controller)
    )

    things_to_launch = [
        # Always
        robot_state_pub_node,
        robot_controller_spawner,
        joint_state_broadcaster_spawner,
        lidar_delay_fixer,
        rviz_node,

        # If hardware_type == real
        imu_driver,
        imu_calibrator,
        gps_driver,  # OR, if hardware_type == simulated and sim_real_gps == true
        velodyne_driver_node,
        velodyne_transform_node,

        # If hardware type != simulated
        controller_manager_node,
        gpio_controller_spawner,
        
        # If hardware_type == simulated
        gz_sim_resource_path,
        old_sim_resource_path,
        gazebo_launch_include,  # if show_sim == true
        gazebo_no_gui_launch_include, # if show_sim == false
        gzbridge,  # if sim_real_gps == false
        gzbridge_no_gps,  # if sim_real_gps == true
        spawn_imprimis_gazebo,

        # If use_controller == true
        controller_input_launch_include,

        # if hardware_type != fake
        gps_covariance_fixer,
    ]

    return LaunchDescription(declared_arguments + things_to_launch)


# Remap wheel odometry topic to mirror ekf output if publish_odom_tf is false, so higher level stuff doesn't have to care
    #odom_remapper = Node(
    #    package="utils",
    #    executable="odom_remapper",
    #    condition=IfCondition(publish_odom_tf),
    #    parameters=[{"use_sim_time": PythonExpression(["'", hardware_type, "' == 'simulated'"])}]
    #)