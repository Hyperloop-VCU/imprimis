from launch import LaunchDescription
from launch.substitutions import PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Declare and initialize arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "publish_odom_tf",
            default_value="true",
            description="Enable/disable the diff drive controller publising the odom->base_link transform. Should be false when using robot_localization.",
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
            description="Velodyne LIDAR RPM."
        )
    )
    gui = LaunchConfiguration("gui")
    hardware_type = LaunchConfiguration("hardware_type")
    use_controller = LaunchConfiguration("use_controller")
    publish_odom_tf = LaunchConfiguration("publish_odom_tf")
    lidar_rpm = LaunchConfiguration("lidar_rpm")


    # Get URDF via xacro and pass arguments to it
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("imprimis_description"), "urdf", "diffbot.urdf.xacro"]
            ),
            " ",
            "hardware_type:=",
            hardware_type,
            " publish_odom_tf:=",
            publish_odom_tf
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # controller manager
    controller_config_filename = "diffbot_controllers.yaml"
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("imprimis_hardware_platform"),
            "config",
            controller_config_filename,
        ]
    )
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers, {"enable_odom_tf": publish_odom_tf}],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        condition=IfCondition(PythonExpression(["'", hardware_type, "' != 'simulated'"]))
    )

    # robot state publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": PythonExpression(["'", hardware_type, "' == 'simulated'"])}],
    )

    # rviz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("imprimis_description"), "rviz", "diffbot.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": PythonExpression(["'", hardware_type, "' == 'simulated'"])}],
        condition=IfCondition(gui),
    )


    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        parameters=[{"use_sim_time": PythonExpression(["'", hardware_type, "' == 'simulated'"])}],
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        
    )

    # Diff drive controller spawner
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        parameters=[{"use_sim_time": PythonExpression(["'", hardware_type, "' == 'simulated'"])}],
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    )

    # GPIO controller spawner
    gpio_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        parameters=[{"use_sim_time": PythonExpression(["'", hardware_type, "' == 'simulated'"])}],
        arguments=["gpio_controller", "--controller-manager", "/controller_manager"],
        condition=IfCondition(PythonExpression(["'", hardware_type, "' != 'simulated'"]))
    )

    # Velodyne LIDAR driver, parser, and republisher
    lidar_config_file = PathJoinSubstitution(
        [FindPackageShare("imprimis_hardware_platform"), "config", "VLP16-velodyne_driver_node-params.yaml"]
    )
    velodyne_driver_node = Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        output='both',
        parameters=[lidar_config_file, {"rpm": lidar_rpm}],
        condition=IfCondition(PythonExpression(["'", hardware_type, "' != 'simulated'"]))
    )
    
    velodyne_converter_launch_include = IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('velodyne_pointcloud'),
                'launch',
                'velodyne_transform_node-VLP16-launch.py'
            ]),
            condition=IfCondition(PythonExpression(["'", hardware_type, "' == 'real'"])),
    ) 
    velodyne_republisher = Node(
        package="republisher",
        executable="lidar",
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

    # GPS driver and republisher
    gps_driver = Node(
        package="arduino_gps_driver",
        executable="arduino_gps_driver",
        condition=IfCondition(PythonExpression(["'", hardware_type, "' == 'real'"])),
    )
    gps_republisher = Node(
        package="republisher",
        executable="gps",
        condition=IfCondition(PythonExpression(["'", hardware_type, "' == 'simulated'"])),
        parameters=[{"use_sim_time": PythonExpression(["'", hardware_type, "' == 'simulated'"])}]
    )

    # Gazebo
    gazebo_launch_include = IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ]),
            launch_arguments={'gz_args': ['-r ', LaunchConfiguration("world"), '.sdf'], "on_exit_shutdown": "true"}.items(),
            condition=IfCondition(PythonExpression(["'", hardware_type, "' == 'simulated'"]))
        )

    # Spawn imprimis into the gazebo simulation
    spawn_imprimis_gazebo = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=['-topic', 'robot_description', '-name', 'imprimis', '-z', '0.1'],
        output="screen",
        condition=IfCondition(PythonExpression(["'", hardware_type, "' == 'simulated'"]))
    )

    # Bridge gazebo and ROS topics
    gzbridge_config_file = PathJoinSubstitution(
        [FindPackageShare("imprimis_hardware_platform"), "config", "gz_bridge.yaml"]
    )
    gzbridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=['--ros-args', '-p', ['config_file:=', gzbridge_config_file]],
        condition=IfCondition(PythonExpression(["'", hardware_type, "' == 'simulated'"]))
    )

    # Directories
    pkg_imprimis_hardware = get_package_share_directory('imprimis_hardware_platform')
    pkg_imprimis_description = get_package_share_directory('imprimis_description')

    # Determine all ros packages that are sourced
    packages_paths = [os.path.join(p, 'share') for p in os.getenv('AMENT_PREFIX_PATH').split(':')]

    # Set gazebo resource path to include all sourced ros packages
    gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(pkg_imprimis_hardware, 'worlds') + ':',
            os.path.join(pkg_imprimis_hardware, 'meshes') + ':',
            os.path.join(pkg_imprimis_description, 'meshes') + ':',
            ':' + ':'.join(packages_paths)])
    
    # Do the same for old ignition variable
    old_sim_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(pkg_imprimis_hardware, 'worlds') + ':',
            os.path.join(pkg_imprimis_hardware, 'meshes') + ':',
            os.path.join(pkg_imprimis_description, 'meshes') + ':',
            ':' + ':'.join(packages_paths)])
    

    # Controller input
    controller_input_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('teleop_twist_joy'),
                'launch',
                'teleop-launch.py'
            ])
        ]),
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
        velodyne_republisher,

        # If hardware_type == real
        imu_driver,
        imu_calibrator,
        #gps_driver,
        velodyne_driver_node,
        velodyne_converter_launch_include,

        # If hardware type != simulated
        controller_manager_node,
        gpio_controller_spawner,
        
        # If hardware_type == simulated
        gz_sim_resource_path,
        old_sim_resource_path,
        gazebo_launch_include,
        gzbridge,
        spawn_imprimis_gazebo,
        gps_republisher,

        # If use_controller == true
        controller_input_launch_include,

        # If gui == true
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + things_to_launch)