from launch import LaunchDescription
from launch.substitutions import PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.actions import SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable
from launch_ros.substitutions import FindPackagePrefix

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
            "world_file",
            default_value= PathJoinSubstitution([FindPackageShare("imprimis_hardware_platform"), "worlds", "empty.world"]),
            description="World file for gazebo simulation"
        )
    )
    gui = LaunchConfiguration("gui")
    hardware_type = LaunchConfiguration("hardware_type")
    use_controller = LaunchConfiguration("use_controller")
    publish_odom_tf = LaunchConfiguration("publish_odom_tf")
    world_file = LaunchConfiguration("world_file")


    # Get URDF via xacro
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


    # Velodyne LIDAR driver and parser
    velodyne_driver_launch_include = IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('velodyne_driver'),
                'launch',
                'velodyne_driver_node-VLP16-launch.py'
            ]),
            condition=IfCondition(PythonExpression(["'", hardware_type, "' == 'real'"]))
        ) 
    velodyne_converter_launch_include = IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('velodyne_pointcloud'),
                'launch',
                'velodyne_transform_node-VLP16-launch.py'
            ]),
            condition=IfCondition(PythonExpression(["'", hardware_type, "' == 'real'"]))
        ) 


    # Gazebo
    gazebo_launch_include = IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ]),
            launch_arguments={'gz_args': ['-r ', world_file], "on_exit_shutdown": "true"}.items(),
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

        # If hardware_type == real
        #velodyne_driver_launch_include,
        #velodyne_converter_launch_include

        # If hardware type != simulated
        controller_manager_node,
        gpio_controller_spawner,
        
        # If hardware_type == simulated
        gazebo_launch_include,
        gzbridge,
        spawn_imprimis_gazebo,

        # If use_controller == true
        controller_input_launch_include,

        # If gui == true
        rviz_node,

    ]

    return LaunchDescription(declared_arguments + things_to_launch)