from launch import LaunchDescription
from launch import LaunchContext
from launch.substitutions import PythonExpression
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, LogInfo, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from os import path as os_path
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
            "hardware_type",
            default_value="real",
            choices=("real", "fake", "simulated"),
            description="Choose between real hardware, completely faked hardware, or gazebo-simulated hardware.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "control_type",
            default_value="other",
            choices=("keyboard", "controller", "other"),
            description="How imprimis will actually be controlled. Choice between keyboard, joystick controller, or other. Keyboard input is not implemented yet.",
        )
    )
    gui = LaunchConfiguration("gui")
    hardware_type = LaunchConfiguration("hardware_type")
    control_type = LaunchConfiguration("control_type")


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


    # controller manager, if we are not using simulated hardware
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("imprimis_hardware_platform"),
            "config",
            "diffbot_controllers.yaml",
        ]
    )
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
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
        parameters=[robot_description],
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
        condition=IfCondition(gui),
    )


    # joint state broadcaster spawner, if we aren't using simulated hardware
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        condition=IfCondition(PythonExpression(["'", hardware_type, "' != 'simulated'"]))
    )


    # Imprimis diff drive controller spawner
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    )

    # Actual velodyne LIDAR driver, if we are using real hardware
    velodyne_driver_launch_include = IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('velodyne_driver'),
                'launch',
                'velodyne_driver_node-VLP16-launch.py'
            ]),
            condition=IfCondition(PythonExpression(["'", hardware_type, "' == 'real'"]))
        ) 
    
    # Convert LIDAR packets to usable data, if we are using real hardware
    velodyne_converter_launch_include = IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('velodyne_pointcloud'),
                'launch',
                'velodyne_transform_node-VLP16-launch.py'
            ]),
            condition=IfCondition(PythonExpression(["'", hardware_type, "' == 'real'"]))
        ) 


    # Gazebo, if we're using simulated hardware
    gazebo_launch_include = IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ]),
            condition=IfCondition(PythonExpression(["'", hardware_type, "' == 'simulated'"]))
        )
    

    # Spawn imprimis into the gazebo simulation, if we're using simulated hardware
    spawn_imprimis_gazebo = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=['-topic', 'robot_description', '-entity', 'imprimis'],
        output="screen",
        condition=IfCondition(PythonExpression(["'", hardware_type, "' == 'simulated'"]))
    )
    

    # Controller input, if we specified to use it
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
        condition=IfCondition(PythonExpression(["'", control_type, "' == 'controller'"]))
        )
    
    

    things_to_launch = [
        controller_manager_node,         # not in sim
        robot_state_pub_node,
        robot_controller_spawner,
        rviz_node,
        joint_state_broadcaster_spawner, # not in sim
        gazebo_launch_include,           # sim only
        spawn_imprimis_gazebo,           # sim only
        controller_input_launch_include, # if we need it
        velodyne_driver_launch_include,   # real hardware only
        velodyne_converter_launch_include # real hardware only

    ]

    return LaunchDescription(declared_arguments + things_to_launch)



"""
Old code for delaying node startup
# Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )
    # Delay start of joint_state_broadcaster after `robot_controller`
    # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
"""