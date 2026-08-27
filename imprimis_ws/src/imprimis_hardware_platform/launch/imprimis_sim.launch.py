from launch import LaunchDescription
from launch.substitutions import PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, SetEnvironmentVariable, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import tempfile
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
            description="If false, the simulation will run in headless mode (no GUI). If true, the gazebo GUI will run as usual."
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
            "use_cams",
            default_value="true",
            choices=("true", "false"),
            description="Whether or not to include cameras in the simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_lidar",
            default_value="true",
            choices=("true", "false"),
            description="Whether or not to include LiDAR in the simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_imu",
            default_value="true",
            choices=("true","false"),
            description="Whether or not to include IMU in the simulation."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_gps",
            default_value="true",
            choices=("true", "false"),
            description="Whether or not to include GPS in the simulation.",
        )
    )
    use_controller = LaunchConfiguration("use_controller")
    publish_odom_tf = LaunchConfiguration("publish_odom_tf")
    show_sim = LaunchConfiguration("show_sim")
    use_gps = LaunchConfiguration("use_gps")
    use_cams = LaunchConfiguration("use_cams")
    use_lidar = LaunchConfiguration("use_lidar")
    use_imu = LaunchConfiguration("use_imu")

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
        "simulated",
        " publish_odom_tf:=",
        publish_odom_tf
    ])
    robot_description = {"robot_description": robot_description_content}

    # robot state publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": True}],
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
        parameters=[{"use_sim_time": True}],
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        parameters=[{"use_sim_time": True}],
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", "--ros-args", "--log-level", "warn"],
    )

    # Diff drive controller spawner
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        parameters=[{"use_sim_time": True}],
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager", "--ros-args", "--log-level", "warn"],
    )
    lidar_delay_fixer = Node(
        package="utils",
        executable="fix_lidar_delay",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_lidar)
    )

    # Gazebo simulation
    gazebo_launch_include = IncludeLaunchDescription(PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
        launch_arguments={
            'gz_args': ['-v0 -r ', LaunchConfiguration("world"), '.sdf'], 
            "on_exit_shutdown": "true"
        }.items(),
        condition=IfCondition(show_sim)
    )
    gazebo_no_gui_launch_include = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
        launch_arguments={
            'gz_args': ['--headless-rendering -s -v0 -r ', LaunchConfiguration("world"), '.sdf'], 
            "on_exit_shutdown": "true"
        }.items(),
        condition=UnlessCondition(show_sim)
    )

    # Spawn imprimis into Gazebo simulation
    spawn_imprimis_gazebo = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=['-topic', 'robot_description', '-name', 'imprimis', '-z', '0.1', "--ros-args", "--log-level", "warn"],
        output="screen"
    )

    # Bridge Gazebo topics and ROS topics.
    # gz_bridge.yaml is the single source of truth. Entries tagged with a "sensor:" key
    # are dropped when the matching use_<sensor> argument is false, and the filtered list
    # is written to a temporary config file that is handed to the bridge. The gz topics
    # still exist either way; this only stops them being bridged into ROS.
    def make_gzbridge(context):
        sensor_enabled = {
            "lidar": LaunchConfiguration("use_lidar").perform(context) == "true",
            "cams": LaunchConfiguration("use_cams").perform(context) == "true",
            "imu": LaunchConfiguration("use_imu").perform(context) == "true",
            "gps": LaunchConfiguration("use_gps").perform(context) == "true",
        }

        source_config = os.path.join(hardware_src_dir_os, 'config', 'gz_bridge.yaml')
        with open(source_config) as f:
            all_entries = yaml.safe_load(f) or []

        entries = []
        for entry in all_entries:
            entry = dict(entry)
            sensor = entry.pop("sensor", None)  # strip, ros_gz_bridge doesn't know this key
            if sensor is None:
                entries.append(entry)
            elif sensor not in sensor_enabled:
                raise RuntimeError(
                    f"{source_config}: unknown sensor '{sensor}' on topic "
                    f"'{entry.get('ros_topic_name')}'. Expected one of {sorted(sensor_enabled)}."
                )
            elif sensor_enabled[sensor]:
                entries.append(entry)

        fd, filtered_config = tempfile.mkstemp(prefix='imprimis_gz_bridge_', suffix='.yaml')
        with os.fdopen(fd, 'w') as f:
            yaml.safe_dump(entries, f, sort_keys=False)

        def remove_filtered_config(*_args, **_kwargs):
            try:
                os.remove(filtered_config)
            except OSError:
                pass

        return [
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=['--ros-args', '-p', ['config_file:=', filtered_config], "--ros-args", "--log-level", "warn"]
            ),
            RegisterEventHandler(OnShutdown(on_shutdown=remove_filtered_config)),
        ]

    gzbridge = OpaqueFunction(function=make_gzbridge)

    # Add covariance to simulated GPS
    gps_covariance_fixer = Node(
        package="utils",
        executable="gps_add_sim_covariance",
        condition=IfCondition(use_gps),
        parameters=[{"use_sim_time": True}]
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
        launch_arguments={'port': '9091'}.items()
    )

    foxglove_launch_include = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml'])]),
    )

    things_to_launch = [
        # Always
        robot_state_pub_node,
        robot_controller_spawner,
        joint_state_broadcaster_spawner,
        lidar_delay_fixer,
        #rviz_node,
        #rosbridge_launch_include,
        foxglove_launch_include,
        gz_sim_resource_path,
        old_sim_resource_path,
        gazebo_launch_include,  # if show_sim == true
        gazebo_no_gui_launch_include, # if show_sim == false
        gzbridge,
        spawn_imprimis_gazebo,
        gps_covariance_fixer,

        # If use_controller == true
        controller_input_launch_include
    ]

    return LaunchDescription(declared_arguments + things_to_launch)