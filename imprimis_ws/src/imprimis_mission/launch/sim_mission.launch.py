import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """The simulator with everything on: navigation with reverse and recovery, the ramp detector, the
    lane mapper, the lap manager, and the control window.

    world        simulation world (igvc2027 has a course file, so laps are timed and remembered)
    nav2_params  Course2027 for the igvc2027 world, FullFootprint for the others
    gui          open the control window
    autostart    start an automatic lap as soon as navigation is up (used for unattended test laps)
    memory_dir   folder for the lap records and the remembered route
    """
    declared_arguments = [
        DeclareLaunchArgument("world", default_value="igvc2027"),
        DeclareLaunchArgument("nav2_params", default_value="Course2027"),
        DeclareLaunchArgument("show_sim", default_value="true"),
        DeclareLaunchArgument("ui_type", default_value="rviz", choices=("foxglove", "rviz", "none")),
        DeclareLaunchArgument("gui", default_value="true", choices=("true", "false")),
        DeclareLaunchArgument("autostart", default_value="false", choices=("true", "false")),
        DeclareLaunchArgument("exit_when_done", default_value="false", choices=("true", "false")),
        DeclareLaunchArgument("memory_dir", default_value="~/imprimis_lap_memory"),
        DeclareLaunchArgument("blend_file", default_value="",
                              description="The Blender map of the course. While the simulator runs, saving it in Blender updates the course."),
        DeclareLaunchArgument("repo_src", default_value="",
                              description="The Gethub/imprimis folder on the Windows side, where the course files are written."),
        DeclareLaunchArgument("workspace_src", default_value="", description="The copy of that folder inside Ubuntu."),
        DeclareLaunchArgument("top_speed", default_value="2.1", description="Speed on clear stretches, m/s. 2.1 is 4.7 mph; the IGVC limit of 5 mph is 2.235."),
    ]
    world = LaunchConfiguration("world")

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare("imprimis_navigation"), "launch", "basic_nav.launch.py"])]),
        launch_arguments={
            "hardware_type": "simulated",
            "world": world,
            "nav2_params": LaunchConfiguration("nav2_params"),
            "show_sim": LaunchConfiguration("show_sim"),
            "ui_type": LaunchConfiguration("ui_type"),
            "use_cams": "true",
            "use_recoveries": "true",
        }.items(),
    )

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare("imprimis_perception"), "launch", "course_perception.launch.py"])]),
        launch_arguments={"imu_yaw_in_base": "3.14159"}.items(),
    )

    # Course files are read from the source folder, like the other configuration in this repository.
    # These nodes are deliberately NOT given use_sim_time. They take the time from the stamps on the
    # sensor messages, which spares each of them several hundred clock messages a second.
    def mission_nodes(context):
        courses = os.path.join(get_package_share_directory("imprimis_mission"), "../../../../src/imprimis_mission/courses")
        course_file = os.path.join(courses, world.perform(context) + ".json")
        if not os.path.isfile(course_file):
            course_file = ""
        return [
            Node(
                package="imprimis_mission",
                executable="lap_manager",
                output="screen",
                parameters=[{
                    "course_file": course_file,
                    "memory_dir": LaunchConfiguration("memory_dir"),
                    "autostart": ParameterValue(LaunchConfiguration("autostart"), value_type=bool),
                    "exit_when_done": ParameterValue(LaunchConfiguration("exit_when_done"), value_type=bool),
                }],
            ),
            Node(
                package="imprimis_mission",
                executable="course_watcher",
                output="screen",
                parameters=[{
                    "blend_file": LaunchConfiguration("blend_file"),
                    "repo_src": LaunchConfiguration("repo_src"),
                    "workspace_src": LaunchConfiguration("workspace_src"),
                }],
            ),
            Node(
                package="imprimis_mission",
                executable="speed_governor",
                output="screen",
                parameters=[{"top_speed": ParameterValue(LaunchConfiguration("top_speed"), value_type=float)}],
            ),
            Node(
                package="imprimis_mission",
                executable="control_gui",
                output="screen",
                parameters=[{"course_file": course_file}],
                condition=IfCondition(LaunchConfiguration("gui")),
            ),
        ]

    # Gazebo's own record of where every moving body is. The lap manager uses the robot's entry to
    # count barrel contacts and lane line touches; the navigation software never sees it.
    # The topic name holds the world's name, which is "default" in igvc2027.sdf and igvc2.sdf.
    truth_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge_truth",
        arguments=["/world/default/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V", "--ros-args", "--log-level", "warn"],
        remappings=[("/world/default/dynamic_pose/info", "sim/true_poses")],
    )

    return LaunchDescription(declared_arguments + [navigation, perception, truth_bridge, OpaqueFunction(function=mission_nodes)])
