from launch import LaunchDescription
from launch.substitutions import PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, SetEnvironmentVariable, GroupAction, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
import yaml

def launch_setup(context, *args, **kwargs):
    headless = LaunchConfiguration('headless').perform(context)
    do_fake_localization = LaunchConfiguration('do_fake_localization').perform(context)
    publish_tf_odom2baselink = LaunchConfiguration('publish_tf_odom2baselink').perform(context)
    publish_log_topics = LaunchConfiguration('publish_log_topics').perform(context)
    force_publish_vehicle_namespace = LaunchConfiguration('force_publish_vehicle_namespace').perform(context)
    use_rviz = LaunchConfiguration('use_rviz').perform(context)
    world_file = LaunchConfiguration('world_file').perform(context)
    rviz_file = world_file.removesuffix(".world.xml") + ".rviz"

    mvsim_dir = get_package_share_directory('mvsim')
    hardware_dir = get_package_share_directory('imprimis_hardware_platform')
    description_src_dir_os = os.path.join(hardware_dir, '../imprimis_description')
    
    try:
        rviz_config_file = os.path.join(mvsim_dir, 'mvsim_tutorial', rviz_file)
        world_path = os.path.join(mvsim_dir, 'mvsim_tutorial', world_file)
        open(world_path) # confirm file exists
    except Exception:
        rviz_config_file = os.path.join(description_src_dir_os, "rviz", "diffbot.rviz")
        world_path = os.path.join(hardware_dir, 'sim', world_file)
    
    launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mvsim'),
                'launch',
                'launch_world.launch.py'
            )
        ),
        launch_arguments={
            'world_file': world_path,
            'headless': headless,
            'do_fake_localization': do_fake_localization,
            'publish_tf_odom2baselink': publish_tf_odom2baselink,
            'force_publish_vehicle_namespace': force_publish_vehicle_namespace,
            'publish_log_topics': publish_log_topics,
            'use_rviz': use_rviz,
            'rviz_config_file': rviz_config_file
        }.items()
    )
    
    # 3. RETURN the entities as a list instead of calling ld.add_action()
    return [launch]

def generate_launch_description():
    # Get mvsim package directory
    mvsim_dir = get_package_share_directory('mvsim')

    # rviz config
    rviz_config_file = os.path.join(
        mvsim_dir, 'mvsim_tutorial', 'demo_1robot_ros2.rviz'
    )

    # Create launch description
    ld = LaunchDescription()

    # Declare launch arguments (defaults defined here)
    ld.add_action(
        DeclareLaunchArgument(
            'headless',
            default_value='False',
            description='Run in headless mode'
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'do_fake_localization',
            default_value='True',
            description='Publish fake identity tf "map" -> "odom"'
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'publish_tf_odom2baselink',
            default_value='True',
            description='Publish tf "odom" -> "base_link"'
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'force_publish_vehicle_namespace',
            default_value='True',
            description='Use vehicle name namespace even if there is only one vehicle'
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='True',
            description='Whether to launch RViz2'
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'publish_log_topics',
            default_value='False',
            description='Publish every CSV-logger column as a std_msgs/Float64 topic per vehicle. '
                        'High-rate, disabled by default.'
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            'world_file',
            default_value="demo_1robot.world.xml"
        )
    )

    of = OpaqueFunction(function=launch_setup)
    ld.add_entity(of)

    return ld






