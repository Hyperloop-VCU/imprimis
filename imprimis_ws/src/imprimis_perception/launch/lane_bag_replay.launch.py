"""Replay a lane bag through the current lane_projection and the Nav2 local costmap, and show it in RViz.

The bag supplies the lane masks, camera_info, camera images and tf_static. Lane points and the costmap are
rebuilt live, so the bag's recorded lane points and costmap are not played.
Example: ros2 launch imprimis_perception lane_bag_replay.launch.py bag:=spot1_clean denoise_group_size:=3
"""
import os
import re
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SHARE = Path(get_package_share_directory('imprimis_perception'))
DEFAULT_BAG_DIR = SHARE.parents[3] / 'src' / 'imprimis_perception' / 'lane_iterations' / '1' / 'RealBags'
MASK_TOPIC = re.compile(r'^/perception/(.+)/lane_mask/compressed$')


def resolve_bag(context):
    bag = os.path.expanduser(LaunchConfiguration('bag').perform(context))
    path = Path(bag)
    if not path.exists():
        path = Path(os.path.expanduser(LaunchConfiguration('bag_dir').perform(context))) / bag
    if not (path / 'metadata.yaml').exists():
        raise RuntimeError(f'Bag not found: {path}')
    return path


def setup(context):
    path = resolve_bag(context)
    info = yaml.safe_load(open(path / 'metadata.yaml'))['rosbag2_bagfile_information']
    topics = [t['topic_metadata']['name'] for t in info['topics_with_message_count']]
    cameras = sorted(m.group(1) for m in map(MASK_TOPIC.match, topics) if m)
    if not cameras:
        raise RuntimeError(f'No /perception/<camera>/lane_mask/compressed topic in {path}')
    play = [t for t in topics if t == '/tf_static' or MASK_TOPIC.match(t)
            or any(t in (f'/cameras/{c}/color/camera_info', f'/cameras/{c}/color/image_raw') for c in cameras)]

    cmd = ['ros2', 'bag', 'play', str(path), '--clock', '--rate', LaunchConfiguration('rate').perform(context),
           '--topics', *play]
    if LaunchConfiguration('loop').perform(context) == 'true':
        cmd.append('--loop')

    return [
        ExecuteProcess(cmd=cmd, output='screen'),
        Node(
            package='imprimis_perception',
            executable='lane_projection',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'camera_names': cameras,
                'base_frame': 'base_link',
                'output_height': 0.0,
                'detect_potholes': True,
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(SHARE / 'launch' / 'lane_costmap.launch.py')),
            launch_arguments={
                'nav2_params': LaunchConfiguration('nav2_params'),
                'denoise_group_size': LaunchConfiguration('denoise_group_size'),
                'use_sim_time': 'true',
                'start_delay': '3.0',
            }.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag', description='Bag folder name in bag_dir, or a path to a bag'),
        DeclareLaunchArgument('bag_dir', default_value=str(DEFAULT_BAG_DIR)),
        DeclareLaunchArgument('nav2_params', default_value='Lane2027',
                              description='Nav2 params file in imprimis_navigation/config/nav2; its local_costmap is used'),
        DeclareLaunchArgument('denoise_group_size', default_value='0',
                              description='Add a DenoiseLayer that removes lethal groups smaller than this; 0 for none'),
        DeclareLaunchArgument('loop', default_value='true', choices=('true', 'false')),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('rviz', default_value='true', choices=('true', 'false')),
        OpaqueFunction(function=setup),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', str(SHARE / 'rviz' / 'lane_bags.rviz')],
            parameters=[{'use_sim_time': True}],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
