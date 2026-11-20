"""Standalone Nav2 local costmap, built from the local_costmap section of a Nav2 params file, and started by a
lifecycle manager. Publishes /local_costmap/costmap, the same topic as the full nav stack."""
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SHARE = Path(get_package_share_directory('imprimis_perception'))
NAV_CONFIG_DIR = SHARE.parents[3] / 'src' / 'imprimis_navigation' / 'config' / 'nav2'


def params_file(context, use_sim_time):
    name = LaunchConfiguration('nav2_params').perform(context)
    params = yaml.safe_load(open(NAV_CONFIG_DIR / f'{name}.yaml'))['local_costmap']['local_costmap']
    p = params['ros__parameters']
    p['use_sim_time'] = use_sim_time
    size = int(LaunchConfiguration('denoise_group_size').perform(context))
    if size > 1:
        plugins = list(p['plugins'])
        at = plugins.index('inflation_layer') if 'inflation_layer' in plugins else len(plugins)
        p['plugins'] = plugins[:at] + ['denoise_layer'] + plugins[at:]
        p['denoise_layer'] = {'plugin': 'nav2_costmap_2d::DenoiseLayer', 'enabled': True,
                              'minimal_group_size': size, 'group_connectivity_type': 8}
    out = tempfile.NamedTemporaryFile('w', prefix='lane_costmap_', suffix='.yaml', delete=False)
    yaml.safe_dump({'local_costmap': {'local_costmap': params}}, out)
    out.close()
    return out.name


def setup(context):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    return [
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='local_costmap',
            namespace='local_costmap',
            output='screen',
            parameters=[params_file(context, use_sim_time)],
        ),
        TimerAction(period=float(LaunchConfiguration('start_delay').perform(context)), actions=[Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_lane_costmap',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'autostart': True, 'bond_timeout': 0.0,
                         'node_names': ['local_costmap/local_costmap']}],
        )]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('nav2_params', default_value='Lane2027',
                              description='Nav2 params file in imprimis_navigation/config/nav2; its local_costmap is used'),
        DeclareLaunchArgument('denoise_group_size', default_value='0',
                              description='Add a DenoiseLayer that removes lethal groups smaller than this; 0 for none'),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=('true', 'false')),
        DeclareLaunchArgument('start_delay', default_value='3.0',
                              description='Seconds to wait before starting the costmap, so TF is up'),
        OpaqueFunction(function=setup),
    ])
