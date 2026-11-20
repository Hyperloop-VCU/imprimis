"""Record a lane bag (mcap) for a set time into lane_iterations/<iteration>/RealBags/<bag>.
Example: ros2 launch imprimis_perception lane_bag_record.launch.py bag:=spot4_clean"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration

ITERATIONS_DIR = Path(get_package_share_directory('imprimis_perception')).parents[3] / 'src' / 'imprimis_perception' / 'lane_iterations'


def setup(context):
    bag = LaunchConfiguration('bag').perform(context)
    duration = float(LaunchConfiguration('duration').perform(context))
    cam = LaunchConfiguration('camera_name').perform(context)
    path = ITERATIONS_DIR / LaunchConfiguration('iteration').perform(context) / 'RealBags' / bag
    if path.exists():
        raise RuntimeError(f'Bag already exists, pick another name: {path}')
    path.parent.mkdir(parents=True, exist_ok=True)

    topics = [
        f'/cameras/{cam}/color/image_raw', f'/cameras/{cam}/color/camera_info', f'/cameras/{cam}/color/metadata',
        f'/cameras/{cam}/accel/sample', f'/cameras/{cam}/gyro/sample',
        f'/perception/{cam}/lane_mask/compressed', '/perception/lane_points', '/perception/pothole_points',
        LaunchConfiguration('costmap_topic').perform(context), '/tf', '/tf_static', '/rosout',
    ]
    # the recorder needs about 3 s to find every topic before data starts arriving
    cmd = ['timeout', '--preserve-status', '-s', 'INT', f'{duration + 3:g}',
           'ros2', 'bag', 'record', '-s', 'mcap', '-o', str(path), *topics]
    recorder = ExecuteProcess(cmd=cmd, name='lane_bag_record', output='screen')
    done = RegisterEventHandler(OnProcessExit(
        target_action=recorder,
        on_exit=[LogInfo(msg=f'Finished recording {path}')],
    ))
    return [TimerAction(period=float(LaunchConfiguration('delay').perform(context)), actions=[
        LogInfo(msg=f'Recording {duration:g} s to {path}'), recorder, done,
    ])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('bag', description='Name of the bag folder to create'),
        DeclareLaunchArgument('duration', default_value='30', description='Seconds to record'),
        DeclareLaunchArgument('iteration', default_value='1', description='lane_iterations/<iteration>/RealBags'),
        DeclareLaunchArgument('camera_name', default_value='d435i'),
        DeclareLaunchArgument('costmap_topic', default_value='/local_costmap/costmap'),
        DeclareLaunchArgument('delay', default_value='5.0',
                              description='Seconds to wait before recording, so the camera and costmap are up'),
        OpaqueFunction(function=setup),
    ])
