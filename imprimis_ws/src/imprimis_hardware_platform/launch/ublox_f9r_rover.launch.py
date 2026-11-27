"""ZED-F9R rover bringup for ublox_dgnss (kilted).

Starts:
  * ublox_dgnss core driver (DEVICE_FAMILY F9R)
  * ublox_nav_sat_fix_hp translator, NavSatFix remapped fix -> /gps/fix
  * ntrip_client (ros-kilted-ntrip-client, only when use_rtk:=true):
      /gps/fix -> GGA uplink to the caster (AUTO mountpoint picks a base from it)
      caster RTCM -> /ntrip_client/rtcm (rtcm_msgs/Message) -> ublox_dgnss -> receiver
  * rosbridge_server websocket (only when start_rosbridge:=true) for the GPS panel

Receiver configuration policy: the driver only writes CFG keys that are given
here as parameters (PARAM_USER); every other key is read back from the device
(CFG-VALGET) and left alone. So the ONLY CFG_* values below are the UBX
message-output rates the GPS panel and the NavSatFix translator need. Nav rate,
sensor-fusion (SFCORE/SFIMU/SFODO), dynamic model, signals, protocol and NMEA
settings are all untouched. The driver writes to the RAM layer only, so these
rates reset on a receiver power cycle and are re-sent on every launch/hotplug.

Usage:
  ros2 launch ublox_f9r_rover.launch.py                 # with RTK corrections
  ros2 launch ublox_f9r_rover.launch.py use_rtk:=false  # standalone GNSS
  ros2 launch ublox_f9r_rover.launch.py start_rosbridge:=true  # + websocket on 9091
"""
import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode

# ---------------------------------------------------------------- NTRIP caster
# TODO: move these out of the launch file before committing.
NTRIP_HOST = 'truertk.pointonenav.com'
NTRIP_PORT = 2101
NTRIP_MOUNTPOINT = 'AUTO'
NTRIP_USERNAME = 'frvrgfugpj'
NTRIP_PASSWORD = 'q5j9bkf8g6'
NTRIP_SSL = False              # 2101 is plain NTRIP; Point One's TLS port is 2102

# ---------------------------------------------------------------- topics
FIX_TOPIC = '/gps/fix'                 # NavSatFix from the translator; also the GGA source
RTCM_TOPIC = '/ntrip_client/rtcm'      # hard-coded subscription in ublox_dgnss (RTCM_INPUT_ENABLED)

# ---------------------------------------------------------------- UBX output
# Rate is "output once every N navigation epochs" on the USB port; 1 = every
# epoch, so each message follows whatever nav rate is already on the receiver.
UBX_MSGOUT = {
    'CFG_MSGOUT_UBX_NAV_PVT_USB': 1,       # fix type, carrier soln, sats, accuracy, vehicle heading
    'CFG_MSGOUT_UBX_NAV_STATUS_USB': 1,    # fix flags, diff corr, TTFF, spoofing   (also feeds /gps/fix status)
    'CFG_MSGOUT_UBX_NAV_DOP_USB': 1,       # HDOP / VDOP / PDOP ...
    'CFG_MSGOUT_UBX_NAV_COV_USB': 1,       # NED covariance                       (feeds /gps/fix covariance)
    'CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB': 1,  # high-precision lat/lon               (feeds /gps/fix position)
    'CFG_MSGOUT_UBX_NAV_SAT_USB': 1,       # per-satellite signal / usage / corrections
    'CFG_MSGOUT_UBX_ESF_STATUS_USB': 1,    # sensor-fusion state, IMU / wheel-tick sensor health
    # ESF-MEAS counts sensor samples, not nav epochs: at 1 it streams raw IMU
    # data at ~100 Hz and starves the NAV messages (/gps/fix fell to ~0.36 Hz
    # with 7 s gaps). 50 gives ~2 Hz of mixed gyro/accel/wheel-tick samples.
    'CFG_MSGOUT_UBX_ESF_MEAS_USB': 50,     # wheel-tick counts & direction (confirms the WT wiring)
    'CFG_MSGOUT_UBX_RXM_RTCM_USB': 1,      # each RTCM message the receiver accepts (corrections flowing)
}


def generate_launch_description():
  """Generate launch description for the ZED-F9R rover."""

  use_rtk = LaunchConfiguration('use_rtk')
  frame_id = LaunchConfiguration('frame_id')
  device_serial_string = LaunchConfiguration('device_serial_string')
  log_level = LaunchConfiguration('log_level')

  use_rtk_arg = DeclareLaunchArgument(
    'use_rtk', default_value=TextSubstitution(text='true'),
    description='Start the NTRIP client and feed RTCM corrections to the receiver'
  )
  frame_id_arg = DeclareLaunchArgument(
    'frame_id', default_value=TextSubstitution(text='gps_link'),
    description='frame_id in the header of published messages'
  )
  device_serial_string_arg = DeclareLaunchArgument(
    'device_serial_string', default_value=TextSubstitution(text=''),
    description='USB serial string of the receiver (empty = first u-blox found)'
  )
  log_level_arg = DeclareLaunchArgument(
    'log_level', default_value=TextSubstitution(text='INFO')
  )
  # Off by default: the robot's hardware/sim bringup already runs rosbridge.
  start_rosbridge_arg = DeclareLaunchArgument(
    'start_rosbridge', default_value=TextSubstitution(text='false'),
    description='Start rosbridge_server (websocket) for the GPS panel'
  )
  rosbridge_port_arg = DeclareLaunchArgument(
    'rosbridge_port', default_value=TextSubstitution(text='9091'),
    description='rosbridge websocket port (the GPS panel defaults to 9091)'
  )

  driver_params = [
    {'DEVICE_FAMILY': 'F9R'},
    {'DEVICE_SERIAL_STRING': device_serial_string},
    {'FRAME_ID': frame_id},
    {'CFG_USBOUTPROT_NMEA': False},
    UBX_MSGOUT,
  ]

  ntrip_params = [{
    'host': NTRIP_HOST,
    'port': NTRIP_PORT,
    'mountpoint': NTRIP_MOUNTPOINT,
    'authenticate': True,              # node default is False: credentials would be ignored
    'username': NTRIP_USERNAME,
    'password': NTRIP_PASSWORD,
    'ssl': NTRIP_SSL,
    # ublox_dgnss subscribes with rtcm_msgs/msg/Message; the node defaults to mavros_msgs
    'rtcm_message_package': 'rtcm_msgs',
    'rtcm_frame_id': frame_id,
    # The node reconnects when no RTCM arrives for rtcm_timeout_seconds and
    # raises once reconnect_attempt_max attempts in a row fail. A robot
    # driving through dead spots should keep trying, not exit after ~50 s.
    'rtcm_timeout_seconds': 4,
    'reconnect_attempt_max': 1000000,
    'reconnect_attempt_wait_seconds': 5,
  }]

  # Driver and translator share a container so /ubx_nav_* hops stay in-process.
  gnss_container = ComposableNodeContainer(
    name='ublox_f9r_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container_mt',
    arguments=['--ros-args', '--log-level', log_level],
    composable_node_descriptions=[
      ComposableNode(
        package='ublox_dgnss_node',
        plugin='ublox_dgnss::UbloxDGNSSNode',
        name='ublox_dgnss',
        parameters=driver_params,
      ),
      ComposableNode(
        package='ublox_nav_sat_fix_hp_node',
        plugin='ublox_nav_sat_fix_hp::UbloxNavSatHpFixNode',
        name='ublox_nav_sat_fix_hp',
        remappings=[('fix', FIX_TOPIC)],
        # The translator publishes with SensorDataQoS (best effort), but
        # ntrip_client subscribes reliable and has no override hook, so it would
        # get no fixes and send no GGA. A reliable publisher still serves
        # best-effort subscribers (robot_localization, rosbridge). The key uses
        # the topic name after remapping.
        parameters=[{'qos_overrides.' + FIX_TOPIC + '.publisher.reliability': 'reliable'}],
      ),
    ]
  )

  # Topics in this node are relative ('fix', 'nmea', 'rtcm'); left alone, its
  # RTCM would land on /rtcm, which ublox_dgnss itself publishes (base mode).
  # 'nmea' stays unremapped: nothing publishes it, GGA comes from the fix.
  ntrip_client = Node(
    condition=IfCondition(use_rtk),
    package='ntrip_client',
    executable='ntrip_ros.py',
    name='ntrip_client',
    namespace='',
    parameters=ntrip_params,
    remappings=[
      ('fix', FIX_TOPIC),
      ('rtcm', RTCM_TOPIC),
    ],
    ros_arguments=['--log-level', log_level],
    # it also exits if the very first connect fails (e.g. no network at boot)
    respawn=True,
    respawn_delay=5.0,
    output='screen',
  )

  rosbridge = IncludeLaunchDescription(
    AnyLaunchDescriptionSource(PathJoinSubstitution([
      FindPackageShare('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml'])),
    launch_arguments={'port': LaunchConfiguration('rosbridge_port')}.items(),
    condition=IfCondition(LaunchConfiguration('start_rosbridge')),
  )

  return launch.LaunchDescription([
    use_rtk_arg,
    frame_id_arg,
    device_serial_string_arg,
    log_level_arg,
    start_rosbridge_arg,
    rosbridge_port_arg,
    gnss_container,
    ntrip_client,
    rosbridge,
  ])
