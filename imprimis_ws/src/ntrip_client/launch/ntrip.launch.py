from launch import LaunchDescription
from launch.substitutions import PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, SetEnvironmentVariable, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    #Note the simulation may be deprecated.
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Launch the simulated GPS publisher as well as the NTRIP client.',
    )

    #Launches a map showing live position of fixed GPS GGA data.
    show_map_arg = DeclareLaunchArgument(
        'show_map',
        default_value='false',
        choices=['true', 'false'],
        description='Shows a map of current GPS GGA (asher UI)',
    )
    start_rosbridge_arg = DeclareLaunchArgument(
        'start_rosbridge',
        default_value='false',
        choices=['true', 'false'],
        description='Starts an instance of the rosbridge node. Useful if you want to visualize with the GPS goal map (ray UI)',
    )
    use_rtk_arg = DeclareLaunchArgument(
        'use_rtk',
        default_value='true',
        choices=['true', 'false'],
        description='If false, just starts the nmea navsat driver and nothing else.',
    )

    return LaunchDescription([
        sim_arg,
        show_map_arg,
        start_rosbridge_arg,
        use_rtk_arg,

        #Note the map only works with valid GGA sentences.
        Node(
            package='ntrip_client',
            executable='ntrip_node',
            name='ntrip_client',
            output='screen',
            parameters=[
                {
                    'show_map': LaunchConfiguration('show_map'),
                    'sim': ParameterValue(
                        LaunchConfiguration('sim'),
                        value_type=bool,
                    ),
                    'debug': False,
                    'caster': "rtk.geodnet.com" ,
                    'port': 2101,
                    'mountpoint': "AUTO" ,
                    'baud_rate': 38400,
                    'gps_port': "/dev/ttyACM0",
                }
            ],
            condition=IfCondition(LaunchConfiguration('use_rtk'))
        ),
        Node(
            package='ntrip_client',
            executable='simulated_gps_node',
            name='simulated_gps',
            output='screen',
            condition=IfCondition(LaunchConfiguration('sim')),
        ),
        Node(
            package='nmea_navsat_driver',
            executable='nmea_topic_driver',
            name='nmea_topic_driver',
            output='screen',
            remappings=[
                ('fix', 'gps/fix')
            ],
            condition=IfCondition(LaunchConfiguration('use_rtk'))
        ),
        Node(
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            name='nmea_serial_driver',
            output='screen',
            remappings=[
                ('fix', 'gps/fix')
            ],
            parameters=[{
                'port': '/dev/ttyACM0'
            }],
            condition=UnlessCondition(LaunchConfiguration('use_rtk'))
        ),
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml'])]),
            launch_arguments={'port': '9091'}.items(),
            condition=IfCondition(LaunchConfiguration('start_rosbridge'))
        )
    ])
