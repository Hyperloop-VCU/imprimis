from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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
        description='Shows a map of current GPS GGA',
    )

    return LaunchDescription([
        sim_arg,
        show_map_arg,

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
                    'debug': True,
                    'caster': "rtk.geodnet.com" ,
                    'port': 2101,
                    'mountpoint': "AUTO" ,
                    'baud_rate': 38400,
                    'gps_port': "/dev/ttyACM0",
                }
            ]
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
        ),
    ])
