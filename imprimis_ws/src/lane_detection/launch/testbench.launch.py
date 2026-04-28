import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true pointcloud.enable:=true

def generate_launch_description():
    pkg_dir = get_package_share_directory('lane_detection')
    costmap_config = os.path.join(pkg_dir, 'config', 'isolated_costmap.yaml')

    no_roi_arg = DeclareLaunchArgument(
        'no_roi',
        default_value='false',
        description='If true, no crop and processes the entire image.'
    )

    no_roi_val = LaunchConfiguration('no_roi')

    return LaunchDescription([
        no_roi_arg, 
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.2', '0', '0.5', '-1.57', '0', '-1.57', 'base_link', 'camera_color_optical_frame']
        ),

        Node(
            package='lane_detection',
            executable='lane_detection_node',
            name='lane_detection_node',
            output='screen',
            parameters=[{'no_roi': no_roi_val}] 
        ),

        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='local_costmap',
            output='screen',
            parameters=[costmap_config]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap',
            output='screen',
            parameters=[
                {'use_sim_time': False},
                {'autostart': True},
                {'node_names': ['local_costmap']},
                {'bond_timeout': 0.0}
            ]
        )
    ])