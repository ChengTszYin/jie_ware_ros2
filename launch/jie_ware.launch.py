from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jie_ware',
            executable='lidar_filter_node',
            name='lidar_filter_node',
            parameters=[{
                'source_topic': '/scan',
                'pub_topic': '/scan_filtered',
                'outlier_threshold': 0.1
            }],
            output='screen'
        ),
        Node(
            package='jie_ware',
            executable='costmap_cleaner',
            name='costmap_cleaner',
            output='screen'
        ),
        # lidar_loc example
        Node(
            package='jie_ware',
            executable='lidar_loc',
            name='lidar_loc',
            parameters=[{
                'base_frame': 'base_link',
                'odom_frame': 'odom',
                'laser_frame': 'laser',
                'laser_topic': 'scan'
            }],
            output='screen'
        ),
    ])
