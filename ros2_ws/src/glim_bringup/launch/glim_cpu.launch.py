from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    cfg_dir = os.path.join(get_package_share_directory('glim_bringup'), 'config', 'glim_config')
    # Relative path from glim package share dir
    rel_cfg = os.path.relpath(cfg_dir, os.path.join(get_package_share_directory('glim')))
    return LaunchDescription([
        Node(
            package='glim_ros',
            executable='glim_rosnode',
            name='glim',
            output='screen',
            remappings=[
                ('/glim/points', '/livox/lidar'),
                ('points', '/livox/lidar'),
                ('/glim/imu', '/livox/imu'),
                ('imu', '/livox/imu'),
            ],
            parameters=[
                {'build_with_cuda': False},
                {'config_path': rel_cfg},
            ],
        )
    ])
