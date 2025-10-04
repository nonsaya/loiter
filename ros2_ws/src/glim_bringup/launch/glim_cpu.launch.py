from launch import LaunchDescription
from launch_ros.actions import Node
import os
try:
    from ament_index_python.packages import get_package_share_directory
except Exception:
    get_package_share_directory = None


def generate_launch_description():
    # Try installed share first, then fallback to source
    cfg_dir = None
    if get_package_share_directory is not None:
        try:
            share_dir = get_package_share_directory('glim_bringup')
            cfg_dir = os.path.join(share_dir, 'config', 'glim_config')
        except Exception:
            cfg_dir = None
    if cfg_dir is None or not os.path.isfile(os.path.join(cfg_dir, 'config.json')):
        here = os.path.dirname(__file__)
        cfg_dir = os.path.abspath(os.path.join(here, '..', 'config', 'glim_config'))
    return LaunchDescription([
        # LiDAR-only odometry diagnostic path (set via env or quick switch if needed)
        # Switch between CPU LiDAR-IMU and LiDAR-only by changing config file below.
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
                {'config_path': cfg_dir},
                # To force LiDAR-only: point config_odometry to config_odometry_ct.json in config.json
            ],
        )
    ])
