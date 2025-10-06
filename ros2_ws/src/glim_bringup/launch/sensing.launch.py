from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    glim_config_path = LaunchConfiguration('glim_config_path', default='/home/nonsaya-n/glim_config')

    livox_launch = os.path.join(
        get_package_share_directory('livox_mid360_bringup'), 'launch', 'livox_mid360.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('glim_config_path', default_value=glim_config_path),

        # Livox MID360 driver
        IncludeLaunchDescription(PythonLaunchDescriptionSource(livox_launch)),

        # GLIM
        Node(
            package='glim_ros',
            executable='glim_rosnode',
            name='glim',
            output='screen',
            parameters=[{'config_path': glim_config_path}],
            remappings=[('/glim/points', '/livox/lidar'), ('/glim/imu', '/livox/imu')],
        ),
    ])




