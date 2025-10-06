from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource, XMLLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    fcu_url = LaunchConfiguration('fcu_url', default='serial:///dev/ttyTHS1:921600')
    glim_config_path = LaunchConfiguration('glim_config_path', default='/home/nonsaya-n/glim_config')
    use_corrected = LaunchConfiguration('use_corrected', default='false')
    publish_rate_hz = LaunchConfiguration('publish_rate_hz', default='15.0')
    odom_child_frame_id = LaunchConfiguration('odom_child_frame_id', default='base_link')
    restamp_source = LaunchConfiguration('restamp_source', default='now')
    reject_older_than_ms = LaunchConfiguration('reject_older_than_ms', default='200.0')
    publish_immediately = LaunchConfiguration('publish_immediately', default='true')
    target_topic = LaunchConfiguration('target_topic', default='/mavros/odometry/out')

    livox_launch = os.path.join(
        get_package_share_directory('livox_mid360_bringup'), 'launch', 'livox_mid360.launch.py')
    mavros_launch = os.path.join(
        get_package_share_directory('mavros'), 'launch', 'apm.launch')

    return LaunchDescription([
        DeclareLaunchArgument('fcu_url', default_value=fcu_url),
        DeclareLaunchArgument('glim_config_path', default_value=glim_config_path),
        DeclareLaunchArgument('use_corrected', default_value=use_corrected),
        DeclareLaunchArgument('publish_rate_hz', default_value=publish_rate_hz),
        DeclareLaunchArgument('odom_child_frame_id', default_value=odom_child_frame_id),
        DeclareLaunchArgument('restamp_source', default_value=restamp_source),
        DeclareLaunchArgument('reject_older_than_ms', default_value=reject_older_than_ms),
        DeclareLaunchArgument('publish_immediately', default_value=publish_immediately),
        DeclareLaunchArgument('target_topic', default_value=target_topic),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(livox_launch)),

        Node(
            package='glim_ros',
            executable='glim_rosnode',
            name='glim',
            output='screen',
            parameters=[{'config_path': glim_config_path}],
            remappings=[('/glim/points', '/livox/lidar'), ('/glim/imu', '/livox/imu')],
        ),

        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(mavros_launch),
            launch_arguments={'fcu_url': fcu_url}.items(),
        ),

        Node(
            package='glim_mavros_bridge',
            executable='odom_bridge_node',
            name='glim_mavros_odometry_bridge',
            output='screen',
            parameters=[{
                'glim_namespace': '/glim_ros',
                'use_corrected': use_corrected,
                'publish_rate_hz': publish_rate_hz,
                'odom_child_frame_id': odom_child_frame_id,
                'restamp_source': restamp_source,
                'reject_older_than_ms': reject_older_than_ms,
                'publish_immediately': publish_immediately,
                'target_topic': target_topic,
            }],
        ),
    ])


