from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    glim_namespace = LaunchConfiguration('glim_namespace', default='/glim')
    use_corrected = LaunchConfiguration('use_corrected', default='false')
    publish_rate_hz = LaunchConfiguration('publish_rate_hz', default='15.0')
    odom_child_frame_id = LaunchConfiguration('odom_child_frame_id', default='')
    restamp_source = LaunchConfiguration('restamp_source', default='now')
    reject_older_than_ms = LaunchConfiguration('reject_older_than_ms', default='200.0')
    publish_immediately = LaunchConfiguration('publish_immediately', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('glim_namespace', default_value='/glim'),
        DeclareLaunchArgument('use_corrected', default_value='false'),
        DeclareLaunchArgument('publish_rate_hz', default_value='15.0'),
        DeclareLaunchArgument('odom_child_frame_id', default_value=''),
        DeclareLaunchArgument('restamp_source', default_value='now'),
        DeclareLaunchArgument('reject_older_than_ms', default_value='200.0'),
        DeclareLaunchArgument('publish_immediately', default_value='true'),

        Node(
            package='glim_mavros_bridge',
            executable='odom_bridge_node',
            name='glim_mavros_odometry_bridge',
            output='screen',
            parameters=[{
                'glim_namespace': glim_namespace,
                'use_corrected': use_corrected,
                'publish_rate_hz': publish_rate_hz,
                'odom_child_frame_id': odom_child_frame_id,
                'restamp_source': restamp_source,
                'reject_older_than_ms': reject_older_than_ms,
                'publish_immediately': publish_immediately,
            }]
        ),
    ])


