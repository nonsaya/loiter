from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='glim_ros2',
            executable='glim_node',
            name='glim',
            output='screen',
            remappings=[
                # Ensure GLIM subscribes to Livox topics regardless of internal names
                ('/glim/points', '/livox/lidar'),
                ('points', '/livox/lidar'),
                ('/glim/imu', '/livox/imu'),
                ('imu', '/livox/imu'),
            ],
            parameters=[
                {'build_with_cuda': False},
            ],
        )
    ])
