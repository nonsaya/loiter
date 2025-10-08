from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='glim_mavros_bridge',
            executable='bridge_node',
            name='glim_mavros_bridge',
            output='screen',
            parameters=[{
                'input_topic': '/glim/odom',
                'output_topic': '/mavros/odometry/in',
                'frame_id': 'map',
                'child_frame_id': 'base_link',
            }],
        )
    ])


