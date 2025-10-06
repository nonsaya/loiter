from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='glim_mavros_bridge',
            executable='vision_bridge_node',
            name='glim_mavros_vision_bridge',
            output='screen',
            parameters=[{
                'input_pose_topic': '/glim/pose',
                'output_pose_topic': '/mavros/vision_pose/pose',
            }]
        ),
    ])



