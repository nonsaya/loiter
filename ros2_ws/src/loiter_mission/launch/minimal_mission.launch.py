from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='loiter_mission',
            executable='minimal_mission_node',
            name='minimal_mission_node',
            output='screen',
            parameters=[{
                'takeoff_altitude': 1.0,
                'hover_seconds': 10.0,
            }],
        ),
    ])


