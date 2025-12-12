from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rm3100_driver',
            executable='rm3100_driver.py',
            name='rm3100_driver',
            output='screen',
            parameters=[
                {'port': 'can0'},
                {'node_id': 124},
                {'bitrate': 1000000},
                {'topic': 'mag'},
                {'frame_id': 'mag'},
                {'publish_rate': 10}
            ]
        )
    ])
