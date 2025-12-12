import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('dronecan_driver'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='dronecan_driver',
            executable='dronecan_driver.py',
            name='dronecan_driver',
            output='screen',
            parameters=[config]
        )
    ])
