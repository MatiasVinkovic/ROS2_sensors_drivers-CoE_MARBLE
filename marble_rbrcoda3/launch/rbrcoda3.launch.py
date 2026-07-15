from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('rbrcoda3'),
        'config',
        'rbrcoda3.yaml'
    )

    return LaunchDescription([
        Node(
            package='rbrcoda3',
            executable='rbrcoda3_node',
            name='rbrcoda3_node',
            output='screen',
            parameters=[config_file]
        )
    ])
