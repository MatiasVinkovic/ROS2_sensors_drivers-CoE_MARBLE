from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('oculus'),
        'config',
        'oculus.yaml'
    )

    return LaunchDescription([
        Node(
            package='oculus',
            executable='oculus_node',
            name='oculus_node',
            output='screen',
            parameters=[config_file]
        )
    ])