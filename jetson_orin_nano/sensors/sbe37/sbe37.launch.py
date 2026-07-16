from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('sbe37'),
        'config',
        'sbe37.yaml'
    )

    return LaunchDescription([
        Node(
            package='sbe37',
            executable='sbe37_node',
            name='sbe37_node',
            output='screen',
            parameters=[config_file]
        )
    ])