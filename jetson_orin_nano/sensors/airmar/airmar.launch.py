from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('airmar'),
        'config',
        'airmar.yaml'
    )

    return LaunchDescription([
        Node(
            package='airmar',
            executable='airmar_node',
            name='airmar_node',
            output='screen',
            parameters=[config_file]
        )
    ])