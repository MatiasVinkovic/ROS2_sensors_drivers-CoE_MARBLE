from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = '/home/cytech/ros2_ws/src/marble_sensors_hmi/marble_sensors_hmi/yaml/oculus.yaml'

    return LaunchDescription([
        Node(
            package='marble_sensors_hmi',
            executable='oculus_node',
            name='oculus_node',
            output='screen',
            parameters=[config_file]
        )
    ])