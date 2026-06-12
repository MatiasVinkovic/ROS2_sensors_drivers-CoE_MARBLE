#!/usr/bin/env python3
"""
Lance les 5 nœuds :  aanderaa_node  |  aquadopp_node  |  sbe37_node  |  rbrcoda3_node  |  hmi_node

Utilisation :
  ros2 launch marble_sensors_hmi sensors_hmi.launch.py

Surcharger les ports série :
  ros2 launch marble_sensors_hmi sensors_hmi.launch.py \
      aquadopp_port:=/dev/ttyUSB0 \
      aanderaa_port:=/dev/ttyUSB1 \
      sbe37_port:=COM11
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([

        # ── Arguments surchargeables ──────────────────────────────────────────
        DeclareLaunchArgument(
            'aquadopp_port',
            default_value='/dev/ttyUSB0',
            description='Port série Aquadopp S4VP'),

        DeclareLaunchArgument(
            'aanderaa_port',
            default_value='/dev/ttyUSB1',
            description='Port série AANDERAA Motus 5729'),

        DeclareLaunchArgument(
            'aanderaa_sample_interval',
            default_value='10',
            description='Intervalle entre deux mesures AANDERAA (secondes)'),

        DeclareLaunchArgument(
            'sbe37_port',
            default_value='COM11',
            description='Port série SBE 37-SIP MicroCAT'),

        DeclareLaunchArgument(
            'sbe37_baud',
            default_value='9600',
            description='Baud rate SBE 37-SIP (défaut : 9600)'),

        DeclareLaunchArgument(
            'sbe37_sample_interval',
            default_value='10',
            description='Intervalle entre deux TS du SBE 37 (secondes)'),

        DeclareLaunchArgument(
            'rbrcoda3_port',
            default_value='COM11',
            description='Port série RBRcoda3'),

        DeclareLaunchArgument(
            'rbrcoda3_sample_interval',
            default_value='0',
            description='Intervalle min entre deux publications RBRcoda3 (0 = chaque échantillon)'),

        # ── Nœuds ─────────────────────────────────────────────────────────────
        Node(
            package='marble_sensors_hmi',
            executable='aquadopp_node',
            name='aquadopp_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('aquadopp_port'),
                'baud': 115200,
            }],
        ),

        Node(
            package='marble_sensors_hmi',
            executable='aanderaa_node',
            name='aanderaa_node',
            output='screen',
            parameters=[{
                'port':            LaunchConfiguration('aanderaa_port'),
                'baud':            115200,
                'passkey':         '1',
                'sample_interval': LaunchConfiguration('aanderaa_sample_interval'),
            }],
        ),

        Node(
            package='marble_sensors_hmi',
            executable='sbe37_node',
            name='sbe37_node',
            output='screen',
            parameters=[{
                'port':            LaunchConfiguration('sbe37_port'),
                'baud':            LaunchConfiguration('sbe37_baud'),
                'sample_interval': LaunchConfiguration('sbe37_sample_interval'),
            }],
        ),

        Node(
            package='marble_sensors_hmi',
            executable='rbrcoda3_node',
            name='rbrcoda3_node',
            output='screen',
            parameters=[{
                'port':            LaunchConfiguration('rbrcoda3_port'),
                'baud':            9600,
                'sample_interval': LaunchConfiguration('rbrcoda3_sample_interval'),
            }],
        ),

        Node(
            package='marble_sensors_hmi',
            executable='hmi_node',
            name='sensors_hmi',
            output='screen',
        ),
    ])
