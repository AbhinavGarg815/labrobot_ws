# Remote visualisation bridge for Foxglove running on your own PC.
#   ros2 launch labrobot_bringup foxglove.launch.py
#   ssh -N -L 8765:localhost:8765 zine@<jetson-ip>

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Everything except raw images and the raw point cloud (see the bridge comment).
WIFI_SAFE_TOPICS = ['^(?!.*(/image_raw|/image_rect_raw|/points)$).*$']


def generate_launch_description():
    port = LaunchConfiguration('port')
    address = LaunchConfiguration('address')
    heavy = LaunchConfiguration('heavy')

    declare_port = DeclareLaunchArgument('port', default_value='8765')
    declare_heavy = DeclareLaunchArgument(
        'heavy', default_value='false',
        description='true: also offer raw images and the raw point cloud. Only on '
                    'a wired link and never while Nav2 is driving - see below.')
    declare_address = DeclareLaunchArgument(
        'address', default_value='127.0.0.1',
        description=('127.0.0.1: reachable only through an SSH tunnel '
                     '(ssh -L 8765:localhost:8765 ...). 0.0.0.0 listens on every '
                     'interface - no authentication, use only on a trusted network.'),
    )

    def bridge(whitelist, condition):
        return Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{
                'port': port,
                'address': address,
                # permessage-deflate, kept ON even though JPEG will not shrink.
                'use_compression': True,
                # Max send buffer: beyond it the bridge drops stale frames (bounds latency).
                'send_buffer_limit': 1000000,
                # Shallow queues: only the newest frame is worth sending.
                'max_qos_depth': 2,
                'num_threads': 0,          # 0 = hardware concurrency
                # Raw images and clouds are hidden: over WiFi they starve the robot.
                'topic_whitelist': whitelist,
                'capabilities': ['clientPublish', 'parameters', 'parametersSubscribe',
                                 'services', 'connectionGraph', 'assets'],
            }],
            condition=condition,
        )

    return LaunchDescription([
        declare_port,
        declare_address,
        declare_heavy,
        bridge(WIFI_SAFE_TOPICS, UnlessCondition(heavy)),
        bridge(['.*'], IfCondition(heavy)),
    ])
