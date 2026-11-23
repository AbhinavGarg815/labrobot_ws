# RTAB-Map RGB-D visual odometry, run as a MEASUREMENT ONLY (M3.T1).
#   ros2 launch labrobot_bringup visual_odometry.launch.py

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

RSUSB_LIB = os.path.expanduser('~/opt/librealsense-rsusb/lib')


def generate_launch_description():
    approx_sync = LaunchConfiguration('approx_sync')
    decimation = LaunchConfiguration('decimation')
    max_features = LaunchConfiguration('max_features')

    declare_decimation = DeclareLaunchArgument(
        'decimation', default_value='2',
        description=('Image decimation before feature extraction. 2 quarters the '
                     'pixel count, which is close to a 4x speedup, at the cost of '
                     'localisation precision on distant features.'),
    )
    declare_max_features = DeclareLaunchArgument(
        'max_features', default_value='600',
        description=('Cap on features per frame (RTAB-Map default 1000). Indoor '
                     'scenes here are feature-rich enough that the extra 400 buy '
                     'little beyond CPU.'),
    )

    declare_approx_sync = DeclareLaunchArgument(
        'approx_sync', default_value='false',
        description=(
            'realsense publishes aligned_depth_to_color with the SAME header '
            'stamp as the colour image, so exact sync should match and is '
            'stricter - it guarantees the pair is genuinely simultaneous rather '
            'than merely close. Set true if the odometry topic stays silent, '
            'which is what a failed exact sync looks like.'
        ),
    )

    visual_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='visual_odometry',
        output='screen',
        parameters=[{
            # The robot frame the odometry is expressed for.
            'frame_id': 'base_footprint',
            'odom_frame_id': 'vo_odom',
            'publish_tf': False,

            'approx_sync': approx_sync,
            'queue_size': 30,

            'Odom/Strategy': '0',
            # Do NOT silently reset on tracking loss.
            'Odom/ResetCountdown': '0',

            'Vis/MinDepth': '0.3',
            'Vis/MaxDepth': '4.0',
            'Odom/ImageDecimation': ParameterValue(decimation, value_type=str),
            'Vis/MaxFeatures': ParameterValue(max_features, value_type=str),
        }],
        remappings=[
            ('rgb/image', '/camera/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/camera/color/camera_info'),
            ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
            ('odom', '/vo/odom'),
        ],
        additional_env={
            'LD_LIBRARY_PATH': RSUSB_LIB + ':' + os.environ.get('LD_LIBRARY_PATH', '')
        },
    )

    return LaunchDescription([
        declare_approx_sync,
        declare_decimation,
        declare_max_features,
        visual_odometry,
    ])
