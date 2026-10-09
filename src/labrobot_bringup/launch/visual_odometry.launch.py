# RTAB-Map RGB-D visual odometry, run as a MEASUREMENT ONLY (M3.T1).
#
#   ros2 launch labrobot_bringup visual_odometry.launch.py
#
# Requires camera.launch.py (with align_depth, the default) to be running.
#
# Why this is deliberately not wired into the robot yet
# ----------------------------------------------------
# Two rules govern this file, and both exist to keep the validation honest:
#
#  1. publish_tf: FALSE. The EKF owns odom -> base_footprint. Two publishers on
#     one TF edge do not average, they alternate, and the tree jitters between
#     whichever wrote last. Visual odometry publishes to a topic here and to
#     nothing else.
#
#  2. NO guess_frame_id. RTAB-Map can take a motion prior from another odometry
#     source, which genuinely helps it survive fast turns and blank walls. But
#     seeding it from the EKF and then comparing the two would be circular - the
#     answer would inherit the very gyro scale error (~1.4%, Problem 25) that
#     this exercise exists to measure. So it runs blind, on images alone, and
#     the comparison means something.
#
# Once M3.T1 has a number, the intended production wiring is the opposite of
# this: visual odometry becomes a second input to the EKF (odom1), the EKF keeps
# ownership of the TF edge, and a motion guess is fed back in.

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

    # Measured on this Jetson: at full 640x480 with 1000 features, rgbd_odometry
    # managed 7.5 Hz from a 30 Hz input while using 90% of a core. Dropping 3 of
    # every 4 frames is worse than it sounds - the baseline between PROCESSED
    # frames grows, which is exactly when feature matching starts to fail.
    # These two knobs are the ones that move that number.
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
            # The robot frame the odometry is expressed for. RTAB-Map chains
            # base_footprint -> camera_color_optical_frame through TF itself, so
            # the mount calibration from M2.T1 (10.78 deg pitch) is what makes
            # this correct.
            'frame_id': 'base_footprint',
            'odom_frame_id': 'vo_odom',
            'publish_tf': False,

            'approx_sync': approx_sync,
            'queue_size': 30,

            # Frame-to-Map: matches each frame against an accumulated local map
            # rather than only the previous frame, which stops per-frame errors
            # from random-walking. Costs more CPU than frame-to-frame.
            'Odom/Strategy': '0',
            # Do NOT silently reset on tracking loss. For validation a reset that
            # quietly zeroes the pose would look like perfect odometry while
            # actually meaning the opposite.
            'Odom/ResetCountdown': '0',

            # The blind zone is 0.79 m ahead of the bumper (M2.T1) and depth
            # noise grows with range squared, so trust neither extreme.
            'Vis/MinDepth': '0.3',
            'Vis/MaxDepth': '4.0',
            # ParameterValue(..., value_type=str) is REQUIRED, not decoration:
            # launch infers "2" as an integer, and every RTAB-Map config
            # parameter is declared as a string. The node then aborts on startup
            # with InvalidParameterTypeException and exit code -6.
            'Odom/ImageDecimation': ParameterValue(decimation, value_type=str),
            'Vis/MaxFeatures': ParameterValue(max_features, value_type=str),
        }],
        remappings=[
            ('rgb/image', '/camera/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/camera/color/camera_info'),
            ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
            ('odom', '/vo/odom'),
        ],
        # Same RSUSB reasoning as camera.launch.py - harmless here, but keeps
        # every node in this stack resolving the same librealsense.
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
