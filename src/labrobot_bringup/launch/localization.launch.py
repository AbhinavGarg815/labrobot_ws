# Sensor fusion: IMU orientation filter + EKF publishing odom -> base_footprint.
#
#   ros2 launch labrobot_bringup localization.launch.py
#
# Expects drivetrain.launch.py and camera.launch.py to be running already - this
# consumes /diff_drive_controller/odom and /camera/camera/imu.
#
# IMPORTANT: the EKF owns the odom -> base_footprint transform, so
# diff_drive_controller must run with enable_odom_tf: false. Two publishers on
# one TF edge do not average out; consumers just see whichever arrived last.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('labrobot_bringup')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
    imu_config = os.path.join(pkg_share, 'config', 'imu_filter.yaml')

    use_imu_filter = LaunchConfiguration('use_imu_filter')

    declare_imu_filter = DeclareLaunchArgument(
        'use_imu_filter',
        default_value='true',
        description=(
            'Run the Madgwick filter. Its orientation output is for RTAB-Map '
            'gravity constraints; the EKF does not use it (see ekf.yaml).'
        ),
    )

    # Produces /imu/data (with orientation) from the driver's raw /camera/camera/imu.
    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        condition=IfCondition(use_imu_filter),
        parameters=[imu_config],
        remappings=[
            ('imu/data_raw', '/camera/camera/imu'),
            ('imu/data', '/imu/data'),
        ],
    )

    # Must run BEFORE the EKF sees any IMU data, and needs the robot stationary
    # for its first few seconds. See the node's docstring for why neither
    # robot_localization nor imu_filter_madgwick can do this job.
    imu_bias = Node(
        package='labrobot_bringup',
        executable='imu_bias_compensator.py',
        name='imu_bias_compensator',
        output='screen',
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        # The EKF's own output. Left un-remapped from the default /odometry/filtered
        # so it is obvious in a topic list which odometry is raw and which is fused.
    )

    return LaunchDescription([
        declare_imu_filter,
        imu_filter,
        imu_bias,
        ekf,
    ])
