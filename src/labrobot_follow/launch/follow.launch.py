# Person following: perception (GPU) + follower (Nav2 goals).
#   ros2 launch labrobot_follow follow.launch.py                      # follows (robot MOVES on "start")
#   ros2 launch labrobot_follow follow.launch.py dry_run:=true        # goals + plans only
#   ros2 launch labrobot_follow follow.launch.py tracker_only:=true   # perception only

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

ENGINES = os.path.expanduser('~/labrobot_models/engines')


def _dds_profile():
    return SetEnvironmentVariable(
        'FASTRTPS_DEFAULT_PROFILES_FILE',
        os.path.join(get_package_share_directory('labrobot_bringup'), 'config', 'fastdds_shm.xml'))

def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('labrobot_follow'), 'config', 'follow.yaml')
    detector = LaunchConfiguration('detector')
    reid = LaunchConfiguration('reid')
    args = [
        DeclareLaunchArgument('detector', default_value='yolo26s-pose',
                              description='pose engine: yolo26s-pose | yolo26n-pose | yolo11*-pose'),
        DeclareLaunchArgument('reid', default_value='osnet_ain_x1_0_msmt17',
                              description='re-ID engine: osnet_ain_x1_0_msmt17 | osnet_x0_25_msmt17'),
        DeclareLaunchArgument('dry_run', default_value='false'),
        DeclareLaunchArgument('max_speed', default_value='0.06',
                              description='follow speed cap, m/s (smoother cap still applies)'),
        DeclareLaunchArgument('tracker_only', default_value='false'),
    ]
    tracker = Node(
        package='labrobot_follow', executable='person_tracker.py', name='person_tracker',
        output='screen', parameters=[cfg, {
            'detector_engine': PythonExpression(["'", ENGINES, "/' + '", detector,
                                                 "' + '_480x640_fp16.engine'"]),
            'reid_engine': PythonExpression(["'", ENGINES, "/' + '", reid, "' + '_fp16.engine'"]),
        }])
    follower = Node(
        package='labrobot_follow', executable='person_follower.py', name='person_follower',
        output='screen', condition=UnlessCondition(LaunchConfiguration('tracker_only')),
        parameters=[cfg, {
            'dry_run': PythonExpression(["'", LaunchConfiguration('dry_run'), "'.lower() == 'true'"]),
            'max_speed': PythonExpression(['float(', LaunchConfiguration('max_speed'), ')']),
        }])
    return LaunchDescription([_dds_profile()] + args + [tracker, follower])
