# Everything, in the right order, with the right delays.
#   ros2 launch labrobot_bringup robot.launch.py                    # drive + see
#   ros2 launch labrobot_bringup robot.launch.py mapping:=true      # build a map
#   ros2 launch labrobot_bringup robot.launch.py localization:=true # use a map

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include(name, condition=None, args=None):
    src = PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare('labrobot_bringup'), 'launch', name]))
    kwargs = {}
    if args:
        kwargs['launch_arguments'] = args
    if condition is not None:
        return IncludeLaunchDescription(src, condition=condition, **kwargs)
    return IncludeLaunchDescription(src, **kwargs)


def _dds_profile():
    return SetEnvironmentVariable(
        'FASTRTPS_DEFAULT_PROFILES_FILE',
        os.path.join(get_package_share_directory('labrobot_bringup'), 'config', 'fastdds_shm.xml'))

def generate_launch_description():
    mapping = LaunchConfiguration('mapping')
    localization = LaunchConfiguration('localization')
    slam = LaunchConfiguration('slam')
    foxglove = LaunchConfiguration('foxglove')
    grid_3d = LaunchConfiguration('grid_3d')
    pointcloud = LaunchConfiguration('pointcloud')
    decimation = LaunchConfiguration('decimation')

    args = [
        DeclareLaunchArgument(
            'mapping', default_value='false',
            description='Build a NEW map (deletes the existing one).'),
        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Load the saved map and localise in it without extending '
                        'it. Ignored if mapping:=true.'),
        DeclareLaunchArgument(
            'slam', default_value='false',
            description='CONTINUOUS SLAM while navigating: load the existing map, '
                        'localise in it, AND keep extending it. This is the mode '
                        'to use when the robot must both know where it is and '
                        'discover new space. mapping:=true starts from scratch '
                        '(deletes the map); localization:=true refuses to learn '
                        'anything new.'),
        DeclareLaunchArgument(
            'grid_3d', default_value='false',
            description='Also publish a 3D voxel cloud on /cloud_map, for looking '
                        'at in Foxglove. Nav2 does not need it.'),
        # Default true: without the cloud Nav2 sees no new obstacles.
        DeclareLaunchArgument(
            'pointcloud', default_value='true',
            description='Publish the depth point cloud. REQUIRED for Nav2 - it is '
                        'the only obstacle source on this lidar-free robot. Set '
                        'false only for teleop mapping with no Nav2.'),
        DeclareLaunchArgument(
            'decimation', default_value='3',
            description='RealSense decimation for the cloud only; the published '
                        'depth images stay full resolution, so mapping quality is '
                        'unaffected. 3 gives ~43k points at 16 Hz, 10.5 MB/s.'),
        DeclareLaunchArgument(
            'foxglove', default_value='true',
            description='WebSocket bridge on :8765 for Foxglove on your PC. '
                        'Costs nothing while no panel is open.'),
    ]

    return LaunchDescription([_dds_profile()] + args + [
        _include('camera.launch.py',
                 args={'pointcloud': pointcloud, 'decimation': decimation}.items()),
        _include('drivetrain.launch.py'),
        _include('foxglove.launch.py', condition=IfCondition(foxglove)),

        TimerAction(period=22.0, actions=[_include('localization.launch.py')]),

        TimerAction(period=45.0, actions=[
            _include('mapping.launch.py', condition=IfCondition(mapping),
                     args={'delete_db': 'true', 'localization': 'false',
                           'grid_3d': grid_3d}.items()),
            _include('mapping.launch.py', condition=IfCondition(localization),
                     args={'delete_db': 'false', 'localization': 'true',
                           'grid_3d': grid_3d}.items()),
            _include('mapping.launch.py', condition=IfCondition(slam),
                     args={'delete_db': 'false', 'localization': 'false',
                           'grid_3d': grid_3d}.items()),
        ]),
    ])
