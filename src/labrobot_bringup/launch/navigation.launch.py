# Nav2 for labrobot (M4).
#   ros2 launch labrobot_bringup robot.launch.py localization:=true pointcloud:=true
#   ros2 launch labrobot_bringup navigation.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _dds_profile():
    return SetEnvironmentVariable(
        'FASTRTPS_DEFAULT_PROFILES_FILE',
        os.path.join(get_package_share_directory('labrobot_bringup'), 'config', 'fastdds_shm.xml'))

def generate_launch_description():
    pkg = get_package_share_directory('labrobot_bringup')
    default_params = os.path.join(pkg, 'config', 'nav2_params.yaml')

    declare_params = DeclareLaunchArgument('params_file', default_value=default_params)
    declare_autostart = DeclareLaunchArgument('autostart', default_value='true')
    declare_cm = DeclareLaunchArgument(
        'collision_monitor', default_value='true',
        description='Put nav2_collision_monitor between the velocity smoother and the '
                    'drive (config/collision_monitor.yaml). false = previous wiring.')
    return LaunchDescription([_dds_profile()] + [declare_params, declare_autostart, declare_cm,
                              OpaqueFunction(function=_nodes)])


def _nodes(context):
    pkg = get_package_share_directory('labrobot_bringup')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_cm = LaunchConfiguration('collision_monitor').perform(context).lower() in ('true', '1')
    smoothed = '/cmd_vel_smoothed' if use_cm else '/diff_drive_controller/cmd_vel_unstamped'

    common = {'use_sim_time': False, 'autostart': autostart}

    nodes = [
        Node(package='nav2_controller', executable='controller_server',
             output='screen', parameters=[params_file],
             remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen', parameters=[params_file]),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen', parameters=[params_file],
             remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='labrobot_bringup', executable='goal_relay.py',
             name='goal_relay', output='screen'),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen',
             parameters=[params_file, {
                 'default_nav_to_pose_bt_xml': os.path.join(
                     pkg, 'config', 'behavior_trees',
                     'navigate_to_pose_no_backup.xml'),
             }],
             remappings=[('goal_pose', '/labrobot/goal_pose_map')]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen', parameters=[params_file],
             remappings=[('cmd_vel', 'cmd_vel_nav'),
                         ('cmd_vel_smoothed', smoothed)]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', output='screen', parameters=[params_file]),
    ]
    # The lifecycle manager must list every managed node, in start order.
    managed = ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator',
               'waypoint_follower', 'velocity_smoother']
    if use_cm:
        nodes.append(Node(package='nav2_collision_monitor', executable='collision_monitor',
                          name='collision_monitor', output='screen',
                          parameters=[os.path.join(pkg, 'config', 'collision_monitor.yaml')]))
        managed.append('collision_monitor')
    nodes.append(
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{**common, 'node_names': managed}]))
    return nodes
