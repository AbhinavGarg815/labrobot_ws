# Nav2 for labrobot (M4).
#
#   ros2 launch labrobot_bringup robot.launch.py localization:=true pointcloud:=true
#   ros2 launch labrobot_bringup navigation.launch.py
#
# Requires a MAP and LOCALIZATION already running - Nav2 plans in the map frame
# and will sit waiting forever without map -> odom.
#
# What provides what, because this is the part people wire up twice
# -----------------------------------------------------------------
#   map -> odom              RTAB-Map (mapping.launch.py localization:=true)
#   odom -> base_footprint   the EKF (localization.launch.py)
#   /map                     RTAB-Map's occupancy grid -> global costmap static layer
#   obstacles                D435i point cloud -> voxel/obstacle layers
#   /cmd_vel                 Nav2 -> velocity_smoother -> diff_drive_controller
#
# AMCL is NOT used: it needs a LaserScan this robot does not have, and RTAB-Map
# already publishes map -> odom from visual place recognition.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('labrobot_bringup')
    default_params = os.path.join(pkg, 'config', 'nav2_params.yaml')

    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    declare_params = DeclareLaunchArgument('params_file', default_value=default_params)
    declare_autostart = DeclareLaunchArgument('autostart', default_value='true')

    # diff_drive_controller subscribes to cmd_vel_unstamped, so the smoother's
    # output is remapped onto it. Nav2's controller writes /cmd_vel, the smoother
    # reads that and writes /cmd_vel_smoothed - see the remap below.
    common = {'use_sim_time': False, 'autostart': autostart}

    nodes = [
        Node(package='nav2_controller', executable='controller_server',
             output='screen', parameters=[params_file],
             remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen', parameters=[params_file]),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen', parameters=[params_file]),
        # Custom tree: the stock one minus the BackUp recovery, which is a
        # blind and (on this chassis) heavily rotating reverse. See the XML.
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen',
             parameters=[params_file, {
                 'default_nav_to_pose_bt_xml': os.path.join(
                     pkg, 'config', 'behavior_trees',
                     'navigate_to_pose_no_backup.xml'),
             }]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen', parameters=[params_file],
             remappings=[('cmd_vel', 'cmd_vel_nav'),
                         ('cmd_vel_smoothed',
                          '/diff_drive_controller/cmd_vel_unstamped')]),
        # The lifecycle manager must list every managed node, in start order.
        # A node missing here simply never activates, and the symptom is Nav2
        # accepting a goal and then doing nothing at all.
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{**common,
                          'node_names': ['controller_server',
                                         'planner_server',
                                         'behavior_server',
                                         'bt_navigator',
                                         'velocity_smoother']}]),
    ]

    return LaunchDescription([declare_params, declare_autostart] + nodes)
