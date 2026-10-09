"""map_server + planner_server + a static map->base_footprint TF, nothing else."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
HERE = os.path.dirname(os.path.abspath(__file__))
def generate_launch_description():
    params = LaunchConfiguration('params')
    m = LaunchConfiguration('map')
    return LaunchDescription([
        DeclareLaunchArgument('map'),
        DeclareLaunchArgument('params', default_value=os.path.join(HERE, 'params.yaml')),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--frame-id', 'map', '--child-frame-id', 'base_footprint']),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[params, {'yaml_filename': m}]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             parameters=[params]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_rig',
             parameters=[{'autostart': True, 'node_names': ['map_server', 'planner_server']}]),
    ])
