# Visualise the labrobot in RViz2 (no simulation, no hardware).
#
# Publishes the robot description and the TF tree. With gui:=true (default) a
# joint_state_publisher_gui lets you articulate the wheels.
#
# Headless TF check over SSH:
#     ros2 launch labrobot_description display.launch.py gui:=false rviz:=false

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('labrobot_description')

    xacro_file = os.path.join(pkg_share, 'urdf', 'labrobot.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'labrobot.rviz')

    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    use_sim = LaunchConfiguration('use_sim')

    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start joint_state_publisher_gui to articulate the joints.',
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Open RViz2. Set false for headless checks over SSH.',
    )

    declare_use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Build the simulation variant of the description.',
    )

    # Expand the xacro at launch time so use_sim is honoured. ParameterValue
    # with value_type=str is required: without it the URDF is mis-inferred as a
    # different parameter type. Passing the URDF as a --ros-args override
    # instead would fail, because rcl rejects embedded newlines.
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_sim:=', use_sim]),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(gui),
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(gui),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(rviz),
        output='screen',
    )

    return LaunchDescription([
        declare_gui,
        declare_rviz,
        declare_use_sim,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
