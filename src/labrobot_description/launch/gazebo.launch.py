# Bring up the labrobot in Gazebo Classic with ros2_control.
#
# Starts Gazebo, publishes the robot description, spawns the robot, and then
# activates the controllers in order (joint_state_broadcaster first, then the
# diff_drive_controller). Drive the robot with:
#
#   ros2 run teleop_twist_keyboard teleop_twist_keyboard \
#       --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped

import os
import re

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = 'labrobot_description'
    pkg_share = get_package_share_directory(package_name)

    xacro_file = os.path.join(pkg_share, 'urdf', 'labrobot.urdf.xacro')
    # gazebo_ros2_control 0.4.x re-publishes robot_description to the
    # controller_manager as a node argument; rcl's argument parser rejects
    # embedded newlines, so flatten the URDF to a single comment-free line.
    raw_urdf = xacro.process_file(xacro_file).toxml()
    raw_urdf = re.sub(r'<!--.*?-->', '', raw_urdf, flags=re.DOTALL)
    robot_description = ' '.join(raw_urdf.split())

    rviz_config = os.path.join(pkg_share, 'rviz', 'labrobot.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')
    rviz = LaunchConfiguration('rviz')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use the Gazebo (simulation) clock.',
    )

    declare_gui = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to "false" to run Gazebo headless (no gzclient GUI).',
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Open RViz2 to visualise the robot, ground truth and odometry.',
    )

    declare_world = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution(
            [FindPackageShare('gazebo_ros'), 'worlds', 'empty.world']
        ),
        description='Absolute path to a Gazebo world file.',
    )

    # robot_state_publisher: TF tree + /robot_description for the spawner.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    # Gazebo Classic (gzserver + gzclient) with the ROS factory/init plugins.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py']
            )
        ),
        launch_arguments={'gui': gui, 'world': world}.items(),
    )

    # Spawn the robot in Gazebo from the /robot_description topic.
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_labrobot',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'labrobot',
            '-z', '0.0',
        ],
    )

    # Controller spawners. gazebo_ros2_control starts the controller_manager
    # inside Gazebo, so these only need to load + activate the controllers.
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Static map -> odom transform (identity). The diff_drive_controller owns
    # odom -> base_link; the ground-truth plugin reports poses in "map". This
    # link joins both into one TF tree so RViz can overlay them under a single
    # fixed frame. As wheel odometry drifts, odom slides away from map and the
    # two trajectories visibly separate.
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--yaw', '0', '--pitch', '0', '--roll', '0',
            '--frame-id', 'map', '--child-frame-id', 'odom',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # RViz2 with the saved view (robot model, TF, ground-truth + odometry).
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
        output='screen',
    )

    # Sequence: spawn robot -> joint_state_broadcaster -> diff_drive_controller.
    load_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    load_diff_drive_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_gui,
        declare_world,
        declare_rviz,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        load_joint_state_broadcaster,
        load_diff_drive_controller,
        map_to_odom,
        rviz_node,
    ])
