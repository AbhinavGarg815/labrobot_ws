# Bring up the labrobot drivetrain on real hardware.
#
# Starts robot_state_publisher, the ros2_control node talking to the Arduino
# Mega, and the two controllers in order.
#
#   ros2 launch labrobot_bringup drivetrain.launch.py
#
# Drive it with:
#   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
#       -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
#
# SAFETY: this robot has no kill switch. The firmware brakes if commands stop
# arriving, but that cannot cover an MCU hang.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_directory('labrobot_description')

    xacro_file = os.path.join(description_share, 'urdf', 'labrobot.urdf.xacro')
    controllers_file = os.path.join(description_share, 'config', 'controllers.yaml')
    rviz_config = os.path.join(description_share, 'rviz', 'labrobot.rviz')

    mcu_port = LaunchConfiguration('mcu_port')
    rviz = LaunchConfiguration('rviz')

    declare_mcu_port = DeclareLaunchArgument(
        'mcu_port',
        default_value='/dev/labrobot_mcu',
        description='Serial device for the Arduino Mega (a udev symlink pinned '
                    'to the board USB serial number).',
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Open RViz. Leave false when running headless over SSH.',
    )

    # Expanded at launch so mcu_port reaches the hardware plugin. value_type=str
    # is required or the URDF is mis-inferred as another parameter type.
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' mcu_port:=', mcu_port]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
    )

    # On hardware the controller_manager is its own process, unlike Gazebo where
    # the plugin hosts it. It gets the description directly so the hardware
    # plugin can read its <param> block.
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[{'robot_description': robot_description}, controllers_file],
    )

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

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(rviz),
        output='screen',
    )

    # Order matters: joint_state_broadcaster first so /joint_states exists
    # before anything consumes it, then the drive controller.
    load_diff_drive_after_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    return LaunchDescription([
        declare_mcu_port,
        declare_rviz,
        robot_state_publisher,
        control_node,
        joint_state_broadcaster_spawner,
        load_diff_drive_after_broadcaster,
        rviz_node,
    ])
