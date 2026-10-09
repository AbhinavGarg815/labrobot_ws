# RTAB-Map RGB-D SLAM (M3.T2).
#
#   ros2 launch labrobot_bringup mapping.launch.py
#
# Needs camera.launch.py, drivetrain.launch.py and localization.launch.py first.
#
# The one decision that shapes this file
# --------------------------------------
# RTAB-Map does NOT compute its own odometry here. It is given the EKF's, because
# M3.T1 measured both over the same 360 deg spin:
#
#                     rotation      translation (truth: zero)
#   EKF (wheels+gyro) 360.52 deg    0.2 mm
#   visual odometry   360.04 deg    70.8 mm
#
# They agree on rotation to 0.13%, but visual odometry drifts 70 mm of phantom
# translation per revolution - during pure rotation the baseline is tiny, so
# per-frame errors integrate freely. The wheels are two orders of magnitude
# better at translation and cannot slip sideways, and the gyro cannot slip at
# all. So the camera is used for what only it can do: recognising a place it has
# been before, and correcting the accumulated drift through loop closure.
#
# TF ownership, which must stay unambiguous:
#   map   -> odom            RTAB-Map   (loop-closure correction)
#   odom  -> base_footprint  the EKF    (see ekf.yaml)
# Neither touches the other's edge. Two publishers on one edge do not average,
# they alternate, and the tree jitters between whichever wrote last.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    delete_db = LaunchConfiguration('delete_db')
    database_path = LaunchConfiguration('database_path')
    localization = LaunchConfiguration('localization')
    grid_3d = LaunchConfiguration('grid_3d')

    declare_grid_3d = DeclareLaunchArgument(
        'grid_3d', default_value='false',
        description=(
            'Publish a 3D voxel cloud on /cloud_map as well as the 2D grid. '
            'Nav2 needs only the 2D grid, and 3D costs memory and CPU that buys '
            'the robot nothing - so this is for LOOKING at the map in Foxglove, '
            'not for navigating. The RGB-D data is stored per node either way, so '
            'turning this on later loses nothing.'
        ),
    )

    declare_localization = DeclareLaunchArgument(
        'localization', default_value='false',
        description=(
            'LOCALIZATION-ONLY: load the existing map and localise in it without '
            'adding to it. This is the mode Nav2 runs in. Mapping mode keeps '
            'growing the graph, so on a robot that lives in one room it would '
            'accumulate memory and re-map the same lab forever.'
        ),
    )

    declare_delete_db = DeclareLaunchArgument(
        'delete_db', default_value='true',
        description=('Start a fresh map. Set false to CONTINUE the existing one - '
                     'appending to a map built in a different room silently '
                     'produces nonsense loop closures.'),
    )
    declare_database_path = DeclareLaunchArgument(
        'database_path', default_value='~/.ros/rtabmap.db',
        description='Where the map lives. Save/load is just this file.',
    )

    # Every RTAB-Map config parameter is declared as a STRING. Passing a launch
    # argument that looks numeric makes launch infer an int, and the node dies at
    # startup with InvalidParameterTypeException and exit code -6.
    def s(v):
        return ParameterValue(v, value_type=str)

    def if_loc(when_true, when_false):
        """Pick a string parameter based on the `localization` argument."""
        return ParameterValue(
            PythonExpression(["'", when_true, "' if '", localization,
                              "' == 'true' else '", when_false, "'"]),
            value_type=str)

    params = [{
        'frame_id': 'base_footprint',
        'map_frame_id': 'map',
        'odom_frame_id': 'odom',
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_scan': False,
        'approx_sync': False,
        'sync_queue_size': 30,
        'database_path': database_path,
        # Publishes map -> odom only; the EKF keeps odom -> base_footprint.
        'publish_tf': True,

        # --- planar robot -------------------------------------------------
        # The robot cannot leave the floor plane, so telling the optimiser
        # that removes three degrees of freedom that only noise could move
        # and stops the map slowly tilting out of the ground plane.
        'Reg/Force3DoF': s('true'),
        'Optimizer/Slam2D': s('true'),

        # --- 2D occupancy grid for Nav2 ------------------------------------
        'Grid/FromDepth': s('true'),
        'Grid/3D': ParameterValue(grid_3d, value_type=str),
        'Grid/CellSize': s('0.05'),
        # Ray tracing marks the space BETWEEN the camera and an obstacle as
        # free. Without it the map only ever gains occupied cells and never
        # clears anything that moved away.
        'Grid/RayTracing': s('true'),
        'Grid/MaxGroundHeight': s('0.05'),
        # Above the robot's own height nothing can block it, and including
        # ceilings and shelf undersides would wall the map in.
        'Grid/MaxObstacleHeight': s('0.90'),
        'Grid/RangeMin': s('0.30'),
        # Depth noise grows with range squared; beyond 4 m this camera's
        # points are too coarse to be worth writing into a 5 cm grid.
        'Grid/RangeMax': s('4.0'),
        'Grid/NoiseFilteringRadius': s('0.05'),
        'Grid/NoiseFilteringMinNeighbors': s('2'),

        # --- loop closure ---------------------------------------------------
        # 1 Hz is RTAB-Map's own default and is right: map updates are not a
        # control loop, and this is the single largest CPU knob in the node.
        'Rtabmap/DetectionRate': s('1'),
        'RGBD/ProximityBySpace': s('true'),
        'RGBD/AngularUpdate': s('0.05'),
        'RGBD/LinearUpdate': s('0.05'),
        # Reject a loop closure whose implied correction is absurd. With good
        # odometry a genuine closure needs only a small correction, so a huge
        # one means the place was misrecognised - and one accepted false
        # closure folds the map in half irrecoverably.
        'RGBD/OptimizeMaxError': s('3.0'),
        'Vis/MinInliers': s('20'),

        # --- mapping vs localization ----------------------------------------
        # IncrementalMemory false stops new nodes being created: the robot
        # localises against the stored map instead of extending it.
        'Mem/IncrementalMemory': if_loc('false', 'true'),
        # Load the WHOLE map into working memory at startup. Without this the
        # robot can only recognise places near where it thinks it already is,
        # so a robot switched on somewhere unexpected never localises - it just
        # sits there reporting no correction, which looks like a broken map.
        'Mem/InitWMWithAllNodes': if_loc('true', 'false'),
    }]

    remaps = [
        ('rgb/image', '/camera/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/camera/color/camera_info'),
        ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
        ('odom', '/odometry/filtered'),
    ]

    # --delete_db_on_start is a command-line ARGUMENT, not a parameter, and
    # arguments cannot be made conditional within one Node. Hence two
    # declarations sharing one params/remaps definition; exactly one ever runs.
    rtabmap_fresh = Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap',
        output='screen',
        condition=IfCondition(PythonExpression(
            ["'", delete_db, "' == 'true' and '", localization, "' != 'true'"])),
        parameters=params, remappings=remaps,
        arguments=['--delete_db_on_start'],
    )
    rtabmap_continue = Node(
        package='rtabmap_slam', executable='rtabmap', name='rtabmap',
        output='screen',
        condition=IfCondition(PythonExpression(
            ["'", delete_db, "' != 'true' or '", localization, "' == 'true'"])),
        parameters=params, remappings=remaps,
    )

    return LaunchDescription([
        declare_delete_db,
        declare_database_path,
        declare_localization,
        declare_grid_3d,
        rtabmap_fresh,
        rtabmap_continue,
    ])
