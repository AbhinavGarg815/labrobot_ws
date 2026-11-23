# RTAB-Map RGB-D SLAM (M3.T2).
#   ros2 launch labrobot_bringup mapping.launch.py

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

    # Every RTAB-Map config parameter is declared as a STRING.
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

        'Reg/Force3DoF': s('true'),
        'Optimizer/Slam2D': s('true'),

        # 2D occupancy grid for Nav2
        'Grid/FromDepth': s('true'),
        'Grid/3D': ParameterValue(grid_3d, value_type=str),
        'Grid/CellSize': s('0.05'),
        # Ray tracing marks the space BETWEEN the camera and an obstacle as free.
        'Grid/RayTracing': s('true'),
        'Grid/MaxGroundHeight': s('0.05'),
        'Grid/MaxObstacleHeight': s('0.90'),
        'Grid/RangeMin': s('0.30'),
        'Grid/RangeMax': s('4.0'),
        'Grid/NoiseFilteringRadius': s('0.05'),
        'Grid/NoiseFilteringMinNeighbors': s('2'),

        'Rtabmap/DetectionRate': s('1'),
        'RGBD/ProximityBySpace': s('true'),
        'RGBD/AngularUpdate': s('0.05'),
        'RGBD/LinearUpdate': s('0.05'),
        # Reject a loop closure whose implied correction is absurd.
        'RGBD/OptimizeMaxError': s('3.0'),
        'Vis/MinInliers': s('20'),
        'RGBD/LoopClosureReextractFeatures': s('true'),
        # 1000 visual words stored per node, not the default 500.
        'Kp/MaxFeatures': s('1000'),
        # Journal on disk + synchronous writes: the map survives a power cut.
        'DbSqlite3/JournalMode': s('1'),
        'DbSqlite3/Synchronous': s('2'),

        'Mem/IncrementalMemory': if_loc('false', 'true'),
        # Load the WHOLE map into working memory at startup.
        'Mem/InitWMWithAllNodes': if_loc('true', 'false'),
    }]

    remaps = [
        ('rgb/image', '/camera/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/camera/color/camera_info'),
        ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
        ('odom', '/odometry/filtered'),
    ]

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
