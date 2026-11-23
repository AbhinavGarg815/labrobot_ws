# Bring up the Intel RealSense D435i for RGB-D SLAM.
#   ros2 launch labrobot_bringup camera.launch.py

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# librealsense built with FORCE_RSUSB_BACKEND.
RSUSB_LIB = os.path.expanduser('~/opt/librealsense-rsusb/lib')


def generate_launch_description():
    publish_tf = LaunchConfiguration('camera_publish_tf')
    fps = LaunchConfiguration('fps')
    pointcloud = LaunchConfiguration('pointcloud')
    decimation = LaunchConfiguration('decimation')
    align_depth = LaunchConfiguration('align_depth')
    enable_imu = LaunchConfiguration('enable_imu')
    jpeg_quality = LaunchConfiguration('jpeg_quality')

    declare_publish_tf = DeclareLaunchArgument(
        'camera_publish_tf',
        default_value='true',
        description=(
            'Let the camera publish its own internal TF. MUST stay true: the URDF '
            'deliberately stops at camera_link and does NOT define the depth / '
            'colour / IMU frames, because only the camera knows this unit\'s '
            'factory calibration (measured: 0.98 deg of depth-to-colour roll). '
            'With this false, those frames simply do not exist and anything '
            'consuming the images cannot place them.'
        ),
    )

    # 15 fps: visual odometry keeps up with every frame (it managed ~7 of 30).
    declare_fps = DeclareLaunchArgument(
        'fps', default_value='15',
        description='Frame rate for BOTH depth and colour. 30 wastes CPU that '
                    'visual odometry needs; 15 is well inside what the robot\'s '
                    '0.26 m/s and rotation limits require.',
    )

    declare_pointcloud = DeclareLaunchArgument(
        'pointcloud', default_value='false',
        description='Publish depth/color/points. For visualisation only; costs '
                    'real CPU and bandwidth.',
    )

    declare_decimation = DeclareLaunchArgument(
        'decimation', default_value='1',
        description='RealSense decimation filter magnitude (1 = off). Use 3 when '
                    'publishing a point cloud for the Nav2 costmap.',
    )

    declare_jpeg_quality = DeclareLaunchArgument(
        'jpeg_quality', default_value='60',
        description='JPEG quality 1-100 for the /compressed image topics. '
                    'Only applies at launch, not via ros2 param set.')

    declare_align_depth = DeclareLaunchArgument(
        'align_depth', default_value='true',
        description='Align depth into the colour frame. RTAB-Map RGB-D needs this.',
    )

    declare_enable_imu = DeclareLaunchArgument(
        'enable_imu', default_value='true',
        description='Enable gyro + accel. Requires the RSUSB librealsense build.',
    )

    camera = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='camera',
        name='camera',
        output='screen',
        additional_env={'LD_LIBRARY_PATH': RSUSB_LIB + ':' + os.environ.get('LD_LIBRARY_PATH', '')},
        parameters=[{
            'depth_module.depth_profile': PythonExpression(
                ["'848x480x' + str(", fps, ")"]),
            'rgb_camera.color_profile': PythonExpression(
                ["'640x480x' + str(", fps, ")"]),

            'enable_depth': True,
            'enable_color': True,
            'enable_infra1': False,
            'enable_infra2': False,

            'align_depth.enable': align_depth,

            'enable_gyro': enable_imu,
            'enable_accel': enable_imu,
            'unite_imu_method': 2,

            # Both names: on ARM the filter is 'Pointcloud (NEON)'.
            'pointcloud.enable': pointcloud,
            'pointcloud__neon_.enable': pointcloud,
            'decimation_filter.enable': PythonExpression(
                ["'", decimation, "' != '1'"]),
            'decimation_filter.filter_magnitude': ParameterValue(
                decimation, value_type=int),
            # Leading dot: compressed_image_transport's name for /camera/... topics.
            '.camera.color.image_raw.jpeg_quality': ParameterValue(
                jpeg_quality, value_type=int),

            'publish_tf': publish_tf,
        }],
    )

    return LaunchDescription([
        declare_publish_tf,
        declare_fps,
        declare_pointcloud,
        declare_decimation,
        declare_jpeg_quality,
        declare_align_depth,
        declare_enable_imu,
        camera,
    ])
