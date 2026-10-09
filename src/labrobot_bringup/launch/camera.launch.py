# Bring up the Intel RealSense D435i for RGB-D SLAM.
#
#   ros2 launch labrobot_bringup camera.launch.py
#
# Profiles are the ones measured on this Jetson (scripts/measure_camera_cost.sh),
# not guesses:
#
#   depth 848x480 + colour 640x480 @ 30            9.4% of one core
#   ... plus align_depth                          39.4%
#   ... plus IMU                                  66.1%
#
# All streams hold full rate at that setting, which leaves the rest of the
# 15 W budget for RTAB-Map.
#
# 848x480 is deliberate: it is the D435i depth sensor's native resolution, so it
# is the sharpest depth per CPU-cycle available. 1280x720 costs more and only
# upsamples the same underlying data.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# librealsense built with FORCE_RSUSB_BACKEND. The packaged build uses the V4L2
# backend, which reads the IMU from kernel IIO - and the hid-sensor modules that
# create those IIO entries do not exist for this JetPack kernel, so the IMU is
# invisible to it. Same soname, so this is selected purely by LD_LIBRARY_PATH and
# reverts by unsetting it. See scripts/02_build_librealsense_rsusb.sh.
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

    # 30 fps was chosen before anything consumed the stream. Measured since:
    # rgbd_odometry processes only ~7 frames/s on this Jetson, so at 30 fps three
    # quarters of every frame's capture, transport and deserialisation cost is
    # spent on data that is thrown away - while starving the consumer that needed
    # it. At 0.26 m/s the robot moves 17 mm between frames at 15 fps, which is
    # ample overlap for RGB-D odometry.
    declare_fps = DeclareLaunchArgument(
        'fps', default_value='15',
        description='Frame rate for BOTH depth and colour. 30 wastes CPU that '
                    'visual odometry needs; 15 is well inside what the robot\'s '
                    '0.26 m/s and rotation limits require.',
    )

    # Off by default: RTAB-Map builds its own cloud from depth + colour, so the
    # driver's would be computed twice and thrown away once. Turn it on ONLY to
    # look at the live cloud in Foxglove - it is ~110 MB/s and the single most
    # expensive thing this robot can publish.
    declare_pointcloud = DeclareLaunchArgument(
        'pointcloud', default_value='false',
        description='Publish depth/color/points. For visualisation only; costs '
                    'real CPU and bandwidth.',
    )

    # Decimation happens IN THE DRIVER, before the cloud is built, so it cuts the
    # point count, the serialisation and the costmap's work all at once. 640x480
    # is 307k points per frame; at 15 Hz that is 4.6M points/s, which no costmap
    # on this Jetson will keep up with. Magnitude 3 gives ~34k points/frame.
    declare_decimation = DeclareLaunchArgument(
        'decimation', default_value='1',
        description='RealSense decimation filter magnitude (1 = off). Use 3 when '
                    'publishing a point cloud for the Nav2 costmap.',
    )

    # JPEG quality for the /compressed topics, which is how the camera is
    # watched remotely over WiFi. Measured on this robot at 640x480x15:
    #
    #   raw image_raw             900 kB/frame   101.6 Mbit/s   unusable on WiFi
    #   compressed, default q95    82 kB/frame     9.3 Mbit/s
    #
    # MUST be set here, at launch. compressed_image_transport reads this
    # parameter once when the publisher is constructed and caches it, so
    # `ros2 param set` on a running node succeeds, reads back the new value, and
    # changes nothing - verified by sweeping q85/75/65/55/20 on a live node and
    # measuring 82.8/82.8/82.9/82.1/80.7 kB per frame, i.e. no effect at all.
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
        # Prepending is what makes the node pick our RSUSB build over the
        # packaged one; both export the same soname.
        additional_env={'LD_LIBRARY_PATH': RSUSB_LIB + ':' + os.environ.get('LD_LIBRARY_PATH', '')},
        parameters=[{
            'depth_module.depth_profile': PythonExpression(
                ["'848x480x' + str(", fps, ")"]),
            'rgb_camera.color_profile': PythonExpression(
                ["'640x480x' + str(", fps, ")"]),

            'enable_depth': True,
            'enable_color': True,
            # The IR streams are what stereo depth is computed FROM; we consume
            # the depth result, so publishing them is pure CPU and bandwidth.
            'enable_infra1': False,
            'enable_infra2': False,

            'align_depth.enable': align_depth,

            'enable_gyro': enable_imu,
            'enable_accel': enable_imu,
            # 2 = linear_interpolation: gyro and accel arrive at different rates
            # (199.5 and 100.5 Hz measured), and this interpolates them onto one
            # /imu topic. "copy" instead would hold the last accel sample and
            # emit a stale reading at every gyro tick.
            'unite_imu_method': 2,

            # BOTH names, deliberately. realsense2_camera derives this parameter
            # from the librealsense FILTER name, and on ARM that filter is
            # "Pointcloud (NEON)" -> pointcloud__neon_.enable. On x86 it is
            # "Pointcloud" -> pointcloud.enable. The node accepts the wrong one
            # silently ("Parameter not set") and simply never publishes a cloud,
            # which looks like the driver ignoring you. Setting both makes this
            # file work on either architecture.
            'pointcloud.enable': pointcloud,
            'pointcloud__neon_.enable': pointcloud,
            'decimation_filter.enable': PythonExpression(
                ["'", decimation, "' != '1'"]),
            'decimation_filter.filter_magnitude': ParameterValue(
                decimation, value_type=int),
            # Leading dot is not a typo. compressed_image_transport derives the
            # parameter name from the base topic with '/' replaced by '.', so
            # /camera/color/image_raw becomes .camera.color.image_raw and the
            # leading slash becomes a leading dot. Confirmed against
            # `ros2 param list /camera/camera` on the running node.
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
