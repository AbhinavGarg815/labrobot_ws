# Remote visualisation bridge for Foxglove running on your own PC.
#
#   ros2 launch labrobot_bringup foxglove.launch.py
#
# Remote access is over SSH (no VPN). From your PC, tunnel the bridge port:
#
#   ssh -N -L 8765:localhost:8765 zine@<jetson-ip>
#
# then in Foxglove: Open connection -> Foxglove WebSocket -> ws://localhost:8765
#
# The bridge listens on 127.0.0.1 only, so it is reachable solely through that
# tunnel: the robot's topics and its teleop/goal publishing are not open to
# everyone on the network, and SSH provides the authentication and encryption.
# address:=0.0.0.0 opens it on every interface again (e.g. on a private bench
# network).
#
# Why this rather than RViz
# -------------------------
# RViz needs either X forwarding (unusable over SSH) or a matching ROS_DOMAIN_ID
# on your PC. The second sounds tidier but means DDS discovery and raw sample
# traffic across WiFi/VPN: multicast discovery is unreliable over both, and a
# 640x480 point cloud at 15 Hz is ~110 MB/s of UDP that will simply not fit.
#
# foxglove_bridge is one TCP WebSocket. It subscribes to a topic ONLY while a
# panel is open on it, so an idle connection costs nothing and the expensive
# topics cost only while you are actually looking at them.
#
# Bandwidth, in the order it will bite you
# ----------------------------------------
#   /camera/camera/depth/color/points      ~110 MB/s  only open this briefly
#   /camera/camera/color/image_raw          ~18 MB/s  NEVER open this over WiFi
#   /camera/camera/color/image_raw/compressed ~0.5 MB/s  <-- open THIS instead
#   /rtabmap/cloud_map                       bursty   the assembled map; grows
#   /map (OccupancyGrid)                     small    what you actually want
#   /tf, odometry                            tiny
#
# For "is the map right?" open /map and /tf. Reach for the live point cloud only
# to check what the camera can currently see - it is by far the most expensive
# thing on the robot to stream.
#
# Watching live video over WiFi
# -----------------------------
# Open the COMPRESSED topic, never image_raw. 848x480 BGR8 at 15 fps is
# 848*480*3*15 = 18.3 MB/s = 146 Mbit/s, which does not fit on WiFi at all; the
# same stream as JPEG is ~35 kB a frame, about 4 Mbit/s. Foxglove's Image panel
# renders sensor_msgs/CompressedImage directly, so nothing has to be decoded on
# the robot.
#
# There is no H.264 option on this machine. The Jetson Orin Nano has no NVENC -
# NVIDIA removed the hardware video encoder on this SKU (Orin NX and AGX Orin
# keep it), confirmed by the absence of /dev/nvhost-msenc. Software x264 would
# have to compete with visual odometry for CPU, so JPEG is the right trade: a
# little more bandwidth, no CPU contention, and no inter-frame latency.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Everything except raw images and the raw point cloud (see the bridge comment).
WIFI_SAFE_TOPICS = ['^(?!.*(/image_raw|/image_rect_raw|/points)$).*$']


def generate_launch_description():
    port = LaunchConfiguration('port')
    address = LaunchConfiguration('address')
    heavy = LaunchConfiguration('heavy')

    declare_port = DeclareLaunchArgument('port', default_value='8765')
    declare_heavy = DeclareLaunchArgument(
        'heavy', default_value='false',
        description='true: also offer raw images and the raw point cloud. Only on '
                    'a wired link and never while Nav2 is driving - see below.')
    declare_address = DeclareLaunchArgument(
        'address', default_value='127.0.0.1',
        description=('127.0.0.1: reachable only through an SSH tunnel '
                     '(ssh -L 8765:localhost:8765 ...). 0.0.0.0 listens on every '
                     'interface - no authentication, use only on a trusted network.'),
    )

    def bridge(whitelist, condition):
        return Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{
                'port': port,
                'address': address,
                # permessage-deflate, kept ON even though JPEG will not shrink.
                # It is a per-connection setting, not per-topic, so the choice is
                # made by whichever topic dominates. An occupancy grid for this lab
                # at 5 cm is megabytes of mostly-identical cells and deflates by
                # ~50x; a 35 kB JPEG costs well under a millisecond to attempt and
                # comes out the same size. Keeping it on is clearly the right side
                # of that trade.
                'use_compression': True,
                # THE LATENCY KNOB. When a connection's send buffer reaches this
                # limit the bridge drops messages "to avoid a queue of outdated
                # messages building up" - so the limit is really "how far behind
                # live am I willing to fall before frames start being dropped".
                #
                # The old 10 MB holds ~280 JPEG frames: on a congested link you end
                # up watching nearly twenty seconds into the past while teleoperating
                # a moving robot. 1 MB is ~28 frames, and in practice the buffer only
                # fills at all when WiFi is struggling - on a healthy link it sits
                # near empty and this value never bites.
                #
                # Raise it if you would rather have complete map updates than fresh
                # video; lower it if video still feels behind.
                'send_buffer_limit': 1000000,
                # Subscribe shallow. A deep QoS queue is latency by another name:
                # for live viewing the newest frame is the only one worth sending,
                # and anything queued behind it is already stale.
                'max_qos_depth': 2,
                'num_threads': 0,          # 0 = hardware concurrency
                # NOT everything. By default the raw images and the raw point cloud
                # are hidden, because opening one over WiFi starves the robot.
                #
                # Measured 2026-10-09: a remote Foxglove session that had opened
                # aligned_depth_to_color/image_raw drove this bridge to 98% CPU and
                # kept it there after the panel was closed ("outbox ... full"). The
                # camera driver lost the CPU race and the point cloud fell from
                # 10.5 Hz to 2.2 Hz, with 1.3-1.8 s gaps - so the costmaps went
                # stale 38 times a minute and Nav2 (correctly) refused to drive.
                # Bridge stopped: cloud back to 7.3-10.5 Hz, worst gap 269 ms.
                #
                # The compressed colour stream stays available (3 Mbit/s); the
                # costmaps, map, plan, TF and odometry show what Nav2 sees.
                # heavy:=true restores the full list for wired, non-driving use.
                'topic_whitelist': whitelist,
                # Lets Foxglove call services and set parameters, which is how you
                # trigger a map save without an SSH session.
                'capabilities': ['clientPublish', 'parameters', 'parametersSubscribe',
                                 'services', 'connectionGraph', 'assets'],
            }],
            condition=condition,
        )

    return LaunchDescription([
        declare_port,
        declare_address,
        declare_heavy,
        bridge(WIFI_SAFE_TOPICS, UnlessCondition(heavy)),
        bridge(['.*'], IfCondition(heavy)),
    ])
