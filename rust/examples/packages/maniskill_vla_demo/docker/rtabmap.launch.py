"""ROS2 launch file for RTAB-Map RGB-D SLAM.

Launches rtabmap_slam/rtabmap with topic remappings matching the
rtabmap_bridge.py publisher topics.

Odometry source: ground-truth camera pose broadcast by rtabmap_bridge.py as
the TF chain  odom → camera_link.  RTAB-Map reads this TF for odometry and
performs visual loop-closure detection on top of it.

Published topics used by rtabmap_bridge.py:
  /rtabmap/cloud_map   (sensor_msgs/PointCloud2) — accumulated 3-D map
  /rtabmap/grid_map    (nav_msgs/OccupancyGrid)  — 2-D occupancy grid
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    remappings = [
        ("rgb/image",       "/camera/color/image_raw"),
        ("depth/image",     "/camera/depth/image_rect_raw"),
        ("rgb/camera_info", "/camera/color/camera_info"),
    ]

    rtabmap_params = {
        # ── Frame / TF ─────────────────────────────────────────────────────
        # camera_link is both the sensor frame and the tracking frame.
        # The bridge publishes odom → camera_link TF (ground-truth pose),
        # so RTAB-Map uses that for odometry without running visual odometry.
        "frame_id":                      "camera_link",
        "odom_frame_id":                 "odom",
        "subscribe_depth":               True,

        # ── Topic sync ─────────────────────────────────────────────────────
        # Use approximate time sync; the bridge stamps rgb/depth/camera_info
        # with the same ROS clock tick, so a small window is sufficient.
        "approx_sync":                   True,
        "approx_sync_max_interval":      0.2,
        "queue_size":                    20,

        # ── SLAM rate & memory ─────────────────────────────────────────────
        "Rtabmap/DetectionRate":         "2.0",  # Hz
        "Mem/STMSize":                   "30",   # short-term memory nodes
        "Mem/NotLinkedNodesKept":        "false",

        # ── Loop closure (visual) ──────────────────────────────────────────
        "RGBD/NeighborLinkRefining":     "true",
        "Reg/Strategy":                  "0",    # Vis (appearance-based)
        "Vis/MinInliers":                "10",

        # ── 3-D map (PointCloud2 → /rtabmap/cloud_map) ────────────────────
        # Zero thresholds: create a new keyframe on every detection cycle
        # regardless of camera motion.  Without this a static simulation camera
        # keeps WM=1 forever and cloud_map is never re-published after startup.
        "RGBD/LinearUpdate":             "0.0",
        "RGBD/AngularUpdate":            "0.0",
        # Force the cloud_map / grid_map topics to be republished on every
        # detection cycle.  Without this, RTAB-Map only publishes when a loop
        # closure or global map update occurs, which never happens in a short
        # static-camera simulation run → viz always sees an empty map.
        "map_always_update":             True,
        # Disable TRANSIENT_LOCAL (latched) publishing on map topics.
        # With latch=True, FastDDS delivers cached messages via a special code
        # path that competes with blocked executor threads in the bridge, causing
        # the callbacks to never fire.  With latch=False (VOLATILE) and
        # map_always_update=True, RTAB-Map re-publishes every 0.5 s so the
        # bridge receives fresh data as soon as a thread becomes free.
        "latch":                         False,

        # ── 2-D occupancy grid (OccupancyGrid → /rtabmap/grid_map) ────────
        # RTAB-Map publishes /rtabmap/grid_map automatically when these
        # Grid/* parameters are set.
        "Grid/FromDepth":                "true",
        "Grid/Sensor":                   "1",    # 1 = depth image (0 = laser scan, wrong for RGB-D)
        "Grid/CellSize":                 "0.05", # m/cell
        "Grid/RangeMax":                 "5.0",  # m
        "Grid/MaxGroundHeight":          "0.05", # m  — floor filter
        "Grid/MaxObstacleHeight":        "2.0",  # m
        "Grid/FootprintHeight":          "0.0",  # robot footprint not modelled
        "Grid/MapFrameProjection":       "false",
        # Publish the map on every RTAB-Map detection cycle (not only on
        # loop closures) so the bridge always receives fresh data.
        "Rtabmap/PublishLastSignature":  "true",
    }

    return LaunchDescription([
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            # namespace="rtabmap" is required so that relative topic names
            # (e.g. "cloud_map") resolve to /rtabmap/cloud_map.
            # Without it they resolve to /cloud_map (root namespace) and the
            # bridge subscriptions to /rtabmap/* never match.
            namespace="rtabmap",
            output="screen",
            parameters=[rtabmap_params],
            remappings=remappings,
        ),
    ])
