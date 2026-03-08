# ROS 2 IDL diff report (progressive by distro order)

Distros are compared in **chronological release order** (each vs next): `foxy` → `humble` → `jazzy` → `rolling`.

## Summary
- Repos: `common_interfaces`, `geometry2`, `rcl_interfaces`, `control_msgs`
- Distros (order): `foxy`, `humble`, `jazzy`, `rolling`
- Total interfaces: 227

### Per repo

| Column | Meaning |
|--------|--------|
| **Interfaces** | Number of distinct definitions (e.g. `std_msgs/msg/Header` = 1). Same definition in several distros still counts as 1. |
| **Only in some distros** | Among the distros **fetched for this repo**, how many definitions exist in only part of them. For repos with fewer distros (e.g. only humble+jazzy), only those count—not the full list. |
| **Changed at some step** | Count of *steps* where a definition differs (each foxy→humble, humble→jazzy, jazzy→rolling). One interface can contribute more than once if it changes in multiple steps; so this number can be larger than Interfaces. |

| Repo | Interfaces | Only in some distros | Changed at some step |
|------|------------|----------------------|----------------------|
| `common_interfaces` | 135 | 13 | 19 |
| `control_msgs` | 36 | 11 | 12 |
| `geometry2` | 4 | 0 | 0 |
| `rcl_interfaces` | 52 | 12 | 15 |

---

## Interface table (one row per definition)

Distro columns: `✓ (n)` = present; `—` = absent in that distro; `∅` = branch not fetched for this repo.

| Interface | foxy | humble | jazzy | rolling | Changes (progressive) |
| --- | --- | --- | --- | --- | --- |
| `common_interfaces/actionlib_msgs/msg/GoalID` | ✓ (2) | ✓ (2) | ✓ (2) | — | `jazzy`→`rolling`: only in `jazzy` |
| `common_interfaces/actionlib_msgs/msg/GoalStatus` | ✓ (13) | ✓ (13) | ✓ (13) | — | `jazzy`→`rolling`: only in `jazzy` |
| `common_interfaces/actionlib_msgs/msg/GoalStatusArray` | ✓ (2) | ✓ (2) | ✓ (2) | — | `jazzy`→`rolling`: only in `jazzy` |
| `common_interfaces/diagnostic_msgs/msg/DiagnosticArray` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `common_interfaces/diagnostic_msgs/msg/DiagnosticStatus` | ✓ (9) | ✓ (9) | ✓ (9) | ✓ (9) |  |
| `common_interfaces/diagnostic_msgs/msg/KeyValue` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `common_interfaces/diagnostic_msgs/srv/AddDiagnostics` | ✓ (3) | ✓ (3) | ✓ (3) | ✓ (3) |  |
| `common_interfaces/diagnostic_msgs/srv/SelfTest` | ✓ (3) | ✓ (3) | ✓ (3) | ✓ (3) |  |
| `common_interfaces/geometry_msgs/msg/Accel` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `common_interfaces/geometry_msgs/msg/AccelStamped` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `common_interfaces/geometry_msgs/msg/AccelWithCovariance` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `common_interfaces/geometry_msgs/msg/AccelWithCovarianceStamped` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `common_interfaces/geometry_msgs/msg/Inertia` | ✓ (8) | ✓ (8) | ✓ (8) | ✓ (8) |  |
| `common_interfaces/geometry_msgs/msg/InertiaStamped` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `common_interfaces/geometry_msgs/msg/Point` | ✓ (3) | ✓ (3) | ✓ (3) | ✓ (3) |  |
| `common_interfaces/geometry_msgs/msg/Point32` | ✓ (3) | ✓ (3) | ✓ (3) | ✓ (3) |  |
| `common_interfaces/geometry_msgs/msg/PointStamped` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `common_interfaces/geometry_msgs/msg/Polygon` | ✓ (1) | ✓ (1) | ✓ (1) | ✓ (1) |  |
| `common_interfaces/geometry_msgs/msg/PolygonInstance` | — | ✓ (2) | ✓ (2) | ✓ (2) | `foxy`→`humble`: only in `humble` |
| `common_interfaces/geometry_msgs/msg/PolygonInstanceStamped` | — | ✓ (2) | ✓ (2) | ✓ (2) | `foxy`→`humble`: only in `humble` |
| `common_interfaces/geometry_msgs/msg/PolygonStamped` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `common_interfaces/geometry_msgs/msg/Pose` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `common_interfaces/geometry_msgs/msg/Pose2D` | ✓ (3) | ✓ (3) | ✓ (3) | — | `jazzy`→`rolling`: only in `jazzy` |
| `common_interfaces/geometry_msgs/msg/PoseArray` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `common_interfaces/geometry_msgs/msg/PoseStamped` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `common_interfaces/geometry_msgs/msg/PoseWithCovariance` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `common_interfaces/geometry_msgs/msg/PoseWithCovarianceStamped` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `common_interfaces/geometry_msgs/msg/Quaternion` | ✓ (4) | ✓ (4) | ✓ (4) | ✓ (4) |  |
| `common_interfaces/geometry_msgs/msg/QuaternionStamped` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `common_interfaces/geometry_msgs/msg/Transform` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `common_interfaces/geometry_msgs/msg/TransformStamped` | ✓ (3) | ✓ (3) | ✓ (3) | ✓ (3) |  |
| `common_interfaces/geometry_msgs/msg/Twist` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `common_interfaces/geometry_msgs/msg/TwistStamped` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `common_interfaces/geometry_msgs/msg/TwistWithCovariance` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `common_interfaces/geometry_msgs/msg/TwistWithCovarianceStamped` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `common_interfaces/geometry_msgs/msg/Vector3` | ✓ (3) | ✓ (3) | ✓ (3) | ✓ (3) |  |
| `common_interfaces/geometry_msgs/msg/Vector3Stamped` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `common_interfaces/geometry_msgs/msg/VelocityStamped` | — | ✓ (4) | ✓ (4) | ✓ (4) | `foxy`→`humble`: only in `humble` |
| `common_interfaces/geometry_msgs/msg/Wrench` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `common_interfaces/geometry_msgs/msg/WrenchStamped` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `common_interfaces/nav_msgs/msg/Goals` | — | ✓ (2) | ✓ (2) | ✓ (2) | `foxy`→`humble`: only in `humble` |
| `common_interfaces/nav_msgs/msg/GridCells` | ✓ (4) | ✓ (4) | ✓ (4) | ✓ (4) |  |
| `common_interfaces/nav_msgs/msg/MapMetaData` | ✓ (5) | ✓ (5) | ✓ (5) | ✓ (5) |  |
| `common_interfaces/nav_msgs/msg/OccupancyGrid` | ✓ (3) | ✓ (3) | ✓ (3) | ✓ (3) |  |
| `common_interfaces/nav_msgs/msg/Odometry` | ✓ (4) | ✓ (4) | ✓ (4) | ✓ (4) |  |
| `common_interfaces/nav_msgs/msg/Path` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `common_interfaces/nav_msgs/msg/Trajectory` | — | — | ✓ (2) | ✓ (2) | `humble`→`jazzy`: only in `jazzy` |
| `common_interfaces/nav_msgs/msg/TrajectoryPoint` | — | — | ✓ (5) | ✓ (5) | `humble`→`jazzy`: only in `jazzy` |
| `common_interfaces/nav_msgs/srv/GetMap` | ✓ (1) | ✓ (1) | ✓ (1) | ✓ (1) |  |
| `common_interfaces/nav_msgs/srv/GetPlan` | ✓ (4) | ✓ (4) | ✓ (4) | ✓ (4) |  |
| `common_interfaces/nav_msgs/srv/LoadMap` | — | ✓ (8) | ✓ (8) | ✓ (8) | `foxy`→`humble`: only in `humble` |
| `common_interfaces/nav_msgs/srv/SetMap` | ✓ (3) | ✓ (3) | ✓ (3) | ✓ (3) |  |
| `common_interfaces/sensor_msgs/msg/BatteryState` | ✓ (37) | ✓ (37) | ✓ (39) | ✓ (39) | `humble`→`jazzy`: ~ `=`: `uint8 POWER_SUPPLY_TECHNOLOGY_LIMN` → `uint8 POWER_SUPPLY_TECHNOLOGY_VRLA` |
| `common_interfaces/sensor_msgs/msg/CameraInfo` | ✓ (11) | ✓ (11) | ✓ (11) | ✓ (11) |  |
| `common_interfaces/sensor_msgs/msg/ChannelFloat32` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `common_interfaces/sensor_msgs/msg/CompressedImage` | ✓ (3) | ✓ (3) | ✓ (3) | ✓ (3) |  |
| `common_interfaces/sensor_msgs/msg/FluidPressure` | ✓ (3) | ✓ (3) | ✓ (3) | ✓ (3) |  |
| `common_interfaces/sensor_msgs/msg/Illuminance` | ✓ (3) | ✓ (3) | ✓ (3) | ✓ (3) |  |
| `common_interfaces/sensor_msgs/msg/Image` | ✓ (7) | ✓ (7) | ✓ (7) | ✓ (7) |  |
| `common_interfaces/sensor_msgs/msg/Imu` | ✓ (7) | ✓ (7) | ✓ (7) | ✓ (7) |  |
| `common_interfaces/sensor_msgs/msg/JointState` | ✓ (5) | ✓ (5) | ✓ (5) | ✓ (5) |  |
| `common_interfaces/sensor_msgs/msg/Joy` | ✓ (3) | ✓ (3) | ✓ (3) | ✓ (3) |  |
| `common_interfaces/sensor_msgs/msg/JoyFeedback` | ✓ (6) | ✓ (6) | ✓ (6) | ✓ (6) |  |
| `common_interfaces/sensor_msgs/msg/JoyFeedbackArray` | ✓ (1) | ✓ (1) | ✓ (1) | ✓ (1) |  |
| `common_interfaces/sensor_msgs/msg/LaserEcho` | ✓ (1) | ✓ (1) | ✓ (1) | ✓ (1) |  |
| `common_interfaces/sensor_msgs/msg/LaserScan` | ✓ (10) | ✓ (10) | ✓ (10) | ✓ (10) |  |
| `common_interfaces/sensor_msgs/msg/MagneticField` | ✓ (3) | ✓ (3) | ✓ (3) | ✓ (3) |  |
| `common_interfaces/sensor_msgs/msg/MultiDOFJointState` | ✓ (5) | ✓ (5) | ✓ (5) | ✓ (5) |  |
| `common_interfaces/sensor_msgs/msg/MultiEchoLaserScan` | ✓ (10) | ✓ (10) | ✓ (10) | ✓ (10) |  |
| `common_interfaces/sensor_msgs/msg/NavSatFix` | ✓ (11) | ✓ (11) | ✓ (11) | ✓ (11) |  |
| `common_interfaces/sensor_msgs/msg/NavSatStatus` | ✓ (10) | ✓ (10) | ✓ (12) | ✓ (12) |  |
| `common_interfaces/sensor_msgs/msg/PointCloud` | ✓ (3) | ✓ (3) | ✓ (3) | ✓ (3) |  |
| `common_interfaces/sensor_msgs/msg/PointCloud2` | ✓ (9) | ✓ (9) | ✓ (9) | ✓ (9) |  |
| `common_interfaces/sensor_msgs/msg/PointField` | ✓ (12) | ✓ (12) | ✓ (12) | ✓ (15) | `jazzy`→`rolling`: ~ `=`: `uint8 FLOAT64` → `uint8 BOOL` |
| `common_interfaces/sensor_msgs/msg/Range` | ✓ (8) | ✓ (8) | ✓ (9) | ✓ (9) | `humble`→`jazzy`: + `(message)`: `float32` `variance` |
| `common_interfaces/sensor_msgs/msg/RegionOfInterest` | ✓ (5) | ✓ (5) | ✓ (5) | ✓ (5) |  |
| `common_interfaces/sensor_msgs/msg/RelativeHumidity` | ✓ (3) | ✓ (3) | ✓ (3) | ✓ (3) |  |
| `common_interfaces/sensor_msgs/msg/Temperature` | ✓ (3) | ✓ (3) | ✓ (3) | ✓ (3) |  |
| `common_interfaces/sensor_msgs/msg/TimeReference` | ✓ (3) | ✓ (3) | ✓ (3) | ✓ (3) |  |
| `common_interfaces/sensor_msgs/srv/SetCameraInfo` | ✓ (3) | ✓ (3) | ✓ (3) | ✓ (3) |  |
| `common_interfaces/shape_msgs/msg/Mesh` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `common_interfaces/shape_msgs/msg/MeshTriangle` | ✓ (1) | ✓ (1) | ✓ (1) | ✓ (1) |  |
| `common_interfaces/shape_msgs/msg/Plane` | ✓ (1) | ✓ (1) | ✓ (1) | ✓ (1) |  |
| `common_interfaces/shape_msgs/msg/SolidPrimitive` | ✓ (14) | ✓ (17) | ✓ (17) | ✓ (17) | `foxy`→`humble`: + `(message)`: `uint8` `PRISM=5`<br>+ `(message)`: `uint8` `PRISM_HEIGHT=0`<br>+ `(message)`: `geometry_msgs/Polygon` `polygon` |
| `common_interfaces/std_msgs/msg/Bool` | ✓ (1) | ✓ (1) | ✓ (1) | ✓ (1) |  |
| `common_interfaces/std_msgs/msg/Byte` | ✓ (1) | ✓ (1) | ✓ (1) | ✓ (1) |  |
| `common_interfaces/std_msgs/msg/ByteMultiArray` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `common_interfaces/std_msgs/msg/Char` | ✓ (1) | ✓ (1) | ✓ (1) | ✓ (1) |  |
| `common_interfaces/std_msgs/msg/ColorRGBA` | ✓ (4) | ✓ (4) | ✓ (4) | ✓ (4) |  |
| `common_interfaces/std_msgs/msg/Empty` | ✓ | ✓ | ✓ | ✓ |  |
| `common_interfaces/std_msgs/msg/Float32` | ✓ (1) | ✓ (1) | ✓ (1) | ✓ (1) |  |
| `common_interfaces/std_msgs/msg/Float32MultiArray` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `common_interfaces/std_msgs/msg/Float64` | ✓ (1) | ✓ (1) | ✓ (1) | ✓ (1) |  |
| `common_interfaces/std_msgs/msg/Float64MultiArray` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `common_interfaces/std_msgs/msg/Header` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `common_interfaces/std_msgs/msg/Int16` | ✓ (1) | ✓ (1) | ✓ (1) | ✓ (1) |  |
| `common_interfaces/std_msgs/msg/Int16MultiArray` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `common_interfaces/std_msgs/msg/Int32` | ✓ (1) | ✓ (1) | ✓ (1) | ✓ (1) |  |
| `common_interfaces/std_msgs/msg/Int32MultiArray` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `common_interfaces/std_msgs/msg/Int64` | ✓ (1) | ✓ (1) | ✓ (1) | ✓ (1) |  |
| `common_interfaces/std_msgs/msg/Int64MultiArray` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `common_interfaces/std_msgs/msg/Int8` | ✓ (1) | ✓ (1) | ✓ (1) | ✓ (1) |  |
| `common_interfaces/std_msgs/msg/Int8MultiArray` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `common_interfaces/std_msgs/msg/MultiArrayDimension` | ✓ (3) | ✓ (3) | ✓ (3) | ✓ (3) |  |
| `common_interfaces/std_msgs/msg/MultiArrayLayout` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `common_interfaces/std_msgs/msg/String` | ✓ (1) | ✓ (1) | ✓ (1) | ✓ (1) |  |
| `common_interfaces/std_msgs/msg/UInt16` | ✓ (1) | ✓ (1) | ✓ (1) | ✓ (1) |  |
| `common_interfaces/std_msgs/msg/UInt16MultiArray` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `common_interfaces/std_msgs/msg/UInt32` | ✓ (1) | ✓ (1) | ✓ (1) | ✓ (1) |  |
| `common_interfaces/std_msgs/msg/UInt32MultiArray` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `common_interfaces/std_msgs/msg/UInt64` | ✓ (1) | ✓ (1) | ✓ (1) | ✓ (1) |  |
| `common_interfaces/std_msgs/msg/UInt64MultiArray` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `common_interfaces/std_msgs/msg/UInt8` | ✓ (1) | ✓ (1) | ✓ (1) | ✓ (1) |  |
| `common_interfaces/std_msgs/msg/UInt8MultiArray` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `common_interfaces/std_srvs/srv/Empty` | ✓ | ✓ | ✓ | ✓ |  |
| `common_interfaces/std_srvs/srv/SetBool` | ✓ (3) | ✓ (3) | ✓ (3) | ✓ (3) |  |
| `common_interfaces/std_srvs/srv/Trigger` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `common_interfaces/stereo_msgs/msg/DisparityImage` | ✓ (8) | ✓ (8) | ✓ (8) | ✓ (8) |  |
| `common_interfaces/trajectory_msgs/msg/JointTrajectory` | ✓ (3) | ✓ (3) | ✓ (3) | ✓ (3) |  |
| `common_interfaces/trajectory_msgs/msg/JointTrajectoryPoint` | ✓ (5) | ✓ (5) | ✓ (5) | ✓ (5) |  |
| `common_interfaces/trajectory_msgs/msg/MultiDOFJointTrajectory` | ✓ (3) | ✓ (3) | ✓ (3) | ✓ (3) |  |
| `common_interfaces/trajectory_msgs/msg/MultiDOFJointTrajectoryPoint` | ✓ (4) | ✓ (4) | ✓ (4) | ✓ (4) |  |
| `common_interfaces/visualization_msgs/msg/ImageMarker` | ✓ (20) | ✓ (20) | ✓ (20) | ✓ (20) |  |
| `common_interfaces/visualization_msgs/msg/InteractiveMarker` | ✓ (7) | ✓ (7) | ✓ (7) | ✓ (7) |  |
| `common_interfaces/visualization_msgs/msg/InteractiveMarkerControl` | ✓ (21) | ✓ (21) | ✓ (21) | ✓ (21) |  |
| `common_interfaces/visualization_msgs/msg/InteractiveMarkerFeedback` | ✓ (15) | ✓ (15) | ✓ (15) | ✓ (15) |  |
| `common_interfaces/visualization_msgs/msg/InteractiveMarkerInit` | ✓ (3) | ✓ (3) | ✓ (3) | ✓ (3) |  |
| `common_interfaces/visualization_msgs/msg/InteractiveMarkerPose` | ✓ (3) | ✓ (3) | ✓ (3) | ✓ (3) |  |
| `common_interfaces/visualization_msgs/msg/InteractiveMarkerUpdate` | ✓ (8) | ✓ (8) | ✓ (8) | ✓ (8) |  |
| `common_interfaces/visualization_msgs/msg/Marker` | ✓ (31) | ✓ (35) | ✓ (36) | ✓ (36) | `foxy`→`humble`: + `(message)`: `MeshFile` `mesh_file`<br>+ `(message)`: `sensor_msgs/CompressedImage` `texture`<br>+ `(message)`: `string` `texture_resource`<br>+ `(message)`: `UVCoordinate[]` `uv_coordinates`<br><br>`humble`→`jazzy`: + `(message)`: `int32` `ARROW_STRIP=12` |
| `common_interfaces/visualization_msgs/msg/MarkerArray` | ✓ (1) | ✓ (1) | ✓ (1) | ✓ (1) |  |
| `common_interfaces/visualization_msgs/msg/MenuEntry` | ✓ (8) | ✓ (8) | ✓ (8) | ✓ (8) |  |
| `common_interfaces/visualization_msgs/msg/MeshFile` | — | ✓ (2) | ✓ (2) | ✓ (2) | `foxy`→`humble`: only in `humble` |
| `common_interfaces/visualization_msgs/msg/UVCoordinate` | — | ✓ (2) | ✓ (2) | ✓ (2) | `foxy`→`humble`: only in `humble` |
| `common_interfaces/visualization_msgs/srv/GetInteractiveMarkers` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `control_msgs/control_msgs/action/ExecuteMotionPrimitiveSequence` | ∅ | — | ✓ (7) | ∅ | `humble`→`jazzy`: only in `jazzy` |
| `control_msgs/control_msgs/action/FollowJointTrajectory` | ∅ | ✓ (24) | ✓ (24) | ∅ |  |
| `control_msgs/control_msgs/action/GripperCommand` | ∅ | ✓ (9) | ✓ (9) | ∅ |  |
| `control_msgs/control_msgs/action/JointTrajectory` | ∅ | ✓ (1) | ✓ (1) | ∅ |  |
| `control_msgs/control_msgs/action/ParallelGripperCommand` | ∅ | ✓ (5) | ✓ (5) | ∅ |  |
| `control_msgs/control_msgs/action/PointHead` | ∅ | ✓ (6) | ✓ (6) | ∅ |  |
| `control_msgs/control_msgs/action/SingleJointPosition` | ∅ | ✓ (7) | ✓ (7) | ∅ |  |
| `control_msgs/control_msgs/msg/AdmittanceControllerState` | ∅ | ✓ (12) | ✓ (12) | ∅ |  |
| `control_msgs/control_msgs/msg/BatteryStateArray` | ∅ | — | ✓ (1) | ∅ | `humble`→`jazzy`: only in `jazzy` |
| `control_msgs/control_msgs/msg/DynamicInterfaceGroupValues` | ∅ | ✓ (3) | ✓ (3) | ∅ |  |
| `control_msgs/control_msgs/msg/DynamicInterfaceValues` | ∅ | — | ✓ (3) | ∅ | `humble`→`jazzy`: only in `jazzy` |
| `control_msgs/control_msgs/msg/DynamicJointState` | ∅ | ✓ (3) | ✓ (3) | ∅ |  |
| `control_msgs/control_msgs/msg/Float64Values` | ∅ | — | ✓ (2) | ∅ | `humble`→`jazzy`: only in `jazzy` |
| `control_msgs/control_msgs/msg/GripperCommand` | ∅ | ✓ (2) | ✓ (2) | ∅ |  |
| `control_msgs/control_msgs/msg/InterfaceValue` | ∅ | ✓ (2) | ✓ (2) | ∅ |  |
| `control_msgs/control_msgs/msg/JointComponentTolerance` | ∅ | ✓ (10) | ✓ (10) | ∅ |  |
| `control_msgs/control_msgs/msg/JointControllerState` | ∅ | ✓ (12) | ✓ (12) | ∅ |  |
| `control_msgs/control_msgs/msg/JointJog` | ∅ | ✓ (5) | ✓ (5) | ∅ |  |
| `control_msgs/control_msgs/msg/JointTolerance` | ∅ | ✓ (4) | ✓ (4) | ∅ |  |
| `control_msgs/control_msgs/msg/JointTrajectoryControllerState` | ∅ | ✓ (15) | ✓ (12) | ∅ | `humble`→`jazzy`: + `(message)`: `float64` `speed_scaling_factor`<br>- `(message)`: `trajectory_msgs/JointTrajectoryPoint` `actual`<br>- `(message)`: `trajectory_msgs/JointTrajectoryPoint` `desired`<br>- `(message)`: `trajectory_msgs/MultiDOFJointTrajectoryPoint` `multi_dof_actual`<br>- `(message)`: `trajectory_msgs/MultiDOFJointTrajectoryPoint` `multi_dof_desired` |
| `control_msgs/control_msgs/msg/Keys` | ∅ | — | ✓ (2) | ∅ | `humble`→`jazzy`: only in `jazzy` |
| `control_msgs/control_msgs/msg/MecanumDriveControllerState` | ∅ | ✓ (6) | ✓ (6) | ∅ |  |
| `control_msgs/control_msgs/msg/MotionArgument` | ∅ | — | ✓ (2) | ∅ | `humble`→`jazzy`: only in `jazzy` |
| `control_msgs/control_msgs/msg/MotionPrimitive` | ∅ | — | ✓ (9) | ∅ | `humble`→`jazzy`: only in `jazzy` |
| `control_msgs/control_msgs/msg/MotionPrimitiveSequence` | ∅ | — | ✓ (1) | ∅ | `humble`→`jazzy`: only in `jazzy` |
| `control_msgs/control_msgs/msg/MultiDOFCommand` | ∅ | ✓ (3) | ✓ (3) | ∅ |  |
| `control_msgs/control_msgs/msg/MultiDOFStateStamped` | ∅ | ✓ (2) | ✓ (2) | ∅ |  |
| `control_msgs/control_msgs/msg/PidState` | ∅ | ✓ (13) | ✓ (13) | ∅ |  |
| `control_msgs/control_msgs/msg/SingleDOFState` | ∅ | ✓ (8) | ✓ (8) | ∅ |  |
| `control_msgs/control_msgs/msg/SingleDOFStateStamped` | ∅ | ✓ (2) | ✓ (2) | ∅ |  |
| `control_msgs/control_msgs/msg/SpeedScalingFactor` | ∅ | — | ✓ (1) | ∅ | `humble`→`jazzy`: only in `jazzy` |
| `control_msgs/control_msgs/msg/SteeringControllerStatus` | ∅ | ✓ (6) | ✓ (6) | ∅ |  |
| `control_msgs/control_msgs/msg/VDA5050SafetyState` | ∅ | — | ✓ (6) | ∅ | `humble`→`jazzy`: only in `jazzy` |
| `control_msgs/control_msgs/srv/QueryCalibrationState` | ∅ | ✓ (1) | ✓ (1) | ∅ |  |
| `control_msgs/control_msgs/srv/QueryTrajectoryState` | ∅ | ✓ (7) | ✓ (7) | ∅ |  |
| `control_msgs/control_msgs/srv/SetOdometry` | ∅ | — | ✓ (8) | ∅ | `humble`→`jazzy`: only in `jazzy` |
| `geometry2/tf2_msgs/action/LookupTransform` | ✓ (9) | ✓ (9) | ✓ (9) | ✓ (9) |  |
| `geometry2/tf2_msgs/msg/TF2Error` | ✓ (9) | ✓ (9) | ✓ (9) | ✓ (9) |  |
| `geometry2/tf2_msgs/msg/TFMessage` | ✓ (1) | ✓ (1) | ✓ (1) | ✓ (1) |  |
| `geometry2/tf2_msgs/srv/FrameGraph` | ✓ (1) | ✓ (1) | ✓ (1) | ✓ (1) |  |
| `rcl_interfaces/action_msgs/msg/GoalInfo` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `rcl_interfaces/action_msgs/msg/GoalStatus` | ✓ (9) | ✓ (9) | ✓ (9) | ✓ (9) |  |
| `rcl_interfaces/action_msgs/msg/GoalStatusArray` | ✓ (1) | ✓ (1) | ✓ (1) | ✓ (1) |  |
| `rcl_interfaces/action_msgs/srv/CancelGoal` | ✓ (7) | ✓ (7) | ✓ (7) | ✓ (7) |  |
| `rcl_interfaces/builtin_interfaces/msg/Duration` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `rcl_interfaces/builtin_interfaces/msg/Time` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `rcl_interfaces/composition_interfaces/srv/ListNodes` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `rcl_interfaces/composition_interfaces/srv/LoadNode` | ✓ (12) | ✓ (12) | ✓ (12) | ✓ (12) |  |
| `rcl_interfaces/composition_interfaces/srv/UnloadNode` | ✓ (3) | ✓ (3) | ✓ (3) | ✓ (3) |  |
| `rcl_interfaces/lifecycle_msgs/msg/State` | ✓ (13) | ✓ (13) | ✓ (13) | ✓ (13) |  |
| `rcl_interfaces/lifecycle_msgs/msg/Transition` | ✓ (32) | ✓ (32) | ✓ (32) | ✓ (32) |  |
| `rcl_interfaces/lifecycle_msgs/msg/TransitionDescription` | ✓ (3) | ✓ (3) | ✓ (3) | ✓ (3) |  |
| `rcl_interfaces/lifecycle_msgs/msg/TransitionEvent` | ✓ (4) | ✓ (4) | ✓ (4) | ✓ (4) | `jazzy`→`rolling`: + `(message)`: `builtin_interfaces/Time` `stamp`<br>- `(message)`: `uint64` `timestamp` |
| `rcl_interfaces/lifecycle_msgs/srv/ChangeState` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `rcl_interfaces/lifecycle_msgs/srv/GetAvailableStates` | ✓ (1) | ✓ (1) | ✓ (1) | ✓ (1) |  |
| `rcl_interfaces/lifecycle_msgs/srv/GetAvailableTransitions` | ✓ (1) | ✓ (1) | ✓ (1) | ✓ (1) |  |
| `rcl_interfaces/lifecycle_msgs/srv/GetState` | ✓ (1) | ✓ (1) | ✓ (1) | ✓ (1) |  |
| `rcl_interfaces/rcl_interfaces/msg/FloatingPointRange` | ✓ (3) | ✓ (3) | ✓ (3) | ✓ (3) |  |
| `rcl_interfaces/rcl_interfaces/msg/IntegerRange` | ✓ (3) | ✓ (3) | ✓ (3) | ✓ (3) |  |
| `rcl_interfaces/rcl_interfaces/msg/ListParametersResult` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `rcl_interfaces/rcl_interfaces/msg/Log` | ✓ (12) | ✓ (12) | ✓ (12) | ✓ (12) | `humble`→`jazzy`: ~ `DEBUG=10`: `byte` → `uint8`<br>~ `ERROR=40`: `byte` → `uint8`<br>~ `FATAL=50`: `byte` → `uint8`<br>~ `INFO=20`: `byte` → `uint8`<br>~ `WARN=30`: `byte` → `uint8` |
| `rcl_interfaces/rcl_interfaces/msg/LoggerLevel` | — | — | ✓ (8) | ✓ (8) | `humble`→`jazzy`: only in `jazzy` |
| `rcl_interfaces/rcl_interfaces/msg/Parameter` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `rcl_interfaces/rcl_interfaces/msg/ParameterDescriptor` | ✓ (7) | ✓ (8) | ✓ (8) | ✓ (8) | `foxy`→`humble`: + `(message)`: `bool` `dynamic_typing` |
| `rcl_interfaces/rcl_interfaces/msg/ParameterEvent` | ✓ (5) | ✓ (5) | ✓ (5) | ✓ (5) |  |
| `rcl_interfaces/rcl_interfaces/msg/ParameterEventDescriptors` | ✓ (3) | ✓ (3) | ✓ (3) | ✓ (3) |  |
| `rcl_interfaces/rcl_interfaces/msg/ParameterType` | ✓ (10) | ✓ (10) | ✓ (10) | ✓ (10) |  |
| `rcl_interfaces/rcl_interfaces/msg/ParameterValue` | ✓ (10) | ✓ (10) | ✓ (10) | ✓ (10) |  |
| `rcl_interfaces/rcl_interfaces/msg/SetLoggerLevelsResult` | — | — | ✓ (2) | ✓ (2) | `humble`→`jazzy`: only in `jazzy` |
| `rcl_interfaces/rcl_interfaces/msg/SetParametersResult` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `rcl_interfaces/rcl_interfaces/srv/DescribeParameters` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `rcl_interfaces/rcl_interfaces/srv/GetLoggerLevels` | — | — | ✓ (2) | ✓ (2) | `humble`→`jazzy`: only in `jazzy` |
| `rcl_interfaces/rcl_interfaces/srv/GetParameterTypes` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `rcl_interfaces/rcl_interfaces/srv/GetParameters` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `rcl_interfaces/rcl_interfaces/srv/ListParameters` | ✓ (4) | ✓ (4) | ✓ (4) | ✓ (4) |  |
| `rcl_interfaces/rcl_interfaces/srv/SetLoggerLevels` | — | — | ✓ (2) | ✓ (2) | `humble`→`jazzy`: only in `jazzy` |
| `rcl_interfaces/rcl_interfaces/srv/SetParameters` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `rcl_interfaces/rcl_interfaces/srv/SetParametersAtomically` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `rcl_interfaces/rosgraph_msgs/msg/Clock` | ✓ (1) | ✓ (1) | ✓ (1) | ✓ (1) |  |
| `rcl_interfaces/service_msgs/msg/ServiceEventInfo` | — | — | ✓ (8) | ✓ (8) | `humble`→`jazzy`: only in `jazzy` |
| `rcl_interfaces/statistics_msgs/msg/MetricsMessage` | ✓ (6) | ✓ (6) | ✓ (6) | ✓ (6) |  |
| `rcl_interfaces/statistics_msgs/msg/StatisticDataPoint` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `rcl_interfaces/statistics_msgs/msg/StatisticDataType` | ✓ (6) | ✓ (6) | ✓ (6) | ✓ (6) |  |
| `rcl_interfaces/test_msgs/action/NestedMessage` | ✓ (9) | ✓ (9) | ✓ (9) | ✓ (9) |  |
| `rcl_interfaces/test_msgs/msg/Builtins` | ✓ (2) | ✓ (2) | ✓ (2) | ✓ (2) |  |
| `rcl_interfaces/type_description_interfaces/msg/Field` | — | — | ✓ (3) | ✓ (3) | `humble`→`jazzy`: only in `jazzy` |
| `rcl_interfaces/type_description_interfaces/msg/FieldType` | — | — | ✓ (93) | ✓ (93) | `humble`→`jazzy`: only in `jazzy` |
| `rcl_interfaces/type_description_interfaces/msg/IndividualTypeDescription` | — | — | ✓ (2) | ✓ (2) | `humble`→`jazzy`: only in `jazzy` |
| `rcl_interfaces/type_description_interfaces/msg/KeyValue` | — | — | ✓ (2) | ✓ (2) | `humble`→`jazzy`: only in `jazzy` |
| `rcl_interfaces/type_description_interfaces/msg/TypeDescription` | — | — | ✓ (2) | ✓ (2) | `humble`→`jazzy`: only in `jazzy` |
| `rcl_interfaces/type_description_interfaces/msg/TypeSource` | — | — | ✓ (3) | ✓ (3) | `humble`→`jazzy`: only in `jazzy` |
| `rcl_interfaces/type_description_interfaces/srv/GetTypeDescription` | — | — | ✓ (8) | ✓ (8) | `humble`→`jazzy`: only in `jazzy` |
