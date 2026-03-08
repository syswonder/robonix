# Zenoh cross-ROS tests

Tests for connecting ROS 2 nodes across distros (Humble / Jazzy) via Zenoh. Two cases: same message type on both sides, or different IDLs with a translation layer.

---

## Cross-distro adaptation (any ROS package)

Goal: have nodes on distro A (e.g. Humble) and distro B (e.g. Jazzy) communicate over Zenoh.

**Same message type on both sides** (e.g. `geometry_msgs/Point`, same topic name):

1. **Same distro** (e.g. all Jazzy): Use rmw_zenoh. Install `ros-<distro>-rmw-zenoh-cpp`, run one router `ros2 run rmw_zenoh_cpp rmw_zenohd`, set `RMW_IMPLEMENTATION=rmw_zenoh_cpp` and connect endpoints (e.g. session config or `ZENOH_CONFIG_OVERRIDE=connect/endpoints=["tcp/<router>:7447"]`). No bridge; all topics/services/actions go over Zenoh.
2. **Different distros** (e.g. Humble + Jazzy): rmw_zenoh is not compatible (type-hash mismatch, [#569](https://github.com/ros2/rmw_zenoh/issues/569)). Use zenoh-bridge-ros2dds: one bridge in router mode (`zenoh-bridge-ros2dds -l tcp/0.0.0.0:7447`), the other in client mode (`zenoh-bridge-ros2dds client -e tcp/<listener>:7447`). Both sides keep default DDS; use different `ROS_DOMAIN_ID` per side so only the bridge links them. Image: `eclipse/zenoh-bridge-ros2dds:nightly`.

**Different message types** (same semantic, different IDL):

3. Keep DDS per side (different `ROS_DOMAIN_ID`). Define a canonical format (e.g. JSON on a fixed Zenoh key). Run a small bridge per side: subscribe to local ROS topic → convert to canonical → publish on Zenoh; subscribe on Zenoh → convert to local msg → publish to local ROS. Topic names and types can differ; conversion lives in the bridge.

---

## What this repo runs

- **Test 1 (different IDL):** Humble publishes `cross_ros_humble_msgs/Point3D` (x,y,z), Jazzy subscribes `cross_ros_jazzy_msgs/Point3D` (header + position_*). Canonical JSON on Zenoh + Python bridges (`humble_bridge.py`, `jazzy_bridge.py`). `./run_test.sh` — uses `docker-compose.yaml`.
- **Test 2 (same IDL, cross-distro):** Both use `geometry_msgs/Point` on `/point`. zenoh-bridge-ros2dds connects Humble and Jazzy. `./run_test_same_idl.sh` — uses `docker-compose.same-idl-bridge.yaml`.

Same distro only (e.g. two Jazzy): `docker-compose.same-idl.yaml` + rmw_zenoh_cpp + one `rmw_zenohd`; session config in `config/rmw_zenoh_client.json5` (disable SHM in Docker: `transport/shared_memory/enabled: false`).

---

## Layout and run

```
zenoh_cross_ros/
├── docker-compose.yaml              # Test 1
├── docker-compose.same-idl-bridge.yaml  # Test 2 cross-distro
├── docker-compose.same-idl.yaml     # Test 2 same-distro (rmw_zenoh)
├── run_test.sh                      # Test 1
├── run_test_same_idl.sh             # Test 2
├── config/rmw_zenoh_client.json5
├── docker/
├── msgs/{humble,jazzy}/
└── scripts/   # bridges + pub/sub demos
```

From repo root: `./run_test.sh` or `./run_test_same_idl.sh`. Requires Docker Compose and `osrf/ros:humble-desktop` / `osrf/ros:jazzy-desktop`.
