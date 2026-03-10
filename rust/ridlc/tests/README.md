# ridlc tests

This directory contains validation and integration-test assets for the RIDL
Python/ROS2 generation flow.

## Test layers

- `test_codegen.sh`
  - runs `ridlc`
  - validates the directly generated ROS workspace layout
  - verifies the generated Python files are syntactically valid
  - verifies the generated `src/app/robonix_interfaces_app` skeleton exists
- `run_zenoh_rmw_e2e.sh`
  - asks `ridlc` to generate a temporary ROS workspace directly
  - adds the test-only `robonix_ridlc_e2e` package into that workspace
  - builds the generated runtime, interface, and app packages together
  - starts `robonix-server`
  - starts a local `rmw_zenohd` router
  - starts one combined test program that imports multiple generated RIDL modules in a single process
  - exercises query/service, stream/topic, and command/action over ROS2 with
    `RMW_IMPLEMENTATION=rmw_zenoh_cpp`
  - checks runtime channel allocation through the gRPC meta API

## Generated workspace layers

- `src/generated`
  - RIDL-generated runtime package and ROS interface packages
- `src/vendor`
  - vendored ROS dependencies needed by the generated workspace
- `src/app`
  - user-editable app skeletons that import `src/generated` and are meant to be customized

## Prerequisites for end-to-end test

The end-to-end script assumes:

- ROS2 is installed and sourceable
- `rclpy` is available
- `rmw_zenoh_cpp` is installed
- `grpcio` is available for Python
- `grpcio-tools` is available if you need to regenerate `proto/gen/*`

The script starts its own Zenoh router and uses shorter default waits for
faster local debugging. You can override them with:

- `RIDLC_E2E_SERVER_TCP_WAIT_RETRIES`
- `RIDLC_E2E_RUNTIME_PROBE_WAIT_RETRIES`
- `RIDLC_E2E_ZENOH_ROUTER_WAIT_RETRIES`

## Quick usage

From `rust/ridlc`:

```bash
sudo apt update
sudo apt install -y python3-grpcio ros-humble-rmw-zenoh-cpp
./tests/test_codegen.sh
./tests/run_zenoh_rmw_e2e.sh
```
