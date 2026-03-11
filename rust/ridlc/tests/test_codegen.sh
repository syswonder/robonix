#!/usr/bin/env bash
set -euo pipefail

RIDLC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARTIFACT_DIR="$RIDLC_DIR/tests/.artifacts/codegen"
INPUT_STREAM_DIR="$ARTIFACT_DIR/input_stream"

if [[ -t 1 ]]; then
    RIDLC_TEST_PREFIX=$'\033[1;38;5;214m[ridlc-tests]\033[0m'
else
    RIDLC_TEST_PREFIX='[ridlc-tests]'
fi

log_info() {
    printf '%s %s\n' "$RIDLC_TEST_PREFIX" "$*"
}

log_error() {
    printf '%s %s\n' "$RIDLC_TEST_PREFIX" "$*" >&2
}

rm -rf "$ARTIFACT_DIR"
mkdir -p "$ARTIFACT_DIR"

log_info "generating Python/ROS2 output into $ARTIFACT_DIR"

cargo run --manifest-path "$RIDLC_DIR/Cargo.toml" -- --lang python \
    --layout workspace \
    -I "$RIDLC_DIR/../robonix-interfaces/lib/robonix_runtime_interfaces" \
    -I "$RIDLC_DIR/../robonix-interfaces/lib/rcl_interfaces" \
    -I "$RIDLC_DIR/../robonix-interfaces/lib/common_interfaces" \
    -o "$ARTIFACT_DIR" \
    -i "$RIDLC_DIR/../robonix-interfaces/ridl"

cat >"$ARTIFACT_DIR/input_stream.ridl" <<'EOF'
namespace robonix/prm/debug

import std_msgs/msg/String

stream alerts {
    input msg std_msgs/msg/String
}
EOF

cargo run --manifest-path "$RIDLC_DIR/Cargo.toml" -- --lang python \
    --layout workspace \
    -I "$RIDLC_DIR/../robonix-interfaces/lib/robonix_runtime_interfaces" \
    -I "$RIDLC_DIR/../robonix-interfaces/lib/rcl_interfaces" \
    -I "$RIDLC_DIR/../robonix-interfaces/lib/common_interfaces" \
    -o "$INPUT_STREAM_DIR" \
    -i "$ARTIFACT_DIR/input_stream.ridl"

required_files=(
    "src/generated/robonix_interfaces/package.xml"
    "src/generated/robonix_interfaces/setup.py"
    "src/generated/robonix_interfaces/setup.cfg"
    "src/generated/robonix_interfaces/resource/robonix_interfaces"
    "src/generated/robonix_interfaces/robonix_runtime_pb2.py"
    "src/generated/robonix_interfaces/robonix_runtime_pb2_grpc.py"
    "src/generated/robonix_interfaces/robonix/hal/base/status_query.py"
    "src/generated/robonix_interfaces/robonix/hal/base/motion_cmd_command.py"
    "src/generated/robonix_interfaces/robonix/hal/localization/pose_stream.py"
    "src/app/robonix_interfaces_app/package.xml"
    "src/app/robonix_interfaces_app/setup.py"
    "src/app/robonix_interfaces_app/robonix_interfaces_app/combined_runtime.py"
    "src/app/robonix_interfaces_app/robonix_interfaces_app/robonix/hal/base/status_server.py"
    "src/app/robonix_interfaces_app/robonix_interfaces_app/robonix/hal/base/motion_cmd_server.py"
    "src/app/robonix_interfaces_app/robonix_interfaces_app/robonix/hal/localization/pose_publisher.py"
    "src/generated/robonix_interfaces_ros2/package.xml"
    "src/generated/robonix_interfaces_ros2/CMakeLists.txt"
    "src/generated/robonix_interfaces_ros2/srv/HalBaseStatus.srv"
    "src/generated/robonix_interfaces_ros2/action/HalBaseMotionCmd.action"
    "src/vendor/robonix_msgs/package.xml"
    "src/vendor/robonix_msgs/CMakeLists.txt"
    "src/vendor/robonix_msgs/msg/CommandResult.msg"
    "src/vendor/std_msgs/package.xml"
    "src/vendor/geometry_msgs/package.xml"
    "src/vendor/nav_msgs/package.xml"
    "src/vendor/action_msgs/package.xml"
)

for rel in "${required_files[@]}"; do
    if [[ ! -f "$ARTIFACT_DIR/$rel" ]]; then
        log_error "missing generated file: $rel"
        exit 1
    fi
done

RIDLC_TEST_PREFIX="$RIDLC_TEST_PREFIX" python3 - "$ARTIFACT_DIR" <<'PY'
from __future__ import annotations

import os
import pathlib
import py_compile
import sys
prefix = os.environ.get("RIDLC_TEST_PREFIX", "[ridlc-tests]")


out_dir = pathlib.Path(sys.argv[1])
runtime_pkg = out_dir / "src" / "generated" / "robonix_interfaces"
app_pkg = out_dir / "src" / "app" / "robonix_interfaces_app" / "robonix_interfaces_app"
rosidl_pkg = out_dir / "src" / "generated" / "robonix_interfaces_ros2"
msg_pkg = out_dir / "src" / "vendor" / "robonix_msgs"
std_msgs_pkg = out_dir / "src" / "vendor" / "std_msgs"

for py_file in runtime_pkg.rglob("*.py"):
    py_compile.compile(str(py_file), doraise=True)
for py_file in app_pkg.rglob("*.py"):
    py_compile.compile(str(py_file), doraise=True)

query_src = (runtime_pkg / "robonix/hal/base/status_query.py").read_text()
assert "class Ros2StatusServer" in query_src
assert "def resolve_status_service" in query_src
assert 'namespace="robonix/hal/base"' in query_src

action_src = (rosidl_pkg / "action/HalBaseMotionCmd.action").read_text()
assert "geometry_msgs/Twist cmd" in action_src
assert "robonix_msgs/CommandResult status" in action_src

srv_src = (rosidl_pkg / "srv/HalBaseStatus.srv").read_text()
assert "std_msgs/String req" in srv_src
assert "std_msgs/String res" in srv_src

package_xml = (runtime_pkg / "package.xml").read_text()
assert "<name>robonix_interfaces</name>" in package_xml

cmd_result_msg = (msg_pkg / "msg/CommandResult.msg").read_text()
assert "bool success" in cmd_result_msg
assert "string message" in cmd_result_msg

std_msgs_package_xml = (std_msgs_pkg / "package.xml").read_text()
assert "<name>std_msgs</name>" in std_msgs_package_xml

app_query_src = (app_pkg / "robonix/hal/base/status_server.py").read_text()
assert "create_status_server" in app_query_src

app_cmd_src = (app_pkg / "robonix/hal/base/motion_cmd_server.py").read_text()
assert "create_motion_cmd_server" in app_cmd_src

app_stream_src = (app_pkg / "robonix/hal/localization/pose_publisher.py").read_text()
assert "create_pose_publisher" in app_stream_src

combined_src = (app_pkg / "combined_runtime.py").read_text()
assert "MultiThreadedExecutor" in combined_src
assert "create_status_server" in combined_src
assert "create_motion_cmd_server" in combined_src
assert "create_pose_publisher" in combined_src

print(f"{prefix} python syntax and generated file assertions passed")
PY

RIDLC_TEST_PREFIX="$RIDLC_TEST_PREFIX" python3 - "$INPUT_STREAM_DIR" <<'PY'
from __future__ import annotations

import os
import pathlib
import sys
prefix = os.environ.get("RIDLC_TEST_PREFIX", "[ridlc-tests]")


out_dir = pathlib.Path(sys.argv[1])
stream_src = (
    out_dir
    / "src"
    / "generated"
    / "robonix_interfaces"
    / "robonix"
    / "prm"
    / "debug"
    / "alerts_stream.py"
).read_text()
app_stream_src = (
    out_dir
    / "src"
    / "app"
    / "robonix_interfaces_app"
    / "robonix_interfaces_app"
    / "robonix"
    / "prm"
    / "debug"
    / "alerts_subscriber.py"
).read_text()

assert "class AlertsSubscriber" in stream_src
assert "class Ros2AlertsSubscriber" in stream_src
assert "def create_alerts_subscriber" in stream_src
assert "resolve_alerts_consumer_topic" in stream_src
assert "register_alerts_provider" not in stream_src
assert "class AlertsPublisher" not in stream_src
assert "class Ros2AlertsPublisher" not in stream_src
assert "def create_alerts_publisher" not in stream_src
assert "create_alerts_subscriber" in app_stream_src

print(f"{prefix} input stream direction assertions passed")
PY

log_info "codegen test passed"
