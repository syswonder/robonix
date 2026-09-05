# VLM Verifier

A generic visual result verification service. Provider ID: `vlm_verifier`.
MCP contract: `robonix/service/verifier/verify`.

The service takes one fresh RGB image from the camera selected by an Executor
verification rule and asks a vision model whether the intended result is visible.
It has no robot-specific grasp logic and does not move the robot.

## Deployment configuration

Add the service entry and verification rule to your deployment manifest.
Merge these entries into the existing `service` list and `system.executor`
mapping; do not create duplicate YAML keys or replace other providers.

```yaml
service:
  - name: vlm_verifier
    path: ${ROBONIX_SOURCE_PATH}/services/verifiers/vlm_verifier
    config:
      vlm:
        base_url: ${VLM_BASE_URL}
        api_key: ${VLM_API_KEY}
        model: ${VLM_MODEL}

system:
  executor:
    listen: 0.0.0.0:50061
    verification:
      - target_contract_id: robonix/skill/pick/pick
        target_provider_id: pick
        verifier_provider_id: vlm_verifier
        verifier_args:
          camera_provider_id: orbbec_wrist_camera
```

| Field | Meaning |
| --- | --- |
| `service[].name` | Provider ID used by `verifier_provider_id`. |
| `service[].path` | Package directory containing `package_manifest.yaml`. |
| `config.vlm.base_url` | OpenAI-compatible API base URL, normally ending in `/v1`. The service appends `/chat/completions`. |
| `config.vlm.api_key` | API credential. Required; sent as a Bearer token. |
| `config.vlm.model` | Model ID supporting image_url data URIs. Required; no model default. |
| `target_contract_id` | Completed capability to verify. Required. |
| `target_provider_id` | Optional exact target provider filter. An exact rule takes precedence over a contract-only rule. |
| `verifier_provider_id` | Exact verifier provider to invoke. Required. |
| `verifier_args.camera_provider_id` | Exact RGB camera provider for this rule. Required by VLM Verifier. |

The camera belongs in each Executor rule, not in the verifier service config.
Different rules can use different cameras while sharing the same VLM Verifier.
The service resolves the camera's ROS topic through Atlas; do not supply a topic
name in `camera_provider_id`.

Credentials can be supplied through the deployment's existing environment:

```dotenv
VLM_BASE_URL=https://your-vlm-host.example/v1
VLM_API_KEY=your-api-key
VLM_MODEL=your-vision-model-id
```

These are placeholders. The Ranger deployment's `start.sh` already exports
assignments from its `.env` file. The verifier receives the expanded values
through lifecycle initialization; it does not read `.env` or Pilot config itself.
Independent environment variable names may be used in the manifest if the
verifier should use a different model or account.

Deleting a verification rule disables verification for that target. Merely
starting the service does not enable verification. The current Ranger arm-grasp
profile enables only `pick`, not `put_down`.

## Runtime flow

1. Executor waits for the target capability to finish. A failed target returns
   its original failure without invoking a verifier.
2. After a successful target call, Executor matches a verification rule and
   invokes the configured verifier over MCP before emitting the final node state.
3. The verifier parses the action description, arguments, output, and camera
   provider from the request.
4. It queries Atlas for that exact provider's
   `robonix/primitive/camera/rgb` capability, connects to its ROS 2 topic, and
   waits for one newly stamped frame. There is no snapshot RPC, default-topic
   fallback, or camera substitution.
5. It converts the image into JPEG and sends it with the action context to the
   configured `<base_url>/chat/completions` endpoint.
6. It accepts only a JSON object containing exactly a boolean `passed` and a
   nonempty string `detail`. Executor then reports one final result.

A passing response looks like this:

```json
{
  "passed": true,
  "detail": "The specified red bottle is visibly held by the gripper."
}
```

The image must provide clear evidence. For picking, the specified object must
visibly be held, not just touched or nearby. For placement, it must visibly be
at the intended destination. An invisible target, wrong object, or ambiguous
evidence returns `passed=false`. The original capability's success flag is
not visual proof. A single image cannot establish continuing grip stability.

| Outcome | Executor result |
| --- | --- |
| `passed=true` | Success, preserving the original capability output. |
| `passed=false` | Failure with `result verification failed: <detail>`. |
| Camera, network, timeout, or model-format error | Failure with `result verification unavailable: ...`. |

Each request has isolated observation state. ROS subscriptions, Atlas channels,
and HTTP connections are released after use. Logs correlate requests with
`call_id`; credentials, image payloads, and upstream response bodies are not logged.

## Request contract

Executor constructs the request automatically; ordinary deployments do not need
to write `args_json` by hand.

```text
string call_id
string args_json
---
bool passed
string detail
```

The decoded `args_json` envelope is:

```json
{
  "target_provider_id": "pick",
  "target_contract_id": "robonix/skill/pick/pick",
  "target_description": "Pick up the red bottle",
  "target_args": {"object_name": "red bottle"},
  "target_output": {"success": true},
  "verifier_args": {"camera_provider_id": "orbbec_wrist_camera"}
}
```

The description and provider/contract IDs must be nonempty strings.
`target_args` and `target_output` are required and retain Executor's parsed JSON
values or original strings.

## Requirements and limits

- Linux and ROS 2, defaulting to Humble; system Python with matching `rclpy`
  and `sensor_msgs`. Pillow and the other Python dependencies are installed at build time.
- `ROS_DISTRO` selects an installed ROS distribution. `RMW_IMPLEMENTATION`
  and `ROS_DOMAIN_ID` must match the camera.
- RGB images must have valid timestamps in the verifier's ROS clock domain.
  Old or zero-stamped frames are ignored. ROS simulated time is not currently
  configured by this service.
- Supported image encodings: `rgb8`, `bgr8`, `rgba8`, `bgra8`, and `mono8`.
  Dimensions, row stride, and payload length are validated. Depth and camera
  intrinsics are not required. Conversion does not depend on cv_bridge.
- Fixed deadlines: 50 seconds overall, 5 seconds for observation, and 40 seconds
  for VLM evaluation, below Executor's 60-second verification timeout.
  These are implementation constants, not configurable YAML fields.
- One image and one model request per verification. No retries or multiple-view voting.

## Build and start

Initialize the IDL submodules from the Robonix checkout:

```bash
git submodule update --init capabilities/lib/common_interfaces capabilities/lib/rcl_interfaces capabilities/lib/unique_identifier_msgs
```

With `uv`, `rbnx`, and `robonix-codegen` on PATH, run from this package:

```bash
bash scripts/build.sh
```

The build uses system Python with system-site-packages for ROS compatibility.
It generates interfaces from this checkout using an isolated CLI configuration
under `rbnx-build/cli`, without changing global rbnx setup.

Start the package through the deployment lifecycle. Its manifest runs
`scripts/start.sh`; no dependencies are installed at runtime. Running that
script alone does not provide the VLM configuration normally supplied by
lifecycle initialization.

The Ranger arm-grasp entry point selects `~/lgw/robonix` by default, configures
a profile-local `ROBONIX_HOME`, and uses that checkout's debug Executor.
Build it once from the repository root with:

```bash
cargo build -p robonix-executor
```

`ROBONIX_ARM_GRASP_SOURCE_PATH` can select another built checkout.
The current robot profile already contains the verifier service and pick rule.
Its existing `start-arm-grasp.sh` starts the robot stack, including hardware.

## Tests

From this package directory:

```bash
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
export PYTHONPATH="$PWD:$PWD/rbnx-build/codegen/proto_gen:$PWD/rbnx-build/codegen/robonix_mcp_types:${PYTHONPATH:-}"
rbnx-build/venv/bin/python -m unittest discover -s tests -v
rbnx-build/venv/bin/python tests/smoke_mcp.py
```

To include a real Atlas and the built Executor:

```bash
VLM_TEST_EXECUTOR_BIN="$PWD/../../../target/debug/robonix-executor" \
  rbnx-build/venv/bin/python tests/smoke_mcp.py
```

The smoke tests use a synthetic ROS camera, real MCP HTTP, and a simulated VLM
HTTP endpoint. The Executor checks assert pass, rejection, unavailable results,
one terminal state, and preserved original output. Test processes are cleaned up;
these tests do not control the physical arm.

Physical acceptance remains operator-owned: test a correct grasp, an empty grasp,
a wrong object, and an invisible target. The configured camera must see the
final gripper/object state after the skill has finished.
