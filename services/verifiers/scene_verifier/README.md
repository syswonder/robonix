# Scene Verifier

Scene Verifier provides the `robonix/service/verifier/verify` contract for
planar navigation. After a configured navigation capability succeeds, Executor
invokes this provider to compare the navigation goal with the robot pose
reported by `robonix/system/scene/get_robot_context`.

The verifier supports `robonix/service/navigation/navigate` goals in the `map`
frame. It uses Scene observations instead of trusting the navigation output as
evidence that the robot reached its goal.

## Deployment

Merge the service entry and verification rule below into the existing
`service` list and `system.executor` mapping in the deployment manifest.

```yaml
system:
  executor:
    verification:
      - target_contract_id: robonix/service/navigation/navigate
        target_provider_id: nav2
        verifier_provider_id: scene_verifier
        verifier_args:
          scene_provider_id: scene
          check_yaw: true

service:
  - name: scene_verifier
    path: ../../services/verifiers/scene_verifier
    config:
      distance_tolerance_m: 0.5
      yaw_tolerance_rad: 0.35
      observation_timeout_s: 5.0
```

Starting the provider alone does not enable verification. Executor invokes it
only when a successful capability call matches a verification rule.

### Service configuration

| Field | Default | Constraint |
| --- | ---: | --- |
| `distance_tolerance_m` | `0.5` | Positive number in metres. |
| `yaw_tolerance_rad` | `0.35` | Positive number no greater than pi. |
| `observation_timeout_s` | `5.0` | Positive number less than 60 seconds. |

Position and yaw errors must be strictly less than their configured
tolerances. An error equal to its tolerance fails verification.

### Verification rule arguments

| Field | Default | Description |
| --- | ---: | --- |
| `scene_provider_id` | None | Required provider ID for Scene observations. |
| `check_yaw` | `true` | Whether to compare the observed yaw with the goal orientation. |
| `expected_map_id` | None | Optional non-empty map ID that the Scene response must match exactly. |

When `check_yaw` is enabled, the goal must contain a valid planar unit
quaternion. An identity quaternion represents zero yaw. When `check_yaw` is
disabled, the verifier does not require or inspect the goal orientation.

When `expected_map_id` is omitted, Scene must still report a non-empty map ID,
but the verifier cannot establish which map the original goal came from.

## Verification behavior

Verification passes when all enabled checks succeed:

- Scene reports a known, current robot pose and a non-empty map ID.
- The observed position is within `distance_tolerance_m` of the goal.
- The observed yaw is within `yaw_tolerance_rad` when `check_yaw` is enabled.
- The observed map ID matches `expected_map_id` when one is configured.

Unknown or stale poses, map mismatches, and errors outside the configured
tolerances produce `passed=false`. Invalid requests, malformed Scene responses,
and Atlas or MCP failures make verification unavailable. Executor converts both
outcomes into a failed capability result and preserves the original result only
when verification passes.

## Limitations

- Only planar goals with `goal.header.frame_id` set to `map` and a zero `z`
  coordinate are supported.
- `map_id` identifies a map, while `frame_id` identifies a coordinate frame;
  they are not interchangeable.
- Scene does not report a separate coordinate frame for the robot pose. The
  verifier relies on Scene's map-frame snapshot convention.
- Freshness is determined from Scene's `stale` field. Host and simulation clocks
  are not compared.
- Each request reads one Scene snapshot. The verifier does not retry, wait for
  further movement, or subscribe to ROS topics.
- `observation_timeout_s` applies to the asynchronous MCP observation. Executor
  applies its own deadline to the complete verification request.

## Build and test

With `uv` and `rbnx` available, run from the repository root:

```bash
rbnx validate services/verifiers/scene_verifier
rbnx build -p services/verifiers/scene_verifier
```

The build generates the `verifier_mcp` module. Do not edit generated files by
hand.

The unit tests do not require Atlas or ROS 2:

```bash
cd services/verifiers/scene_verifier
python3 -m unittest discover -s tests -v
```

For an end-to-end check, start a deployment containing Executor, Atlas, Scene,
and this provider. Inspect the correlated logs with:

```bash
rbnx logs -t scene_verifier
rbnx logs -t executor
```
