<!-- SPDX-License-Identifier: MulanPSL-2.0 -->

# Piper health primitive

This package exposes `robonix/primitive/health/state` and
`robonix/primitive/health/stream` for the static Piper Vitals demo. It reports
deterministic values for the body, arm, six joints, and parallel gripper
actuator declared in `examples/piper_vitals/soma.yaml`. The deployment-level
`health_demo.py` command can inject selected actuator faults at runtime.

The data is not read from physical hardware. `scenario: normal` is the stable
entry point for future fault profiles; unsupported values fail lifecycle
initialization instead of silently returning healthy data.

Configuration:

```yaml
config:
  scenario: normal
  interval_s: 0.5
  voltage: 24.0
  control_file: ../../.runtime/piper_health.json
```

The control file accepts `mode: normal`, or `mode: fault` with targets from
`joint_1` through `joint_6` and `gripper`. The helper writes it atomically; a
missing file means normal health.

Build and test from the deployment directory:

```bash
cd /path/to/robonix/examples/piper_vitals
rbnx build

cd primitives/piper_health
PYTHONPATH="$(rbnx path robonix-api):$PWD:$PWD/rbnx-build/codegen/proto_gen" \
  python3 -m unittest discover -s tests
```
