<!-- SPDX-License-Identifier: MulanPSL-2.0 -->

# TIAGo simulated health primitive

This Webots package provides `robonix/primitive/health/state` and
`robonix/primitive/health/stream`. It publishes deterministic nominal values
for the TIAGo base, wheels, battery, camera, lidar, and audio component paths
declared in `examples/webots/soma.yaml`. The `full` variant also reports all
seven arm joints plus the parallel gripper and its actuator declared in
`soma.full.yaml`.

Configuration is delivered through the primitive entry in
`robonix_manifest.yaml`:

```yaml
config:
  variant: lite
  scenario: normal
  interval_s: 0.5
  battery_percent: 82.0
  voltage: 24.8
  remaining_s: 10800
```

`variant` accepts `lite` (default) or `full`. It must match the Webots and
Soma variant selected by the example launch scripts.

`scenario` is the stable fault-injection entry point. Only `normal` is
implemented now; unsupported values fail lifecycle initialization instead of
silently reporting a healthy robot.
