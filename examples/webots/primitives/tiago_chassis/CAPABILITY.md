---
description: Mobile differential-drive base — all movement during navigation, exploration, and search.
---

# Tiago chassis (`robonix/primitive/chassis`)

The robot's mobile base. Use this for ALL movement during interactive
exploration / search / wandering — paired with `camera/snapshot`.

For "where is the robot" queries, subscribe to `service/map/pose`
(SLAM-corrected, map-frame) — that's a localization concern, not a
chassis-primitive one.

## Tools

### `cmd` — `robonix/primitive/chassis/move`
- input: any of `linear_x linear_y linear_z angular_x angular_y angular_z`
  (all default to 0 — pass only the axes you want to actuate).
- duration: ~1 s by default (`TIAGO_CHASSIS_CMD_DURATION_SEC`); after the
  burst the driver publishes a zero-Twist stop.
- returns: JSON ack `{status, linear_x, angular_z, duration}`.

#### Burst pattern (use this for visual exploration)
1. `camera/snapshot` to see the scene.
2. Reason about what's there.
3. Issue ONE short `cmd` burst (typical magnitudes):
   - move forward: `linear_x ≈ 0.10–0.20`
   - turn left:    `angular_z ≈ +0.4`  (≈ 23°/s)
   - turn right:   `angular_z ≈ −0.4`
4. Repeat from step 1.

Don't issue multiple `cmd`s back-to-back without a snapshot — you'll
overshoot blindly.
