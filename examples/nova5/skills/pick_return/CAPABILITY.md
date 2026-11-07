---
description: Pick the black Robonix box from the fixed checkerboard, return it to the observed position, and return Nova5 to the recorded home pose.
---

# Nova5 pick and return

Call `run` with `target: box` only after observing the live Nova5 wrist RGB camera. This skill locates the box again from fresh RGB frames and uses the calibrated board plane, 165 mm TCP and 50 mm/s Nova5 SDK moves. It checks workspace bounds, geometry and robot state before moving, validates the target from multiple RGB views before descent, and checks gripper contact during handling. Visual lift and placement checks run after releasing the box. It does not accept arbitrary poses or other targets.

`run` returns a run ID immediately. Executor polls `run/status` and forwards cancellation to `run/cancel`. `SUCCEEDED` means the workflow reported that the box was placed back and the arm reached the recorded home pose. If it fails or is canceled, inspect the physical scene before restarting the provider. Cancellation requests a stop but cannot guarantee a physical stop.
