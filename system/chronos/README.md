# Chronos — unified time

One of the 12 Robonix system components. Provides a single monotonic
time source plus PTP / IEEE-1588 alignment between hardware (lidar,
camera, IMU, chassis odometry) and the runtime.

**Status — v0.1 stub.** Not yet implemented.

For now, components read system wall time / per-sensor timestamps
directly. Scene / mapping fuse multimodal data on their own. Chronos
will absorb that responsibility in v0.2.

When implemented, Chronos will:

- expose a single `time/now` capability returning a monotonic, machine-
  global time stamp consistent with sensor headers,
- run a PTP master/slave loop (1588v2) to discipline NIC clocks across
  the body's compute units,
- emit drift / sync-quality metrics to [vitals](../vitals/).

See the whitepaper §3 *L0 系统服务层* for the broader picture.
