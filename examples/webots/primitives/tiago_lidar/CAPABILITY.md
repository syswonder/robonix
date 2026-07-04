---
description: Planar lidar — supplementary distance / range sensing near the base.
---

# Tiago lidar (`robonix/primitive/lidar`)

Hokuyo planar lidar mounted near the base. Supplementary distance sensor.

## Tools

### `snapshot` — `robonix/primitive/lidar/snapshot`
- input: none
- returns: `sensor_msgs/LaserScan` JSON. `ranges[i]` is the distance (meters)
  at angle `angle_min + i*angle_increment`. Hokuyo on tiago has angle range
  roughly ±π/2 (FOV is in front of the robot only).
- use cases:
  - "is there an obstacle within X m in front of me?"  →  scan the middle
    of `ranges[]` for the smallest value.
  - "where is the nearest open space?"  →  argmax of `ranges[]`.
- DO NOT use lidar to localize on a map; it has no map context.

## Reasoning

Lidar is good for "stop before hitting a wall" sanity checks. Use it as a
safety floor before committing to a chassis/cmd burst, not as the primary
sensor for finding things by visual category (use camera for that).
