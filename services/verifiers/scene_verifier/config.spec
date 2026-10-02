specVersion: 1
description: >-
  Tolerances and the Scene observation timeout used to verify planar
  navigation results. Every value must be a finite number; booleans are
  rejected.

# Position and yaw errors must be strictly less than their tolerances; an
# error equal to its tolerance fails verification. Configuration is applied
# once: a repeated Driver(CMD_INIT) with different values fails.
properties:
  distance_tolerance_m:
    type: number
    description: Maximum planar distance in metres between the observed robot position and the goal. Must be greater than 0.
    default: 0.5

  yaw_tolerance_rad:
    type: number
    description: >-
      Maximum yaw error in radians, used when the verification rule sets
      check_yaw. Must not exceed pi. Must be greater than 0.
    default: 0.35
    maximum: 3.141592653589793

  observation_timeout_s:
    type: number
    description: >-
      Timeout in seconds for the Scene robot-context call. Executor applies
      its own deadline to the whole verification request. Must be greater than 0 and less than 60.
    default: 5.0
