# One tracked thing in the world. Seven primitives — no nested types,
# no enums. Position is map-frame xyz + yaw (radians, CCW around +z).
# The robot itself appears here with its current map-frame pose;
# yaw is meaningful for it. For passive objects yaw is the best-effort
# orientation the registry has (0.0 if unknown). The registry only
# emits objects it currently believes exist (no missing flag, no
# observation_count: those are internal telemetry, not API).

string id
string label
float64 x
float64 y
float64 z
float64 yaw
float64 last_seen_unix
