# User-authored semantic annotation anchored to the SLAM map.
# Rooms and POIs are maintained by scene, persisted per map_id, and
# returned with the scene graph so clients/agents can reason about
# human-provided regions as first-class scene context.

string annotation_id
string kind
string name
float64[] points_xy
float64 theta
bool stale
string stale_reason
float64 updated_at_unix
