# Region- and freshness-scoped snapshot request — passed to scene/get_snapshot.
# Pilot calls this every LLM round, so the response is *always* scoped to a
# small region rather than the full registry.
#
# Sentinel conventions (ROS .msg has no optional types):
#   layers           empty array == default ["object","relation"]
#   region_frame     empty string == default "base_link"
#   region_radius_m  must be > 0; the service rejects radii > 50.0
string[] layers
string region_frame
float64 region_center_x
float64 region_center_y
float64 region_center_z
float64 region_radius_m
float64 freshness_s
bool include_stale
float64 min_confidence
uint32 max_objects
