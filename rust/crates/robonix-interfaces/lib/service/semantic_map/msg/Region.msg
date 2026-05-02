# A frame-tagged sphere around a point. Used by scene/get_semantic_map
# (and by SnapshotSpec's region_*  fields, which inline these for
# Pydantic-friendly flat schemas).
string frame_id
float64 center_x
float64 center_y
float64 center_z
float64 radius_m
