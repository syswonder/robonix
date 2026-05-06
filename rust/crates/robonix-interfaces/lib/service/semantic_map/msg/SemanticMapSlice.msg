# Response from scene/get_semantic_map. v1: surfaces (planes registered
# by geometry layer) + a *reference* to Nav2's 2D occupancy grid (the
# grid itself stays where Nav2 has it; shipping voxel data through MCP
# would balloon payloads). 3D voxel grid is deferred (TODO once we have
# a real producer).
Object[] surfaces
string occupancy_grid_topic   # e.g. "/global_costmap/costmap" or "" if none
string occupancy_grid_frame   # e.g. "map"
float64 stamp_unix
