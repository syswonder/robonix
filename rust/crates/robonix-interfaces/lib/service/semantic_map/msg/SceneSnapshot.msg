# Response from scene/get_snapshot. Carries scoped objects + relations +
# the surface objects (planes extracted by the geometry layer) at request
# time. `stamp_unix` is wall-clock seconds (until Chronos lands).
Object[] objects
Relation[] relations
Object[] surfaces
float64 stamp_unix
string frame_id
