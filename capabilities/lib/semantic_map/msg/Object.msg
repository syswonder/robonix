# One tracked thing in the world. Flat primitives only — no nested types,
# no enums. Position is `frame_id` xyz + yaw (radians, CCW around +z).
# The robot itself appears here with its current map-frame pose;
# yaw is meaningful for it. For passive objects yaw is the best-effort
# orientation the registry has (0.0 if unknown). The registry only
# emits objects it currently believes exist (no missing flag).

string id
string label
float64 x
float64 y
float64 z
float64 yaw
# Full extents of the gravity-aligned box along its own axes, metres.
# Zero on entries the registry holds no box for, such as annotations.
# UpdateObjectGeometry requires these, so they have to be readable here:
# an operator nudging one object must be able to resend its current size
# instead of guessing it.
float64 size_x
float64 size_y
float64 size_z
# World frame that x/y/z/yaw and the extents are expressed in.
# UpdateObjectGeometry rejects a frame that differs from the object's own,
# so the caller has to be able to read it here. Empty means the producer did
# not record a frame, not that the coordinates are frameless: scene-graph
# derived entries carry map-frame coordinates but no frame string, so an
# edit built from one of those is rejected rather than applied blind.
string frame_id
# Observations fused into this object so far. It advances on every
# perception update, so a caller that read an object and then edits it can
# tell whether perception moved underneath it. Zero for entries the
# registry did not derive from perception. int32 rather than an unsigned
# type to match SceneGraphNode.observation_count, which is the same
# quantity under the same name and is already published.
int32 observation_count
float64 last_seen_unix
