# Semantic object node — the canonical record `system/scene` keeps for one
# tracked thing in the world. v1 extension: confidence + first/last seen +
# observation count + a `missing` flag flipped after the grace period when
# data association can no longer find this object in incoming detections
# (we keep the record around because Pilot may still ask about it).
#
# Field-name note: `relations[].target_entity_id` is a legacy spelling kept
# for codegen continuity; new Python code reads it back as `target_object_id`.
string id
string label
Relation[] relations
string[] registered_skills
string[] registered_primitives
FrameMapping[] frame_mapping
float32 confidence
builtin_interfaces/Time first_seen
builtin_interfaces/Time last_seen
uint32 observation_count
bool missing
