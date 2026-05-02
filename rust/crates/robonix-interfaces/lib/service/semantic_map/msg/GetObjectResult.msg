# Wrapper around (Object, bool found) so codegen can emit a single
# response message for scene/get_object. `found=false` is set when
# the queried object_id isn't in the registry; in that case `obj.id`
# echoes the queried id and all other fields are zero/empty.
Object obj
bool found
