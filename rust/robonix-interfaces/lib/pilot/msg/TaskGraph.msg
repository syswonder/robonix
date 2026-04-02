# Task graph on the wire: semantically a behavior tree; v1 is a linear list of calls only.
# TODO: evolve to explicit BT nodes + RTDL bindings; keep TaskCall[] as degenerate linear encoding until then.
string graph_id
string session_id
uint32 round
TaskCall[] calls
