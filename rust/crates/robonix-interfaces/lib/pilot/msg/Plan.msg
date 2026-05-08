# Plan: structural payload pilot hands to executor for execution.
# Semantically a behavior tree; v1 is a degenerate linear list of calls.
# TODO: evolve to explicit BT nodes + RTDL bindings; keep CapabilityCall[]
# as the linear encoding until then.
string plan_id
string session_id
uint32 round
CapabilityCall[] calls
