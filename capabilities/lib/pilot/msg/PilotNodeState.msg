# Live RTDL node state forwarded to the chat for the forest view.
# Correlates with a Plan already sent via PilotEvent.plan (by plan_id).
string plan_id
uint32 node_index
uint32 node_kind
# state use RtdlNodeState.msg constants
uint32 state
string op_id
string description
