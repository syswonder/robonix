# Live RTDL node state forwarded to the chat for the forest view.
# Correlates with a Plan already sent via PilotEvent.plan (by plan_id).
# state: 0=PENDING 1=RUNNING 2=SUCCEEDED 3=FAILED 4=CANCELED 5=TIMEOUT 6=PAUSED
string plan_id
uint32 node_index
uint32 node_kind
uint32 state
string op_id
string description
