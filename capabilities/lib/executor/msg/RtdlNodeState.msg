# state: 0=PENDING 1=RUNNING 2=SUCCEEDED 3=FAILED 4=CANCELED 5=TIMEOUT 6=PAUSED
string plan_id
uint32 node_index
uint32 node_kind
uint32 state
# RTDL non-leaf node result, like sequence or parallel.
string operator_detail
# RTDL leaf node result.
pilot/CapabilityCallResult leaf_result
# Stable identifier copied from Plan.nodes[node_index].
string op_id
# Human-readable intent copied from Plan.nodes[node_index].
string description
