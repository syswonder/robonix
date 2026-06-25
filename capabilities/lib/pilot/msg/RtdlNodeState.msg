string plan_id
uint32 node_index
uint32 node_kind
uint32 state
uint32 PENDING=0
uint32 RUNNING=1
uint32 SUCCEEDED=2
uint32 FAILED=3
uint32 CANCELED=4
uint32 TIMEOUT=5
uint32 PAUSED=6
# RTDL non-leaf node result, like sequence or parallel.
string operator_detail
# RTDL leaf node result.
CapabilityCallResult leaf_result
# Stable identifier copied from Plan.nodes[node_index].
string op_id
# Human-readable intent copied from Plan.nodes[node_index].
string description
