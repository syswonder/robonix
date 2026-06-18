# state: 0=PENDING 1=RUNNING 2=SUCCEEDED 3=FAILED 4=CANCELED 5=TIMEOUT 6=PAUSED
string plan_id
uint32 node_index
uint32 node_kind
uint32 state
# rtdl non-leaf node result, like `Parallel`, `Sequence`
string operator_detail 
# rtdl leaf node result
pilot/CapabilityCallResult leaf_result
