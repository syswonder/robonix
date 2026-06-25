# Correlates with Plan.plan_id for this execution round.
string plan_id
string session_id

uint32 round
# Every node that reached a terminal state in this round (leaf and non-leaf),
# as a full RtdlNodeState record — not just the leaf capability-call results.
RtdlNodeState[] results
bool any_failed
