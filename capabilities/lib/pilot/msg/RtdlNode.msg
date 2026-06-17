# RTDL execution tree node encoded as an arena entry in Plan.nodes.
# node_kind: 0=sequence, 1=parallel, 2=do.
uint32 node_kind

# Child node indices for sequence and parallel nodes. Empty for do nodes.
uint32[] children

# Capability call payload for do nodes. Sequence and parallel leave this empty.
CapabilityCall call
