# One plan per stream. START occurs exactly once and must be first.
uint32 message_kind
uint32 START=0
uint32 NODE=1
uint32 CLOSE=2
uint32 END=3

# START fields.
string plan_id
string session_id
uint32 round
# Internal-only Keystone credential. Executor validates it before admitting the
# plan and never copies it into execution events.
string auth_session_token

# NODE indices start at zero and increase by one. Only node zero has no parent.
# Parents must already exist and be open sequence/parallel nodes.
uint32 node_index
bool has_parent
uint32 parent_index
# children must be empty: Executor reconstructs them in arrival order.
# do carries a complete CapabilityCall and never receives CLOSE.
pilot/RtdlNode node

# CLOSE uses node_index, exactly once per operator, innermost first.
# END requires a root and all operators closed. Invalid input terminates with
# invalid_argument; capability failures remain ordinary RtdlEvent messages.
