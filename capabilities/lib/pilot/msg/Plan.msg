# Plan: structural payload pilot hands to executor for execution.
# Encodes an RTDL execution tree as arena nodes. Empty plans are represented
# as one sequence root node with no children.
string plan_id
string session_id
uint32 round
RtdlNode[] nodes
uint32 root_index
# Internal-only credential copied from Task. Pilot clears it from every Plan
# event sent to clients; Executor consumes it once to obtain canonical identity
# from Keystone and clears it before running the tree.
string auth_session_token
