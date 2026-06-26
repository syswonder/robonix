# event_kind: 0=text_chunk 1=plan 2=batch_result 3=status 4=final_text
#             5=node_state 6=task_state
uint32 event_kind
string session_id
string text_chunk
Plan plan
BatchResult batch_result
SessionStatusEvent status
string final_text
RtdlNodeState node_state
TaskStateEvent task_state
