# event_kind: 0=text_chunk 1=task_graph 2=batch_result 3=status 4=final_text
uint32 event_kind
string session_id
string text_chunk
TaskGraph task_graph
BatchResult batch_result
SessionStatusEvent status
string final_text
