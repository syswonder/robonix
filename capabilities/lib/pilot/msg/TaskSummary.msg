# One task of a Pilot session, as TaskStateEvent lists them.
# status: "in_progress" | "done" | "failed" | "superseded".
string task_id
string goal
string success_criterion
string status
string status_reason
# Ids of the user messages filed under this task, in order.
string[] message_ids
# Plan ids of the RTDL trees dispatched for this task, in order.
string[] plan_ids
