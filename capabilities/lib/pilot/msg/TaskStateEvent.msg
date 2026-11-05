# The pilot's tasks, forwarded to the chat for live display.
# goal, success_criterion and status describe the task of the latest user
# message, as before task lists; status is "in_progress" | "done" | "failed" |
# "superseded" (empty until there is a task). Fields are positional: add new
# ones at the end only.
string goal
string success_criterion
string status
# The id of that task.
string task_id
# Every task of the session, oldest first.
TaskSummary[] tasks
