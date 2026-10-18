// SPDX-License-Identifier: MulanPSL-2.0
// Author: wheatfox <wheatfox17@icloud.com>
//
// The tasks of a session.
//
// The user's messages arrive one at a time. A task is one piece of work the
// user asked for: the messages about it (the request, then every correction,
// addition or answer), the goal and success criterion the model has drawn from
// them, its status, and the RTDL trees dispatched for it. Several tasks can be
// open at once and progress concurrently; every tree belongs to exactly one.
//
// Pilot owns identity and facts: message and task ids, the user's words, which
// task each message and tree is filed under, and what the trees did. The model
// owns the judgement, through three reply fields: `messages` (which task each
// new message belongs to), `task` (which task the reply's tree serves) and
// `task_updates` (goal, success criterion and status). Everything is matched by
// id, never by comparing text.
//
// A streaming planner learns a reply in order, so the fields that dispatch
// needs (`messages`, `task`) come before `rtdl` and the ones that need to know
// what was dispatched (`task_updates`) after it; the calls below follow that
// order: `file_messages`, then `attach_tree`, then `apply_updates`.

use anyhow::{Result, bail};
use serde::{Deserialize, Serialize};
use serde_json::{Map, Value};

pub(crate) const IN_PROGRESS: &str = "in_progress";
pub(crate) const DONE: &str = "done";
pub(crate) const FAILED: &str = "failed";
pub(crate) const SUPERSEDED: &str = "superseded";
const STATUSES: [&str; 4] = [IN_PROGRESS, DONE, FAILED, SUPERSEDED];

/// The reply fields this module reads, in the order a reply should give them
/// relative to `rtdl`: `messages` and `task` before it, `task_updates` after.
/// `task_update` is the single-task object of the previous contract.
pub(crate) const REPLY_FIELDS: [&str; 4] = ["messages", "task", "task_updates", "task_update"];
/// What a reply writes for a task that the same reply creates.
const NEW: &str = "new";

pub(crate) const DEFAULT_SUCCESS_CRITERION: &str =
    "The user's request is completed and the result has been verified.";

/// Closed tasks the status board lists, newest first; the transcript keeps all.
const CLOSED_TASKS_SHOWN: usize = 5;
const CLOSED_GOAL_CHARS: usize = 80;

const TREE_RUNNING: &str = "running";

/// One user message, verbatim.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub(crate) struct UserMessage {
    /// `m1`, `m2`, … in arrival order.
    pub id: String,
    pub text: String,
    /// The task it is filed under.
    pub task: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub(crate) struct Task {
    /// `T1`, `T2`, … never reused.
    pub id: String,
    /// The model's statement of what the task is, from its messages; empty
    /// until the model writes one.
    pub goal: String,
    pub success_criterion: String,
    /// `in_progress`, `done`, `failed` or `superseded`.
    pub status: String,
    /// Why the model closed the task.
    pub status_reason: String,
}

impl Task {
    pub(crate) fn is_open(&self) -> bool {
        self.status == IN_PROGRESS
    }
}

/// One RTDL tree and the task it belongs to.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub(crate) struct Tree {
    pub plan_id: String,
    pub task: String,
    pub description: String,
    /// `running`, then `succeeded`, `failed` or `canceled`.
    pub state: String,
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub(crate) struct TaskList {
    pub messages: Vec<UserMessage>,
    pub tasks: Vec<Task>,
    pub trees: Vec<Tree>,
    next_task: u64,
    /// `messages[answered..]` arrived after the model's last reply; only these
    /// may be refiled.
    answered: usize,
}

/// A task as a reply names it: an existing id, or the task this reply creates.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum TaskRef {
    Id(String),
    New,
}

/// What a reply says about tasks.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub(crate) struct TaskFields {
    /// Refiled messages: message id and the task it belongs to.
    pub messages: Vec<(String, TaskRef)>,
    /// The task this reply's tree serves; `None` when the reply dispatches no
    /// tree, or did not say.
    pub task: Option<TaskRef>,
    pub updates: Vec<TaskUpdate>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct TaskUpdate {
    /// `None` only for the previous contract's `task_update`: the newest open
    /// task.
    pub task: Option<TaskRef>,
    pub goal: Option<String>,
    pub success_criterion: Option<String>,
    pub status: Option<String>,
    pub status_reason: Option<String>,
}

/// The outcome of one step of applying a reply.
#[derive(Debug, Default, PartialEq, Eq)]
pub(crate) struct Applied {
    /// Tasks whose state changed, for one history record each.
    pub changed: Vec<String>,
    /// One sentence per rejected item, for the model.
    pub rejected: Vec<String>,
}

impl Applied {
    fn touch(&mut self, task: &str) {
        if !self.changed.iter().any(|id| id == task) {
            self.changed.push(task.to_string());
        }
    }
}

/// The single task state of transcripts written before task lists.
#[derive(Deserialize)]
struct LegacyTaskState {
    goal: String,
    success_criterion: String,
    status: String,
}

impl TaskList {
    /// The list a transcript recorded: a task list, or the single task state of
    /// a transcript written before task lists, which becomes a one-task list.
    pub(crate) fn from_record(value: Value) -> Option<Self> {
        if value.get("tasks").is_some() {
            return serde_json::from_value(value).ok();
        }
        let legacy: LegacyTaskState = serde_json::from_value(value).ok()?;
        let mut list = Self::default();
        let (_, task_id) = list.receive(&legacy.goal)?;
        list.answered();
        let task = list.task_mut(&task_id)?;
        task.success_criterion = legacy.success_criterion;
        task.status = if STATUSES.contains(&legacy.status.as_str()) {
            legacy.status
        } else {
            IN_PROGRESS.to_string()
        };
        Some(list)
    }

    fn task(&self, id: &str) -> Option<&Task> {
        self.tasks.iter().find(|task| task.id == id)
    }

    fn task_mut(&mut self, id: &str) -> Option<&mut Task> {
        self.tasks.iter_mut().find(|task| task.id == id)
    }

    fn new_task(&mut self) -> String {
        self.next_task += 1;
        let id = format!("T{}", self.next_task);
        self.tasks.push(Task {
            id: id.clone(),
            goal: String::new(),
            success_criterion: DEFAULT_SUCCESS_CRITERION.to_string(),
            status: IN_PROGRESS.to_string(),
            status_reason: String::new(),
        });
        id
    }

    /// The newest task still in progress.
    pub(crate) fn newest_open(&self) -> Option<&Task> {
        self.tasks.iter().rev().find(|task| task.is_open())
    }

    pub(crate) fn is_open(&self, id: &str) -> bool {
        self.task(id).is_some_and(Task::is_open)
    }

    /// The task of the latest user message, which that message's caller waits on.
    pub(crate) fn latest_message_task(&self) -> Option<&Task> {
        self.task(&self.messages.last()?.task)
    }

    /// The model's goal, or the user's first words until it has written one.
    pub(crate) fn goal_or_words(&self, task: &Task) -> String {
        if !task.goal.is_empty() {
            return task.goal.clone();
        }
        self.messages
            .iter()
            .find(|message| message.task == task.id)
            .map(|message| message.text.clone())
            .unwrap_or_default()
    }

    pub(crate) fn message_ids(&self, task: &str) -> Vec<String> {
        self.messages
            .iter()
            .filter(|m| m.task == task)
            .map(|m| m.id.clone())
            .collect()
    }

    pub(crate) fn plan_ids(&self, task: &str) -> Vec<String> {
        self.trees
            .iter()
            .filter(|t| t.task == task)
            .map(|t| t.plan_id.clone())
            .collect()
    }

    /// The messages of open tasks, which compaction keeps verbatim.
    pub(crate) fn open_message_ids(&self) -> Vec<String> {
        self.messages
            .iter()
            .filter(|message| self.is_open(&message.task))
            .map(|message| message.id.clone())
            .collect()
    }

    /// Whether any task is still in progress.
    pub(crate) fn any_open(&self) -> bool {
        self.tasks.iter().any(Task::is_open)
    }

    /// Take in a user message (a request or a steer). It is filed under the
    /// newest open task, since corrections and answers are the common case, or
    /// under a new task when none is open; the model may refile it in its next
    /// reply. Returns the message id and its task; empty text is ignored.
    pub(crate) fn receive(&mut self, text: &str) -> Option<(String, String)> {
        let text = text.trim();
        if text.is_empty() {
            return None;
        }
        let task = match self.newest_open() {
            Some(task) => task.id.clone(),
            None => self.new_task(),
        };
        let id = format!("m{}", self.messages.len() + 1);
        self.messages.push(UserMessage {
            id: id.clone(),
            text: text.to_string(),
            task: task.clone(),
        });
        Some((id, task))
    }

    /// The model has replied to every message so far: their filing is final.
    pub(crate) fn answered(&mut self) {
        self.answered = self.messages.len();
    }

    /// Messages the model has not replied to yet.
    pub(crate) fn unanswered(&self) -> &[UserMessage] {
        &self.messages[self.answered.min(self.messages.len())..]
    }

    /// Refile new messages as a reply says, creating the reply's new task when
    /// a message goes to `new`. Returns the new task's id, if one was made.
    pub(crate) fn file_messages(
        &mut self,
        moves: &[(String, TaskRef)],
        applied: &mut Applied,
    ) -> Option<String> {
        let mut new_task: Option<String> = None;
        let mut seen: Vec<&str> = Vec::new();
        for (message, target) in moves {
            if seen.contains(&message.as_str()) {
                continue;
            }
            seen.push(message);
            let Some(index) = self.unanswered_index(message) else {
                applied.rejected.push(format!(
                    "message {message} is not one of the new messages ({}) and cannot be refiled",
                    self.unanswered_ids()
                ));
                continue;
            };
            let from = self.messages[index].task.clone();
            let to = match target {
                TaskRef::New => match &new_task {
                    Some(id) => id.clone(),
                    None => {
                        let id = self.reuse_or_create_for(moves, &from);
                        new_task = Some(id.clone());
                        id
                    }
                },
                TaskRef::Id(id) => match self.task(id) {
                    Some(task) if task.is_open() => id.clone(),
                    Some(task) => {
                        applied.rejected.push(format!(
                            "message {message} cannot be filed under {id}, which is already {}",
                            task.status
                        ));
                        continue;
                    }
                    None => {
                        applied.rejected.push(format!(
                            "message {message} cannot be filed under {id}, which does not exist; open tasks: {}",
                            self.open_ids()
                        ));
                        continue;
                    }
                },
            };
            if from != to {
                self.messages[index].task = to.clone();
                applied.touch(&from);
                applied.touch(&to);
            }
        }
        self.drop_empty_tasks();
        new_task
    }

    /// The task that messages going to `new` become. When every message of the
    /// task they were filed under by default goes to `new`, and that task has
    /// done nothing yet, it already is that new task.
    fn reuse_or_create_for(&mut self, moves: &[(String, TaskRef)], from: &str) -> String {
        let going_new: Vec<&str> = moves
            .iter()
            .filter(|(_, target)| *target == TaskRef::New)
            .map(|(message, _)| message.as_str())
            .collect();
        let only_these = self
            .messages
            .iter()
            .filter(|message| message.task == from)
            .all(|message| going_new.contains(&message.id.as_str()));
        let fresh = self.task(from).is_some_and(|task| task.goal.is_empty())
            && !self.trees.iter().any(|tree| tree.task == from);
        if only_these && fresh {
            from.to_string()
        } else {
            self.new_task()
        }
    }

    /// Tasks left with no message and no tree after refiling never started.
    fn drop_empty_tasks(&mut self) {
        let messages = &self.messages;
        let trees = &self.trees;
        self.tasks.retain(|task| {
            messages.iter().any(|message| message.task == task.id)
                || trees.iter().any(|tree| tree.task == task.id)
                || !task.goal.is_empty()
        });
    }

    fn unanswered_index(&self, message: &str) -> Option<usize> {
        (self.answered..self.messages.len()).find(|&i| self.messages[i].id == message)
    }

    fn unanswered_ids(&self) -> String {
        let ids: Vec<&str> = self.unanswered().iter().map(|m| m.id.as_str()).collect();
        if ids.is_empty() {
            "none".to_string()
        } else {
            ids.join(", ")
        }
    }

    fn open_ids(&self) -> String {
        let ids: Vec<&str> = self
            .tasks
            .iter()
            .filter(|t| t.is_open())
            .map(|t| t.id.as_str())
            .collect();
        if ids.is_empty() {
            "none".to_string()
        } else {
            ids.join(", ")
        }
    }

    /// The task a reply's tree belongs to: the one the reply named, which must
    /// be open, or the reply's new task. `Err` says why the name is unusable.
    pub(crate) fn tree_owner(
        &self,
        named: &TaskRef,
        new_task: Option<&str>,
    ) -> Result<String, String> {
        match named {
            TaskRef::New => new_task.map(str::to_string).ok_or_else(|| {
                "`task` is \"new\" but no message was filed under a new task".to_string()
            }),
            TaskRef::Id(id) => match self.task(id) {
                Some(task) if task.is_open() => Ok(id.clone()),
                Some(task) => Err(format!(
                    "`task` names {id}, which is already {}",
                    task.status
                )),
                None => Err(format!(
                    "`task` names {id}, which does not exist; open tasks: {}",
                    self.open_ids()
                )),
            },
        }
    }

    pub(crate) fn attach_tree(&mut self, task: &str, plan_id: &str, description: &str) {
        self.trees.push(Tree {
            plan_id: plan_id.to_string(),
            task: task.to_string(),
            description: description.to_string(),
            state: TREE_RUNNING.to_string(),
        });
    }

    /// Record how a tree ended (`succeeded`, `failed` or `canceled`) and return
    /// its task.
    pub(crate) fn tree_finished(&mut self, plan_id: &str, state: &str) -> Option<String> {
        let tree = self.trees.iter_mut().find(|tree| tree.plan_id == plan_id)?;
        tree.state = state.to_string();
        Some(tree.task.clone())
    }

    pub(crate) fn task_of_tree(&self, plan_id: &str) -> Option<&str> {
        self.trees
            .iter()
            .find(|tree| tree.plan_id == plan_id)
            .map(|tree| tree.task.as_str())
    }

    fn running_trees(&self, task: &str) -> Vec<&str> {
        self.trees
            .iter()
            .filter(|tree| tree.task == task && tree.state == TREE_RUNNING)
            .map(|tree| tree.plan_id.as_str())
            .collect()
    }

    /// Apply a reply's task updates, item by item: a rejected item is reported
    /// and the others still apply. Call after the reply's tree is attached, so
    /// that it holds its task open.
    pub(crate) fn apply_updates(
        &mut self,
        updates: &[TaskUpdate],
        new_task: Option<&str>,
        applied: &mut Applied,
    ) {
        for update in updates {
            let id = match &update.task {
                None => match self.newest_open() {
                    Some(task) => task.id.clone(),
                    None => {
                        applied.rejected.push(
                            "a task update named no task and none is in progress".to_string(),
                        );
                        continue;
                    }
                },
                Some(named) => match self.tree_owner(named, new_task) {
                    Ok(id) => id,
                    Err(problem) => {
                        applied
                            .rejected
                            .push(problem.replace("`task`", "a task update"));
                        continue;
                    }
                },
            };
            if let Some(status) = update.status.as_deref()
                && (status == DONE || status == FAILED)
            {
                let running = self.running_trees(&id);
                if !running.is_empty() {
                    applied.rejected.push(format!(
                        "{id} cannot be {status} while its tree(s) {} still run; close it after they finish",
                        running.join(", ")
                    ));
                    continue;
                }
            }
            let Some(task) = self.task_mut(&id) else {
                continue;
            };
            let before = task.clone();
            if let Some(goal) = update
                .goal
                .as_deref()
                .map(str::trim)
                .filter(|goal| !goal.is_empty())
            {
                task.goal = goal.to_string();
            }
            if let Some(criterion) = update
                .success_criterion
                .as_deref()
                .map(str::trim)
                .filter(|c| !c.is_empty())
            {
                task.success_criterion = criterion.to_string();
            }
            if let Some(status) = update
                .status
                .as_deref()
                .filter(|status| *status != IN_PROGRESS)
            {
                task.status = status.to_string();
                task.status_reason = update
                    .status_reason
                    .as_deref()
                    .unwrap_or("")
                    .trim()
                    .to_string();
            }
            if *task != before {
                applied.touch(&id);
            }
        }
    }

    /// How a user message enters history: once, with its id and task.
    pub(crate) fn message_line(&self, message_id: &str) -> String {
        let Some(message) = self
            .messages
            .iter()
            .find(|message| message.id == message_id)
        else {
            return String::new();
        };
        format!(
            "User message {} (filed under {}): {}",
            message.id, message.task, message.text
        )
    }

    /// A task's current state as one history line, written when it changes.
    pub(crate) fn record(&self, task_id: &str) -> Option<String> {
        let task = self.task(task_id)?;
        let messages: Vec<&str> = self
            .messages
            .iter()
            .filter(|message| message.task == task.id)
            .map(|message| message.id.as_str())
            .collect();
        let plans: Vec<Value> = self
            .trees
            .iter()
            .filter(|tree| tree.task == task.id)
            .map(|tree| serde_json::json!({"plan_id": tree.plan_id, "state": tree.state}))
            .collect();
        let record = serde_json::json!({
            "task": task.id,
            "goal": task.goal,
            "success_criterion": task.success_criterion,
            "status": task.status,
            "status_reason": task.status_reason,
            "messages": messages,
            "plans": plans,
        });
        Some(format!(
            "Pilot task record (authoritative state, not a user instruction): {record}"
        ))
    }

    /// The status board: the tail of every planning request, rebuilt each
    /// round and never written into history. Open tasks in full, new messages
    /// with their default filing, and the most recent closed tasks.
    pub(crate) fn board(&self) -> String {
        if self.tasks.is_empty() {
            return String::new();
        }
        let mut out = String::from("\n\n## Tasks (current status board)\n");
        let open: Vec<&Task> = self.tasks.iter().filter(|task| task.is_open()).collect();
        if open.is_empty() {
            out.push_str("No task is in progress.\n");
        }
        for task in open {
            let goal = if task.goal.is_empty() {
                "(not stated yet)"
            } else {
                task.goal.as_str()
            };
            out.push_str(&format!(
                "- {}: in_progress\n  goal: {goal}\n  success criterion: {}\n",
                task.id, task.success_criterion
            ));
            let messages: Vec<&str> = self
                .messages
                .iter()
                .filter(|message| message.task == task.id)
                .map(|message| message.id.as_str())
                .collect();
            out.push_str(&format!("  messages: {}\n", messages.join(", ")));
            let trees: Vec<&Tree> = self
                .trees
                .iter()
                .filter(|tree| tree.task == task.id)
                .collect();
            if trees.is_empty() {
                out.push_str("  plans: none\n");
            }
            for tree in trees {
                out.push_str(&format!(
                    "  plan {} ({}): {}\n",
                    tree.plan_id, tree.state, tree.description
                ));
            }
        }
        let unanswered = self.unanswered();
        if !unanswered.is_empty() {
            out.push_str("New messages since your last reply, filed by default; refile one in `messages` if it belongs to another open task or starts a new one:\n");
            for message in unanswered {
                out.push_str(&format!(
                    "- {} → {}: {}\n",
                    message.id, message.task, message.text
                ));
            }
        }
        let closed: Vec<&Task> = self
            .tasks
            .iter()
            .rev()
            .filter(|task| !task.is_open())
            .take(CLOSED_TASKS_SHOWN)
            .collect();
        if !closed.is_empty() {
            out.push_str("Recently closed, newest first:\n");
            for task in closed {
                let goal: String = task.goal.chars().take(CLOSED_GOAL_CHARS).collect();
                let more = if task.goal.chars().count() > CLOSED_GOAL_CHARS {
                    "…"
                } else {
                    ""
                };
                let reason = if task.status_reason.is_empty() {
                    String::new()
                } else {
                    format!(" ({})", task.status_reason)
                };
                out.push_str(&format!(
                    "- {} {}{reason}: {goal}{more}\n",
                    task.id, task.status
                ));
            }
        }
        out
    }
}

fn task_ref(value: &Value, field: &str) -> Result<Option<TaskRef>> {
    let id = match value {
        Value::Null => return Ok(None),
        Value::String(text) => text.trim().to_string(),
        Value::Number(number) => number.to_string(),
        _ => bail!("`{field}` must be a task id, \"new\" or null"),
    };
    if id.is_empty() {
        return Ok(None);
    }
    if id.eq_ignore_ascii_case(NEW) {
        return Ok(Some(TaskRef::New));
    }
    // A bare number is the same task as its T-prefixed id.
    let id = if id.chars().all(|c| c.is_ascii_digit()) {
        format!("T{id}")
    } else {
        id
    };
    Ok(Some(TaskRef::Id(id)))
}

fn message_id(value: &Value, field: &str) -> Result<String> {
    let id = match value {
        Value::String(text) => text.trim().to_string(),
        Value::Number(number) => number.to_string(),
        _ => bail!("`{field}` must be a message id"),
    };
    Ok(if id.chars().all(|c| c.is_ascii_digit()) {
        format!("m{id}")
    } else {
        id
    })
}

fn string(value: Option<&Value>, field: &str) -> Result<Option<String>> {
    match value {
        None | Some(Value::Null) => Ok(None),
        Some(Value::String(text)) => Ok(Some(text.clone())),
        Some(_) => bail!("`{field}` must be a string"),
    }
}

fn status(value: Option<&Value>, field: &str) -> Result<Option<String>> {
    let status = string(value, field)?;
    if let Some(status) = &status
        && !STATUSES.contains(&status.as_str())
    {
        bail!("`{field}` must be one of {}", STATUSES.join(", "));
    }
    Ok(status)
}

fn object<'a>(value: &'a Value, field: &str, keys: &[&str]) -> Result<&'a Map<String, Value>> {
    let Some(object) = value.as_object() else {
        bail!("`{field}` must be an object");
    };
    if let Some(key) = object.keys().find(|key| !keys.contains(&key.as_str())) {
        bail!("`{field}` has an unknown field `{key}`");
    }
    Ok(object)
}

/// The task fields of a reply envelope. Missing fields are empty, so a
/// streaming parser can call this on the fields it has so far.
pub(crate) fn parse_task_fields(envelope: &Map<String, Value>) -> Result<TaskFields> {
    let mut fields = TaskFields::default();
    match envelope.get("messages") {
        None | Some(Value::Null) => {}
        Some(Value::Array(items)) => {
            for (i, item) in items.iter().enumerate() {
                let field = format!("messages[{i}]");
                let item = object(item, &field, &["message", "task"])?;
                let message = message_id(
                    item.get("message").unwrap_or(&Value::Null),
                    &format!("{field}.message"),
                )?;
                let Some(task) = task_ref(
                    item.get("task").unwrap_or(&Value::Null),
                    &format!("{field}.task"),
                )?
                else {
                    bail!("`{field}.task` must name a task id or \"new\"");
                };
                fields.messages.push((message, task));
            }
        }
        Some(_) => bail!("`messages` must be an array"),
    }
    if let Some(value) = envelope.get("task") {
        fields.task = task_ref(value, "task")?;
    }
    match envelope.get("task_updates") {
        None | Some(Value::Null) => {}
        Some(Value::Array(items)) => {
            for (i, item) in items.iter().enumerate() {
                let field = format!("task_updates[{i}]");
                let item = object(
                    item,
                    &field,
                    &[
                        "task",
                        "goal",
                        "success_criterion",
                        "status",
                        "status_reason",
                    ],
                )?;
                let Some(task) = task_ref(
                    item.get("task").unwrap_or(&Value::Null),
                    &format!("{field}.task"),
                )?
                else {
                    bail!("`{field}.task` must name a task id or \"new\"");
                };
                fields.updates.push(TaskUpdate {
                    task: Some(task),
                    goal: string(item.get("goal"), &format!("{field}.goal"))?,
                    success_criterion: string(
                        item.get("success_criterion"),
                        &format!("{field}.success_criterion"),
                    )?,
                    status: status(item.get("status"), &format!("{field}.status"))?,
                    status_reason: string(
                        item.get("status_reason"),
                        &format!("{field}.status_reason"),
                    )?,
                });
            }
        }
        Some(_) => bail!("`task_updates` must be an array"),
    }
    // The previous contract's single update: the newest open task, its `goal`
    // ignored (Pilot keeps the user's words; a summary in it is not a goal).
    match envelope.get("task_update") {
        None | Some(Value::Null) => {}
        Some(value) => {
            let legacy = object(
                value,
                "task_update",
                &["goal", "success_criterion", "status"],
            )?;
            fields.updates.push(TaskUpdate {
                task: None,
                goal: None,
                success_criterion: string(
                    legacy.get("success_criterion"),
                    "task_update.success_criterion",
                )?,
                status: status(legacy.get("status"), "task_update.status")?,
                status_reason: None,
            });
        }
    }
    Ok(fields)
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn fields(value: Value) -> Result<TaskFields> {
        parse_task_fields(value.as_object().unwrap())
    }

    fn update(task: &str, status: &str) -> TaskUpdate {
        TaskUpdate {
            task: Some(TaskRef::Id(task.to_string())),
            goal: None,
            success_criterion: None,
            status: Some(status.to_string()),
            status_reason: None,
        }
    }

    fn messages_of<'a>(list: &'a TaskList, task: &str) -> Vec<&'a str> {
        list.messages
            .iter()
            .filter(|m| m.task == task)
            .map(|m| m.id.as_str())
            .collect()
    }

    /// One reply applied in streaming order.
    fn reply(list: &mut TaskList, value: Value, plan: Option<&str>) -> Applied {
        let fields = fields(value).unwrap();
        let mut applied = Applied::default();
        let new_task = list.file_messages(&fields.messages, &mut applied);
        if let Some(plan) = plan {
            let owner = match &fields.task {
                Some(named) => list.tree_owner(named, new_task.as_deref()).unwrap(),
                None => list.newest_open().unwrap().id.clone(),
            };
            list.attach_tree(&owner, plan, "tree");
        }
        list.apply_updates(&fields.updates, new_task.as_deref(), &mut applied);
        list.answered();
        applied
    }

    #[test]
    fn corrections_and_answers_stay_in_the_task_by_default() {
        let mut list = TaskList::default();
        list.receive("bring the red cup");
        reply(
            &mut list,
            json!({"task": "T1", "task_updates": [{"task": "T1", "goal": "bring the red cup"}]}),
            Some("p1"),
        );
        list.receive("no, the blue one");
        reply(
            &mut list,
            json!({"task_updates": [{"task": "T1", "goal": "bring the blue cup"}]}),
            None,
        );
        assert_eq!(list.tasks.len(), 1);
        assert_eq!(messages_of(&list, "T1"), ["m1", "m2"]);
        assert_eq!(list.tasks[0].goal, "bring the blue cup");
    }

    #[test]
    fn a_new_message_can_start_a_new_task_with_its_tree_and_status_in_the_same_reply() {
        let mut list = TaskList::default();
        list.receive("patrol the meeting room");
        reply(&mut list, json!({"task": "T1"}), Some("p1"));
        list.receive("what time is it");
        let applied = reply(
            &mut list,
            json!({
                "messages": [{"message": "m2", "task": "new"}],
                "task_updates": [{"task": "new", "goal": "tell the time", "status": "done", "status_reason": "answered"}]
            }),
            None,
        );
        assert!(applied.rejected.is_empty(), "{:?}", applied.rejected);
        assert_eq!(messages_of(&list, "T1"), ["m1"]);
        assert_eq!(messages_of(&list, "T2"), ["m2"]);
        assert_eq!(list.task("T2").unwrap().status, DONE);
        assert!(list.task("T1").unwrap().is_open());
    }

    #[test]
    fn a_message_can_go_to_an_older_open_task() {
        let mut list = TaskList::default();
        list.receive("bring the cup");
        reply(&mut list, json!({}), Some("p1"));
        list.receive("open the window");
        reply(
            &mut list,
            json!({"messages": [{"message": "m2", "task": "new"}], "task": "new"}),
            Some("p2"),
        );
        list.receive("make the cup the blue one");
        let applied = reply(
            &mut list,
            json!({"messages": [{"message": 3, "task": 1}]}),
            None,
        );
        assert!(applied.rejected.is_empty(), "{:?}", applied.rejected);
        assert_eq!(messages_of(&list, "T1"), ["m1", "m3"]);
        assert_eq!(list.task_of_tree("p2"), Some("T2"));
    }

    #[test]
    fn only_new_messages_can_be_refiled_and_only_to_open_tasks() {
        let mut list = TaskList::default();
        list.receive("one");
        reply(
            &mut list,
            json!({"task_updates": [{"task": "T1", "status": "done"}]}),
            None,
        );
        list.receive("two");
        let applied = reply(
            &mut list,
            json!({"messages": [{"message": "m1", "task": "new"}, {"message": "m2", "task": "T1"}]}),
            None,
        );
        assert_eq!(applied.rejected.len(), 2, "{:?}", applied.rejected);
        assert!(applied.rejected[0].contains("m2"));
    }

    #[test]
    fn a_message_alone_in_its_default_task_becomes_the_new_task_without_an_empty_one() {
        let mut list = TaskList::default();
        list.receive("first");
        reply(
            &mut list,
            json!({"task_updates": [{"task": "T1", "status": "done"}]}),
            None,
        );
        list.receive("second");
        reply(
            &mut list,
            json!({"messages": [{"message": "m2", "task": "new"}]}),
            None,
        );
        assert_eq!(list.tasks.len(), 2);
        assert_eq!(messages_of(&list, "T2"), ["m2"]);
    }

    #[test]
    fn done_while_a_tree_of_the_task_runs_is_rejected_for_that_item_only() {
        let mut list = TaskList::default();
        list.receive("navigate");
        reply(&mut list, json!({"task": "T1"}), Some("p1"));
        list.receive("say hello");
        let applied = reply(
            &mut list,
            json!({
                "messages": [{"message": "m2", "task": "new"}],
                "task_updates": [{"task": "T1", "status": "done"}, {"task": "new", "status": "done"}]
            }),
            None,
        );
        assert_eq!(applied.rejected.len(), 1);
        assert!(applied.rejected[0].contains("p1"));
        assert!(list.task("T1").unwrap().is_open());
        assert_eq!(list.task("T2").unwrap().status, DONE);
        assert_eq!(list.tree_finished("p1", "succeeded").as_deref(), Some("T1"));
        let mut applied = Applied::default();
        list.apply_updates(&[update("T1", DONE)], None, &mut applied);
        assert!(applied.rejected.is_empty());
    }

    #[test]
    fn a_task_cannot_close_in_the_reply_that_dispatches_its_tree() {
        let mut list = TaskList::default();
        list.receive("look around");
        let applied = reply(
            &mut list,
            json!({"task": "T1", "task_updates": [{"task": "T1", "status": "done"}]}),
            Some("p1"),
        );
        assert_eq!(applied.rejected.len(), 1);
        assert!(list.task("T1").unwrap().is_open());
    }

    #[test]
    fn an_instruction_longer_than_four_hundred_characters_can_be_done() {
        let mut list = TaskList::default();
        list.receive(&"check every shelf in the kitchen ".repeat(20));
        assert!(list.messages[0].text.chars().count() > 400);
        let applied = reply(
            &mut list,
            json!({"task_updates": [{"task": "T1", "status": "done"}]}),
            None,
        );
        assert!(applied.rejected.is_empty());
        assert!(!list.any_open());
    }

    #[test]
    fn the_legacy_task_update_applies_to_the_newest_open_task_without_its_goal() {
        let mut list = TaskList::default();
        list.receive("the user's own words");
        reply(
            &mut list,
            json!({"task_update": {"goal": "a summary", "success_criterion": "c", "status": "done"}}),
            None,
        );
        let task = list.task("T1").unwrap();
        assert_eq!(task.status, DONE);
        assert_eq!(task.goal, "");
        assert_eq!(task.success_criterion, "c");
    }

    #[test]
    fn task_fields_accept_ids_as_strings_or_integers_and_reject_bad_shapes() {
        let parsed =
            fields(json!({"task": 2, "messages": [{"message": 7, "task": "NEW"}]})).unwrap();
        assert_eq!(parsed.task, Some(TaskRef::Id("T2".into())));
        assert_eq!(parsed.messages, [("m7".to_string(), TaskRef::New)]);
        assert!(fields(json!({"task": {}})).is_err());
        assert!(fields(json!({"messages": [{"message": "m1"}]})).is_err());
        assert!(fields(json!({"task_updates": [{"status": "done"}]})).is_err());
        assert!(fields(json!({"task_updates": [{"task": "T1", "status": "finished"}]})).is_err());
        assert!(fields(json!({"task_updates": [{"task": "T1", "extra": 1}]})).is_err());
        assert_eq!(
            fields(json!({"task": null, "messages": [], "task_updates": []})).unwrap(),
            TaskFields::default()
        );
    }

    #[test]
    fn unknown_and_closed_tasks_are_reported_with_the_open_ones() {
        let mut list = TaskList::default();
        list.receive("one");
        assert!(
            list.tree_owner(&TaskRef::Id("T9".into()), None)
                .unwrap_err()
                .contains("T1")
        );
        assert!(list.tree_owner(&TaskRef::New, None).is_err());
        let mut applied = Applied::default();
        list.apply_updates(
            &[update("T1", SUPERSEDED), update("T1", DONE)],
            None,
            &mut applied,
        );
        assert_eq!(applied.rejected.len(), 1);
        assert_eq!(list.task("T1").unwrap().status, SUPERSEDED);
    }

    #[test]
    fn an_old_single_task_state_restores_as_a_one_task_list() {
        let list = TaskList::from_record(json!({
            "goal": "fetch the cup", "success_criterion": "cup delivered", "status": "in_progress"
        }))
        .unwrap();
        assert_eq!(list.messages[0].text, "fetch the cup");
        assert_eq!(list.tasks[0].success_criterion, "cup delivered");
        assert!(list.unanswered().is_empty());
        let value = serde_json::to_value(&list).unwrap();
        assert_eq!(TaskList::from_record(value).unwrap(), list);
    }

    #[test]
    fn records_and_the_board_name_tasks_messages_and_plans_by_id() {
        let mut list = TaskList::default();
        list.receive("old request");
        reply(
            &mut list,
            json!({"task_updates": [{"task": "T1", "goal": "old", "status": "done"}]}),
            None,
        );
        list.receive("go to the kitchen");
        reply(
            &mut list,
            json!({"task_updates": [{"task": "T2", "goal": "reach the kitchen"}]}),
            Some("p4"),
        );
        list.receive("the one upstairs");
        assert_eq!(
            list.message_line("m3"),
            "User message m3 (filed under T2): the one upstairs"
        );
        let record = list.record("T2").unwrap();
        assert!(record.contains(r#""messages":["m2","m3"]"#), "{record}");
        assert!(record.contains(r#""plan_id":"p4""#));
        let board = list.board();
        assert!(
            board.contains("- T2: in_progress\n  goal: reach the kitchen"),
            "{board}"
        );
        assert!(board.contains("plan p4 (running): tree"));
        assert!(board.contains("- m3 → T2: the one upstairs"));
        assert!(board.contains("- T1 done: old"));
    }
}
