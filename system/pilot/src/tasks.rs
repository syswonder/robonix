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
// owns the judgement, through three reply fields: `message_updates` (which task
// each new message belongs to), `rtdl_parent_task_id` (which task the reply's tree
// serves) and `task_updates` (goal, success criterion and status). A task the
// reply creates is named "new" there, and Pilot gives it its id. Everything is matched by
// id, never by comparing text.
//
// A streaming planner learns a reply in order, so the fields that dispatch
// needs (`message_updates`, `rtdl_parent_task_id`) come before `rtdl` and the ones that need to know
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

/// Names the task a reply creates; Pilot gives it its id.
const NEW: &str = "new";
/// The reply fields this module reads, in the order a reply should give them
/// relative to `rtdl`: `message_updates` and `rtdl_parent_task_id` before it,
/// `task_updates` after.
pub(crate) const REPLY_FIELDS: [&str; 3] =
    ["message_updates", "rtdl_parent_task_id", "task_updates"];

pub(crate) const DEFAULT_SUCCESS_CRITERION: &str =
    "The user's request is completed and the result has been verified.";

/// Closed tasks the status board lists, newest first; the transcript keeps all.
const CLOSED_TASKS_SHOWN: usize = 5;
const CLOSED_GOAL_CHARS: usize = 80;

const TREE_RUNNING: &str = "running";
const TREE_LOST: &str = "lost";

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

impl UserMessage {
    /// How the message enters history: once, with its id and task.
    pub(crate) fn line(&self) -> String {
        format!(
            "User message {} (filed under {}): {}",
            self.id, self.task, self.text
        )
    }
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
pub(crate) struct TaskTable {
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
    pub task: TaskRef,
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
    pub(crate) fn touch(&mut self, task: &str) {
        if !self.changed.iter().any(|id| id == task) {
            self.changed.push(task.to_string());
        }
    }
}

impl TaskTable {
    /// The list a transcript recorded.
    pub(crate) fn from_record(value: Value) -> Option<Self> {
        let mut list: Self = serde_json::from_value(value).ok()?;
        // Nothing reports on the trees of a Pilot that restarted.
        for tree in &mut list.trees {
            if tree.state == TREE_RUNNING {
                tree.state = TREE_LOST.to_string();
            }
        }
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
    /// reply. Empty text is ignored.
    pub(crate) fn receive(&mut self, text: &str) -> Option<&UserMessage> {
        let text = text.trim();
        if text.is_empty() {
            return None;
        }
        let task = match self.newest_open() {
            Some(task) => task.id.clone(),
            None => self.new_task(),
        };
        self.messages.push(UserMessage {
            id: format!("m{}", self.messages.len() + 1),
            text: text.to_string(),
            task,
        });
        self.messages.last()
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
                TaskRef::New => new_task
                    .get_or_insert_with(|| self.reuse_or_create_for(moves, &from))
                    .clone(),
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
            .filter(|(message, target)| {
                *target == TaskRef::New && self.unanswered_index(message).is_some()
            })
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
                "`rtdl_parent_task_id` is \"new\" but no message was filed under a new task"
                    .to_string()
            }),
            TaskRef::Id(id) => match self.task(id) {
                Some(task) if task.is_open() => Ok(id.clone()),
                Some(task) => Err(format!(
                    "`rtdl_parent_task_id` names {id}, which is already {}",
                    task.status
                )),
                None => Err(format!(
                    "`rtdl_parent_task_id` names {id}, which does not exist; open tasks: {}",
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
    pub(crate) fn reap_tree(&mut self, plan_id: &str, state: &str) -> Option<String> {
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

    /// Whether an open task other than `task` has no tree running: it waits
    /// for a reply of its own, since a tree serves one task.
    pub(crate) fn others_runnable(&self, task: &str) -> bool {
        self.tasks
            .iter()
            .any(|t| t.is_open() && t.id != task && self.running_trees(&t.id).is_empty())
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
            let id = match self.tree_owner(&update.task, new_task) {
                Ok(id) => id,
                Err(problem) => {
                    applied
                        .rejected
                        .push(problem.replace("`rtdl_parent_task_id`", "a task update"));
                    continue;
                }
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
            let task = self.task_mut(&id).expect("tree_owner found the task");
            let before = task.clone();
            for (value, field) in [
                (&update.goal, &mut task.goal),
                (&update.success_criterion, &mut task.success_criterion),
            ] {
                if let Some(value) = value.as_deref().map(str::trim).filter(|v| !v.is_empty()) {
                    *field = value.to_string();
                }
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

    /// A task's current state as one history line, written when it changes.
    pub(crate) fn record(&self, task_id: &str) -> Option<String> {
        let task = self.task(task_id)?;
        let plans: Vec<Value> = self
            .trees
            .iter()
            .filter(|tree| tree.task == task.id)
            .map(|tree| serde_json::json!({"plan_id": tree.plan_id, "state": tree.state}))
            .collect();
        let record = serde_json::json!({
            "task_id": task.id,
            "goal": task.goal,
            "success_criterion": task.success_criterion,
            "status": task.status,
            "status_reason": task.status_reason,
            "messages": self.message_ids(&task.id),
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
            out.push_str(&format!(
                "  messages: {}\n",
                self.message_ids(&task.id).join(", ")
            ));
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
            out.push_str("New messages since your last reply, filed by default; refile one in `message_updates` if it belongs to another open task or starts a new one:\n");
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
    match envelope.get("message_updates") {
        None | Some(Value::Null) => {}
        Some(Value::Array(items)) => {
            for (i, item) in items.iter().enumerate() {
                let field = format!("message_updates[{i}]");
                let item = object(item, &field, &["message_id", "parent_task_id"])?;
                let message = message_id(
                    item.get("message_id").unwrap_or(&Value::Null),
                    &format!("{field}.message_id"),
                )?;
                let Some(task) = task_ref(
                    item.get("parent_task_id").unwrap_or(&Value::Null),
                    &format!("{field}.parent_task_id"),
                )?
                else {
                    bail!("`{field}.parent_task_id` must name a task id or \"new\"");
                };
                fields.messages.push((message, task));
            }
        }
        Some(_) => bail!("`message_updates` must be an array"),
    }
    if let Some(value) = envelope.get("rtdl_parent_task_id") {
        fields.task = task_ref(value, "rtdl_parent_task_id")?;
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
                        "task_id",
                        "goal",
                        "success_criterion",
                        "status",
                        "status_reason",
                    ],
                )?;
                let Some(task) = task_ref(
                    item.get("task_id").unwrap_or(&Value::Null),
                    &format!("{field}.task_id"),
                )?
                else {
                    bail!("`{field}.task_id` must name a task id or \"new\"");
                };
                fields.updates.push(TaskUpdate {
                    task,
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
            task: TaskRef::Id(task.to_string()),
            goal: None,
            success_criterion: None,
            status: Some(status.to_string()),
            status_reason: None,
        }
    }

    /// One reply applied in streaming order.
    fn reply(list: &mut TaskTable, value: Value, plan: Option<&str>) -> Applied {
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
        let mut list = TaskTable::default();
        list.receive("bring the red cup");
        reply(
            &mut list,
            json!({"rtdl_parent_task_id": "T1", "task_updates": [{"task_id": "T1", "goal": "bring the red cup"}]}),
            Some("p1"),
        );
        list.receive("no, the blue one");
        reply(
            &mut list,
            json!({"task_updates": [{"task_id": "T1", "goal": "bring the blue cup"}]}),
            None,
        );
        assert_eq!(list.tasks.len(), 1);
        assert_eq!(list.message_ids("T1"), ["m1", "m2"]);
        assert_eq!(list.tasks[0].goal, "bring the blue cup");
    }

    #[test]
    fn a_new_message_can_start_a_new_task_with_its_tree_and_status_in_the_same_reply() {
        let mut list = TaskTable::default();
        list.receive("patrol the meeting room");
        reply(&mut list, json!({"rtdl_parent_task_id": "T1"}), Some("p1"));
        list.receive("what time is it");
        let applied = reply(
            &mut list,
            json!({
                "message_updates": [{"message_id": "m2", "parent_task_id": "new"}],
                "task_updates": [{"task_id": "new", "goal": "tell the time", "status": "done", "status_reason": "answered"}]
            }),
            None,
        );
        assert!(applied.rejected.is_empty(), "{:?}", applied.rejected);
        assert_eq!(list.message_ids("T1"), ["m1"]);
        assert_eq!(list.message_ids("T2"), ["m2"]);
        assert_eq!(list.task("T2").unwrap().status, DONE);
        assert!(list.task("T1").unwrap().is_open());
    }

    #[test]
    fn a_task_without_a_running_tree_waits_for_its_own_reply() {
        let mut list = TaskTable::default();
        list.receive("visit three points and take photos");
        reply(&mut list, json!({"rtdl_parent_task_id": "T1"}), Some("p1"));
        list.receive("is there a whiteboard in the meeting room");
        reply(
            &mut list,
            json!({"message_updates": [{"message_id": "m2", "parent_task_id": "new"}]}),
            None,
        );
        assert!(list.others_runnable("T1"));
        reply(&mut list, json!({"rtdl_parent_task_id": "T2"}), Some("p2"));
        assert!(!list.others_runnable("T2"));
        assert!(!list.others_runnable("T1"));
    }

    #[test]
    fn a_message_can_go_to_an_older_open_task() {
        let mut list = TaskTable::default();
        list.receive("bring the cup");
        reply(&mut list, json!({}), Some("p1"));
        list.receive("open the window");
        reply(
            &mut list,
            json!({"message_updates": [{"message_id": "m2", "parent_task_id": "new"}], "rtdl_parent_task_id": "new"}),
            Some("p2"),
        );
        list.receive("make the cup the blue one");
        let applied = reply(
            &mut list,
            json!({"message_updates": [{"message_id": 3, "parent_task_id": 1}]}),
            None,
        );
        assert!(applied.rejected.is_empty(), "{:?}", applied.rejected);
        assert_eq!(list.message_ids("T1"), ["m1", "m3"]);
        assert_eq!(list.task_of_tree("p2"), Some("T2"));
    }

    #[test]
    fn only_new_messages_can_be_refiled_and_only_to_open_tasks() {
        let mut list = TaskTable::default();
        list.receive("one");
        reply(
            &mut list,
            json!({"task_updates": [{"task_id": "T1", "status": "done"}]}),
            None,
        );
        list.receive("two");
        let applied = reply(
            &mut list,
            json!({"message_updates": [{"message_id": "m1", "parent_task_id": "new"}, {"message_id": "m2", "parent_task_id": "T1"}]}),
            None,
        );
        assert_eq!(applied.rejected.len(), 2, "{:?}", applied.rejected);
        assert!(applied.rejected[0].contains("m2"));
    }

    #[test]
    fn a_message_alone_in_its_default_task_becomes_the_new_task_without_an_empty_one() {
        let mut list = TaskTable::default();
        list.receive("first");
        reply(
            &mut list,
            json!({"task_updates": [{"task_id": "T1", "status": "done"}]}),
            None,
        );
        list.receive("second");
        reply(
            &mut list,
            json!({"message_updates": [{"message_id": "m2", "parent_task_id": "new"}]}),
            None,
        );
        assert_eq!(list.tasks.len(), 2);
        assert_eq!(list.message_ids("T2"), ["m2"]);
    }

    #[test]
    fn done_while_a_tree_of_the_task_runs_is_rejected_for_that_item_only() {
        let mut list = TaskTable::default();
        list.receive("navigate");
        // Not even in the reply that dispatches the tree.
        let applied = reply(
            &mut list,
            json!({"rtdl_parent_task_id": "T1", "task_updates": [{"task_id": "T1", "status": "done"}]}),
            Some("p1"),
        );
        assert_eq!(applied.rejected.len(), 1);
        list.receive("say hello");
        let applied = reply(
            &mut list,
            json!({
                "message_updates": [{"message_id": "m2", "parent_task_id": "new"}],
                "task_updates": [{"task_id": "T1", "status": "done"}, {"task_id": "new", "status": "done"}]
            }),
            None,
        );
        assert_eq!(applied.rejected.len(), 1);
        assert!(applied.rejected[0].contains("p1"));
        assert!(list.task("T1").unwrap().is_open());
        assert_eq!(list.task("T2").unwrap().status, DONE);
        assert_eq!(list.reap_tree("p1", "succeeded").as_deref(), Some("T1"));
        let mut applied = Applied::default();
        list.apply_updates(&[update("T1", DONE)], None, &mut applied);
        assert!(applied.rejected.is_empty());
    }

    #[test]
    fn an_instruction_longer_than_four_hundred_characters_can_be_done() {
        let mut list = TaskTable::default();
        list.receive(&"check every shelf in the kitchen ".repeat(20));
        assert!(list.messages[0].text.chars().count() > 400);
        let applied = reply(
            &mut list,
            json!({"task_updates": [{"task_id": "T1", "status": "done"}]}),
            None,
        );
        assert!(applied.rejected.is_empty());
        assert!(!list.any_open());
    }

    #[test]
    fn task_fields_accept_ids_as_strings_or_integers_and_reject_bad_shapes() {
        let parsed =
            fields(json!({"rtdl_parent_task_id": 2, "message_updates": [{"message_id": 7, "parent_task_id": "NEW"}]})).unwrap();
        assert_eq!(parsed.task, Some(TaskRef::Id("T2".into())));
        assert_eq!(parsed.messages, [("m7".to_string(), TaskRef::New)]);
        assert!(fields(json!({"rtdl_parent_task_id": {}})).is_err());
        assert!(fields(json!({"message_updates": [{"message_id": "m1"}]})).is_err());
        assert!(fields(json!({"task_updates": [{"status": "done"}]})).is_err());
        assert!(
            fields(json!({"task_updates": [{"task_id": "T1", "status": "finished"}]})).is_err()
        );
        assert!(fields(json!({"task_updates": [{"task_id": "T1", "extra": 1}]})).is_err());
        assert_eq!(
            fields(json!({"rtdl_parent_task_id": null, "message_updates": [], "task_updates": []}))
                .unwrap(),
            TaskFields::default()
        );
    }

    #[test]
    fn unknown_and_closed_tasks_are_reported_with_the_open_ones() {
        let mut list = TaskTable::default();
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
    fn a_restored_list_does_not_wait_on_trees_of_the_previous_pilot() {
        let mut list = TaskTable::default();
        list.receive("fetch the cup");
        list.answered();
        list.attach_tree("T1", "p1", "tree");
        let mut restored = TaskTable::from_record(serde_json::to_value(&list).unwrap()).unwrap();
        assert_eq!(restored.trees[0].state, TREE_LOST);
        let mut applied = Applied::default();
        restored.apply_updates(&[update("T1", DONE)], None, &mut applied);
        assert!(applied.rejected.is_empty());
    }

    #[test]
    fn records_and_the_board_name_tasks_messages_and_plans_by_id() {
        let mut list = TaskTable::default();
        list.receive("old request");
        reply(
            &mut list,
            json!({"task_updates": [{"task_id": "T1", "goal": "old", "status": "done"}]}),
            None,
        );
        list.receive("go to the kitchen");
        reply(
            &mut list,
            json!({"task_updates": [{"task_id": "T2", "goal": "reach the kitchen"}]}),
            Some("p4"),
        );
        list.receive("the one upstairs");
        assert_eq!(
            list.messages[2].line(),
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
