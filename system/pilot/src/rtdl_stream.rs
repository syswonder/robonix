// SPDX-License-Identifier: MulanPSL-2.0
//! Bounded JSON-prefix parsing and incremental RTDL arena construction.
use crate::pb::executor::ExecuteStreamInput;
use crate::pb::pilot::{CapabilityCall, Plan, RtdlNode};
use crate::planner::{
    self, CapabilityTargetMap, MetaPlanOp, RTDL_DO, RTDL_PARALLEL, RTDL_SEQUENCE, RtdlEnvelope,
};
use crate::tasks::{self, TaskFields};
use anyhow::{Result, bail, ensure};
use serde_json::{Map, Value};
use std::collections::BTreeMap;
use std::sync::{
    Arc,
    atomic::{AtomicU64, Ordering},
};

const MAX_BYTES: usize = 1024 * 1024;
const MAX_NODES: usize = 256;
const MAX_DEPTH: usize = 32;

// Match capabilities/lib/executor/msg/ExecuteStreamInput.msg.
pub(crate) const EXECUTE_STREAM_START: u32 = 0;
pub(crate) const EXECUTE_STREAM_NODE: u32 = 1;
pub(crate) const EXECUTE_STREAM_CLOSE: u32 = 2;
pub(crate) const EXECUTE_STREAM_END: u32 = 3;

/// A discovered JSON value retains closure separately from its completed members.
#[derive(Debug)]
enum Partial {
    Object(BTreeMap<String, Partial>, bool),
    Array(Vec<Partial>, bool),
    Scalar(Value),
}

impl Partial {
    fn closed(&self) -> bool {
        match self {
            Self::Object(_, c) | Self::Array(_, c) => *c,
            Self::Scalar(_) => true,
        }
    }

    /// Materialize only complete values; incomplete descendants cannot become calls.
    fn value(&self) -> Option<Value> {
        if !self.closed() {
            return None;
        }
        match self {
            Self::Scalar(v) => Some(v.clone()),
            Self::Object(map, _) => map
                .iter()
                .map(|(k, v)| Some((k.clone(), v.value()?)))
                .collect::<Option<Map<_, _>>>()
                .map(Value::Object),
            Self::Array(items, _) => items
                .iter()
                .map(Self::value)
                .collect::<Option<Vec<_>>>()
                .map(Value::Array),
        }
    }

    fn string(&self) -> Option<&str> {
        match self {
            Self::Scalar(Value::String(s)) => Some(s),
            _ => None,
        }
    }
}

/// Strict prefix reader: EOF is incomplete; duplicate keys and malformed tokens are errors.
struct Reader<'a> {
    /// Buffered JSON prefix, which may end inside a value.
    text: &'a str,
    /// Next byte to inspect within the prefix.
    pos: usize,
}

impl Reader<'_> {
    fn whitespace(&mut self) {
        while self
            .text
            .as_bytes()
            .get(self.pos)
            .is_some_and(|b| matches!(b, b' ' | b'\t' | b'\r' | b'\n'))
        {
            self.pos += 1;
        }
    }

    /// Decode a complete JSON string, leaving incomplete escape sequences buffered.
    fn string(&mut self) -> Result<Option<String>> {
        let start = self.pos;
        self.pos += 1;
        let mut escape = false;
        while let Some(&b) = self.text.as_bytes().get(self.pos) {
            self.pos += 1;
            if escape {
                escape = false;
            } else if b == b'\\' {
                escape = true;
            } else if b == b'"' {
                return Ok(Some(serde_json::from_str(&self.text[start..self.pos])?));
            } else {
                ensure!(b >= 32, "control character in JSON string");
            }
        }
        Ok(None)
    }

    /// Read a partial container recursively; JSON nesting is bounded even inside args.
    fn read(&mut self, depth: usize) -> Result<Option<Partial>> {
        ensure!(depth <= 96, "JSON nesting exceeds 96");
        self.whitespace();
        let Some(&b) = self.text.as_bytes().get(self.pos) else {
            return Ok(None);
        };
        if b == b'"' {
            return Ok(self.string()?.map(|s| Partial::Scalar(Value::String(s))));
        }
        if b == b'{' {
            self.pos += 1;
            let mut map = BTreeMap::new();
            let mut after_comma = false;
            loop {
                self.whitespace();
                let Some(&b) = self.text.as_bytes().get(self.pos) else {
                    return Ok(Some(Partial::Object(map, false)));
                };
                if b == b'}' {
                    ensure!(!after_comma, "trailing comma in object");
                    self.pos += 1;
                    return Ok(Some(Partial::Object(map, true)));
                }
                ensure!(b == b'"', "object key must be a string");
                let Some(key) = self.string()? else {
                    return Ok(Some(Partial::Object(map, false)));
                };
                ensure!(!map.contains_key(&key), "duplicate JSON field `{key}`");
                self.whitespace();
                let Some(&b) = self.text.as_bytes().get(self.pos) else {
                    return Ok(Some(Partial::Object(map, false)));
                };
                ensure!(b == b':', "expected colon");
                self.pos += 1;
                let Some(value) = self.read(depth + 1)? else {
                    return Ok(Some(Partial::Object(map, false)));
                };
                let closed = value.closed();
                map.insert(key, value);
                if !closed {
                    return Ok(Some(Partial::Object(map, false)));
                }
                self.whitespace();
                match self.text.as_bytes().get(self.pos) {
                    Some(b',') => {
                        self.pos += 1;
                        after_comma = true;
                    }
                    Some(b'}') => {
                        self.pos += 1;
                        return Ok(Some(Partial::Object(map, true)));
                    }
                    None => return Ok(Some(Partial::Object(map, false))),
                    _ => bail!("expected comma or closing object"),
                }
            }
        }
        if b == b'[' {
            self.pos += 1;
            let mut items = Vec::new();
            let mut after_comma = false;
            loop {
                self.whitespace();
                if self.text.as_bytes().get(self.pos) == Some(&b']') {
                    ensure!(!after_comma, "trailing comma in array");
                    self.pos += 1;
                    return Ok(Some(Partial::Array(items, true)));
                }
                let Some(item) = self.read(depth + 1)? else {
                    return Ok(Some(Partial::Array(items, false)));
                };
                let closed = item.closed();
                items.push(item);
                if !closed {
                    return Ok(Some(Partial::Array(items, false)));
                }
                self.whitespace();
                match self.text.as_bytes().get(self.pos) {
                    Some(b',') => {
                        self.pos += 1;
                        after_comma = true;
                    }
                    Some(b']') => {
                        self.pos += 1;
                        return Ok(Some(Partial::Array(items, true)));
                    }
                    None => return Ok(Some(Partial::Array(items, false))),
                    _ => bail!("expected comma or closing array"),
                }
            }
        }
        let start = self.pos;
        while let Some(&b) = self.text.as_bytes().get(self.pos) {
            if b.is_ascii_whitespace() || matches!(b, b',' | b']' | b'}') {
                break;
            }
            self.pos += 1;
        }
        let token = &self.text[start..self.pos];
        match serde_json::from_str(token) {
            Ok(v) => Ok(Some(Partial::Scalar(v))),
            Err(e)
                if self.pos == self.text.len()
                    && (e.is_eof()
                        || ["true", "false", "null"]
                            .iter()
                            .any(|s| s.starts_with(token))) =>
            {
                Ok(None)
            }
            Err(e) => Err(e.into()),
        }
    }
}

/// One validated node ready to extend the contiguous execution prefix.
struct Candidate {
    /// Node metadata and any complete capability call.
    node: RtdlNode,
    /// Previously discovered parent's arena index, absent for the root.
    parent: Option<usize>,
    /// Whether this node's complete JSON object has arrived.
    closed: bool,
}

/// One model response, with a single immutable-identity arena and pending wire messages.
pub(crate) struct RtdlIncrementalParser {
    /// Model output retained for prefix validation and failure diagnostics.
    text: String,
    /// Current catalog names and their executable provider/contract targets.
    targets: CapabilityTargetMap,
    /// Shared source of plan ids, allocated only when a tree is admitted.
    sequence: Arc<AtomicU64>,
    /// Contiguous validated arena with immutable node and call identities.
    pub(crate) plan: Plan,
    /// Completed task fields received before the current RTDL prefix.
    pub(crate) task_fields: TaskFields,
    /// Accepted operators already closed on the incremental input.
    closed: Vec<bool>,
    /// Newly admitted wire messages awaiting submission to Executor.
    pending: Vec<ExecuteStreamInput>,
    /// Whether the structural scanner is inside a JSON string.
    in_string: bool,
    /// Whether the next character belongs to a JSON escape sequence.
    escaped: bool,
    /// Whether the complete outer reply object has been validated.
    complete: bool,
    /// Validated reply consumed once by finish.
    envelope: Option<RtdlEnvelope>,
    /// Whether an invalid prefix has made further input inadmissible.
    failed: bool,
    /// Current JSON nesting depth, including opaque call arguments.
    scan_depth: usize,
    /// Depth of an opaque container whose internals cannot release RTDL nodes.
    args_depth: Option<usize>,
    /// Starting byte of the current string, used to recognize opaque keys.
    string_start: usize,
    /// Whether the preceding string identifies an opaque value.
    args_key: bool,
}

impl RtdlIncrementalParser {
    /// Prepare a response without reserving a plan id or submitting any node.
    pub(crate) fn new(
        targets: CapabilityTargetMap,
        sequence: Arc<AtomicU64>,
        session_id: String,
        round: u32,
    ) -> Self {
        Self {
            text: String::new(),
            targets,
            sequence,
            plan: Plan {
                session_id,
                round,
                ..Default::default()
            },
            task_fields: TaskFields::default(),
            closed: vec![],
            pending: vec![],
            in_string: false,
            escaped: false,
            complete: false,
            envelope: None,
            failed: false,
            scan_depth: 0,
            args_depth: None,
            string_start: 0,
            args_key: false,
        }
    }

    #[cfg(test)]
    fn committed(&self) -> bool {
        !self.plan.nodes.is_empty()
    }
    pub(crate) fn text(&self) -> &str {
        &self.text
    }
    pub(crate) fn take_messages(&mut self) -> Vec<ExecuteStreamInput> {
        std::mem::take(&mut self.pending)
    }

    /// Inspect structural boundaries even inside a large delta so a bad tail cannot erase a prefix.
    pub(crate) fn push(&mut self, delta: &str) -> Result<()> {
        ensure!(!self.failed, "response already failed");
        let result = self.push_inner(delta);
        if result.is_err() {
            self.failed = true;
        }
        result
    }

    /// Buffer strings intact, parsing only at structural boundaries outside string literals.
    fn push_inner(&mut self, delta: &str) -> Result<()> {
        for ch in delta.chars() {
            ensure!(
                !self.complete || matches!(ch, ' ' | '\t' | '\r' | '\n'),
                "invalid JSON tail"
            );
            ensure!(
                self.text.len() + ch.len_utf8() <= MAX_BYTES,
                "response exceeds 1 MiB"
            );
            self.text.push(ch);
            if self.in_string {
                if self.escaped {
                    self.escaped = false;
                } else if ch == '\\' {
                    self.escaped = true;
                } else if ch == '"' {
                    self.in_string = false;
                    let token = &self.text[self.string_start..];
                    // These values never contain RTDL children, so inspect
                    // their containers only once at closure.
                    const OPAQUE_KEYS: [&str; 10] = [
                        "args",
                        "op_id",
                        "description",
                        "op",
                        "cap",
                        "content",
                        "rtdl_description",
                        "goal",
                        "success_criterion",
                        "status",
                    ];
                    self.args_key = if token.contains('\\') {
                        serde_json::from_str::<String>(token)
                            .is_ok_and(|key| OPAQUE_KEYS.contains(&key.as_str()))
                    } else {
                        OPAQUE_KEYS.contains(&&token[1..token.len() - 1])
                    };
                }
            } else if ch == '"' {
                self.in_string = true;
                self.string_start = self.text.len() - 1;
            } else if matches!(ch, '{' | '[') {
                self.scan_depth += 1;
                ensure!(self.scan_depth <= 96, "JSON nesting exceeds 96");
                if self.args_key && self.args_depth.is_none() {
                    self.args_depth = Some(self.scan_depth);
                }
                self.args_key = false;
            } else if matches!(ch, '}' | ']') {
                if self.args_depth == Some(self.scan_depth) {
                    self.args_depth = None;
                }
                self.scan_depth = self.scan_depth.saturating_sub(1);
                if self.args_depth.is_none() {
                    self.inspect()?;
                }
                self.args_key = false;
            } else if ch == ',' {
                if self.args_depth.is_none() {
                    self.inspect()?;
                }
                self.args_key = false;
            } else if ch != ':' && !ch.is_whitespace() {
                self.args_key = false;
            }
        }
        Ok(())
    }

    /// Validate completion and return the envelope and root meta op.
    /// Only push builds nodes and wire messages; EOF never releases buffered work.
    pub(crate) fn finish(&mut self) -> Result<(RtdlEnvelope, Option<MetaPlanOp>)> {
        ensure!(!self.failed, "response already failed");
        ensure!(self.complete, "truncated JSON response");
        let envelope = self
            .envelope
            .take()
            .ok_or_else(|| anyhow::anyhow!("response already finished"))?;
        // meta_op is the root RTDL control operation (`cancel_plan`, `cancel_all`....)
        let meta_op = planner::parse_meta_plan_op(&envelope.rtdl)?;
        Ok((envelope, meta_op))
    }

    /// Build a safe preorder prefix, preserving already assigned ids and sibling order.
    fn inspect(&mut self) -> Result<()> {
        if self.complete {
            return Ok(());
        }
        let mut reader = Reader {
            text: &self.text,
            pos: 0,
        };
        let Some(root) = reader.read(0)? else {
            return Ok(());
        };
        reader.whitespace();
        ensure!(reader.pos == self.text.len(), "invalid JSON tail");
        let Partial::Object(fields, complete) = root else {
            bail!("assistant response must be an object")
        };
        for key in fields.keys() {
            ensure!(
                ["content", "rtdl", "rtdl_description"].contains(&key.as_str())
                    || tasks::REPLY_FIELDS.contains(&key.as_str()),
                "unknown response field `{key}`"
            );
        }
        for name in ["content", "rtdl_description"] {
            if let Some(value) = fields.get(name) {
                ensure!(value.string().is_some(), "{name} must be a string");
            }
        }
        let task_values = tasks::REPLY_FIELDS
            .iter()
            .filter_map(|key| Some(((*key).to_string(), fields.get(*key)?.value()?)))
            .collect();
        self.task_fields = tasks::parse_task_fields(&task_values)?;
        if complete {
            ensure!(
                ["content", "rtdl_description", "rtdl"]
                    .iter()
                    .all(|key| fields.contains_key(*key)),
                "response requires content, rtdl_description and rtdl"
            );
        }
        let Some(rtdl) = fields.get("rtdl") else {
            return Ok(());
        };
        Self::check_limits(rtdl, 1, &mut 0)?;
        let label = fields.get("rtdl_description").and_then(Partial::string);
        let mut candidates = Vec::new();
        let closed = self.walk(rtdl, None, 1, label, &mut candidates)?;
        ensure!(!complete || closed, "incomplete RTDL tree");
        if complete {
            self.envelope = Some(RtdlEnvelope {
                content: fields["content"]
                    .string()
                    .expect("validated content")
                    .into(),
                rtdl_description: label.expect("validated description").into(),
                rtdl: rtdl.value().expect("complete RTDL"),
                task_fields: self.task_fields.clone(),
            });
            self.complete = true;
        }
        let submit = candidates.iter().any(|c| c.node.node_kind == RTDL_DO);
        if !complete
            && !["message_updates", "rtdl_parent_task_id"]
                .iter()
                .all(|key| fields.get(*key).is_some_and(Partial::closed))
        {
            return Ok(());
        }
        if candidates.is_empty() || (!submit && !complete) {
            return Ok(());
        }
        if self.plan.plan_id.is_empty() {
            self.plan.plan_id = format!(
                "{}-{}",
                *planner::RUN_TAG,
                self.sequence.fetch_add(1, Ordering::Relaxed) + 1
            );
            if submit {
                self.pending.push(ExecuteStreamInput {
                    message_kind: EXECUTE_STREAM_START,
                    plan_id: self.plan.plan_id.clone(),
                    session_id: self.plan.session_id.clone(),
                    round: self.plan.round,
                    ..Default::default()
                });
            }
        }
        for (index, candidate) in candidates.iter().enumerate() {
            if index < self.plan.nodes.len() {
                continue;
            }
            let mut node = candidate.node.clone();
            node.op_id = next_op_id();
            let call_index = self.plan.nodes.iter().filter(|n| n.call.is_some()).count();
            if let Some(call) = &mut node.call {
                call.call_id = format!("{}:{call_index}", self.plan.plan_id);
            }
            if submit {
                self.pending.push(ExecuteStreamInput {
                    message_kind: EXECUTE_STREAM_NODE,
                    node_index: index as u32,
                    has_parent: candidate.parent.is_some(),
                    parent_index: candidate.parent.unwrap_or_default() as u32,
                    node: Some(node.clone()),
                    ..Default::default()
                });
            }
            if let Some(parent) = candidate.parent {
                self.plan.nodes[parent].children.push(index as u32);
            }
            self.closed.push(node.node_kind == 2);
            self.plan.nodes.push(node);
        }
        for (index, candidate) in candidates.iter().enumerate().rev() {
            if candidate.closed && !self.closed[index] {
                self.closed[index] = true;
                if submit {
                    self.pending.push(ExecuteStreamInput {
                        message_kind: EXECUTE_STREAM_CLOSE,
                        node_index: index as u32,
                        ..Default::default()
                    });
                }
            }
        }
        Ok(())
    }

    /// Count discovered descendants even while missing ancestor metadata prevents submission.
    fn check_limits(value: &Partial, depth: usize, count: &mut usize) -> Result<()> {
        *count += 1;
        ensure!(*count <= MAX_NODES, "RTDL exceeds 256 nodes");
        ensure!(depth <= MAX_DEPTH, "RTDL nesting exceeds 32");
        if let Partial::Object(fields, _) = value
            && let Some(Partial::Array(children, _)) = fields.get("children")
        {
            for child in children {
                Self::check_limits(child, depth + 1, count)?;
            }
        }
        Ok(())
    }

    /// Visit only a contiguous preorder prefix whose identities and descriptions are determined.
    fn walk(
        &self,
        value: &Partial,
        parent: Option<usize>,
        depth: usize,
        label: Option<&str>,
        out: &mut Vec<Candidate>,
    ) -> Result<bool> {
        ensure!(depth <= MAX_DEPTH, "RTDL nesting exceeds 32");
        let Partial::Object(fields, closed) = value else {
            bail!("RTDL node must be an object")
        };
        let Some(op) = fields.get("op").and_then(Partial::string) else {
            ensure!(!closed, "RTDL node requires op");
            return Ok(false);
        };
        // Meta operations remain on the existing out-of-band control path.
        if parent.is_none() && ["cancel_plan", "cancel_all"].contains(&op) {
            return Ok(true);
        }
        ensure!(
            ["sequence", "parallel", "do"].contains(&op),
            "unknown RTDL operator `{op}`"
        );
        for key in fields.keys() {
            ensure!(
                ["op", "op_id", "description"].contains(&key.as_str())
                    || if op == "do" {
                        ["cap", "args", "children"].contains(&key.as_str())
                    } else {
                        key == "children"
                    },
                "unexpected field `{key}` in RTDL"
            );
        }
        let description = fields
            .get("description")
            .and_then(Partial::string)
            .filter(|s| !s.trim().is_empty());
        if description.is_none() && (!closed || (parent.is_none() && label.is_none())) {
            return Ok(false);
        }
        ensure!(out.len() < MAX_NODES, "RTDL exceeds 256 nodes");
        let index = out.len();
        if op == "do" {
            if !closed {
                return Ok(false);
            }
            ensure!(
                fields.get("children").is_none_or(
                    |value| matches!(value, Partial::Array(items, true) if items.is_empty())
                ),
                "unexpected field `children` on a do node"
            );
            out.push(Candidate {
                node: self.parse_do_node(
                    fields,
                    description,
                    parent.is_none().then_some(label).flatten(),
                )?,
                parent,
                closed: true,
            });
            return Ok(true);
        }
        let Some(Partial::Array(children, _)) = fields.get("children") else {
            ensure!(
                !closed && !fields.contains_key("children"),
                "operator children must be an array"
            );
            return Ok(false);
        };
        let description = pick_description(
            description,
            parent.is_none().then_some(label).flatten(),
            format!("{op} of {} step(s)", children.len()),
        );
        out.push(Candidate {
            node: RtdlNode {
                node_kind: if op == "sequence" {
                    RTDL_SEQUENCE
                } else {
                    RTDL_PARALLEL
                },
                description,
                ..Default::default()
            },
            parent,
            closed: false,
        });
        for child in children {
            if !self.walk(child, Some(index), depth + 1, label, out)? {
                return Ok(false);
            }
        }
        out[index].closed = *closed;
        Ok(*closed)
    }

    /// Decode a complete call without assigning identities or appending to an arena.
    fn parse_do_node(
        &self,
        fields: &BTreeMap<String, Partial>,
        description: Option<&str>,
        root_label: Option<&str>,
    ) -> Result<RtdlNode> {
        let cap = fields
            .get("cap")
            .and_then(Partial::string)
            .ok_or_else(|| anyhow::anyhow!("do.cap must be a string"))?;
        let args = fields
            .get("args")
            .map(Partial::value)
            .unwrap_or_else(|| Some(Value::Object(Map::new())))
            .filter(Value::is_object)
            .ok_or_else(|| anyhow::anyhow!("do.args must be an object"))?;
        let (provider_id, contract_id) = self
            .targets
            .get(cap)
            .ok_or_else(|| anyhow::anyhow!("unknown capability `{cap}`"))?;
        Ok(RtdlNode {
            node_kind: RTDL_DO,
            description: pick_description(description, root_label, format!("call {cap}")),
            call: Some(CapabilityCall {
                provider_id: provider_id.clone(),
                contract_id: contract_id.clone(),
                args_json: serde_json::to_string(&args)?,
                ..Default::default()
            }),
            ..Default::default()
        })
    }
}

/// Prefer a trimmed node description, then the root label, then a generated description.
fn pick_description(
    description: Option<&str>,
    root_label: Option<&str>,
    fallback: String,
) -> String {
    description
        .map(str::trim)
        .filter(|s| !s.is_empty())
        .or_else(|| root_label.filter(|s| !s.is_empty()))
        .map(str::to_string)
        .unwrap_or(fallback)
}

/// Process-wide monotonic source of node `op_id`s. The LLM-emitted RTDL does
/// not carry a usable op_id (it defaults to 0), so pilot assigns one itself
/// while parsing: a globally-unique, auto-incrementing id starting at 1.
/// "Global" = across every plan/round in this pilot process, so each node in
/// the live task-graph forest is uniquely addressable for steering and result
/// correlation — not merely unique within one plan.
static OP_ID_SEQ: AtomicU64 = AtomicU64::new(0);

/// Keep node ids distinct from ids retained across Pilot restarts.
pub(crate) fn next_op_id() -> String {
    format!(
        "{}-{}",
        *planner::RUN_TAG,
        OP_ID_SEQ.fetch_add(1, Ordering::Relaxed) + 1
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Provide one capability and a fresh per-test plan id sequence.
    fn parser() -> RtdlIncrementalParser {
        RtdlIncrementalParser::new(
            HashMap::from([("run".into(), ("executor".into(), "test/run".into()))]),
            Arc::new(AtomicU64::new(0)),
            "session".into(),
            7,
        )
    }
    use std::collections::HashMap;

    /// Production finalization requires all envelope fields and validates completed task metadata.
    #[test]
    fn finish_enforces_envelope_schema() {
        let valid = serde_json::json!({
            "content": "done", "rtdl_description": "", "message_updates": [], "rtdl_parent_task_id": "T1", "task_updates": [],
            "rtdl": {"op": "sequence", "children": []}
        });
        let mut invalid = Vec::new();
        for key in ["content", "rtdl_description", "rtdl"] {
            let mut missing = valid.clone();
            missing.as_object_mut().unwrap().remove(key);
            invalid.push(missing);
        }
        for (key, value) in [
            ("extra", serde_json::json!(null)),
            ("content", serde_json::json!(7)),
            ("rtdl_description", serde_json::json!([])),
            ("rtdl", serde_json::json!(null)),
            ("rtdl", serde_json::json!([])),
            ("task_updates", serde_json::json!({})),
            ("message_updates", serde_json::json!([{"message_id":"m1"}])),
            ("rtdl_parent_task_id", serde_json::json!([])),
            (
                "task_updates",
                serde_json::json!([{"task_id":"T1","status":"bad"}]),
            ),
        ] {
            let mut wrong = valid.clone();
            wrong[key] = value;
            invalid.push(wrong);
        }
        for envelope in invalid {
            let mut p = parser();
            assert!(
                p.push(&envelope.to_string())
                    .and_then(|()| p.finish())
                    .is_err(),
                "{envelope}"
            );
            assert!(p.take_messages().is_empty());
        }
        let mut p = parser();
        p.push(&valid.to_string()).unwrap();
        let (envelope, meta_op) = p.finish().unwrap();
        assert!(envelope.task_fields.updates.is_empty());
        assert!(meta_op.is_none());
    }

    /// Finalization preserves complete metadata and builds no-call trees without wire traffic.
    #[test]
    fn finish_builds_local_trees_and_returns_envelope() {
        for rtdl in [
            serde_json::json!({"op": "sequence", "children": []}),
            serde_json::json!({"op": "parallel", "children": [
                {"op": "sequence", "children": []},
                {"op": "parallel", "description": " nested ", "children": []}
            ]}),
        ] {
            let raw = serde_json::json!({
                "content": "done", "rtdl_description": "root", "rtdl": rtdl,
                "message_updates": [], "rtdl_parent_task_id": "T1", "task_updates": [{"task_id":"T1","goal": "test", "success_criterion": "checked", "status": "done"}]
            })
            .to_string();
            let mut p = parser();
            for ch in raw.chars() {
                p.push(&ch.to_string()).unwrap();
                assert!(p.take_messages().is_empty());
            }
            let (envelope, meta_op) = p.finish().unwrap();
            assert_eq!(envelope.content, "done");
            assert_eq!(envelope.rtdl_description, "root");
            assert_eq!(envelope.rtdl, rtdl);
            assert_eq!(
                envelope.task_fields.updates[0].goal.as_deref(),
                Some("test")
            );
            assert_eq!(
                envelope.task_fields.updates[0].status.as_deref(),
                Some("done")
            );
            assert!(meta_op.is_none());
            assert_eq!(p.plan.plan_id, format!("{}-1", *planner::RUN_TAG));
            assert!(p.take_messages().is_empty());
            if rtdl["op"] == "sequence" {
                assert_nodes(&p, &[(RTDL_SEQUENCE, &[], "root")]);
            } else {
                assert_nodes(
                    &p,
                    &[
                        (RTDL_PARALLEL, &[1, 2], "root"),
                        (RTDL_SEQUENCE, &[], "sequence of 0 step(s)"),
                        (RTDL_PARALLEL, &[], "nested"),
                    ],
                );
            }
        }
    }

    /// Root controls validate at finish and never allocate a Plan or execution messages.
    #[test]
    fn finish_validates_meta_ops_without_allocating_plan_ids() {
        for rtdl in [
            serde_json::json!({"op": "cancel_plan", "plan_id": "p", "wait_ms": 50_000}),
            serde_json::json!({"op": "cancel_all"}),
            serde_json::json!({"op": "cancel_plan", "plan_id": ""}),
            serde_json::json!({"op": "cancel_plan"}),
        ] {
            let expected = planner::parse_meta_plan_op(&rtdl);
            let mut p = parser();
            p.push(
                &serde_json::json!({
                    "content": "", "rtdl_description": "control", "message_updates": [], "rtdl_parent_task_id": "T1", "task_updates": [], "rtdl": rtdl
                })
                .to_string(),
            )
            .unwrap();
            match (p.finish(), expected) {
                (Ok((_, actual)), Ok(expected)) => assert_eq!(actual, expected),
                (Err(_), Err(_)) => {}
                other => panic!("meta op mismatch: {other:?}"),
            }
            assert!(p.plan.nodes.is_empty());
            assert!(p.plan.plan_id.is_empty());
            assert_eq!(p.sequence.load(Ordering::Relaxed), 0);
            assert!(p.take_messages().is_empty());
        }
    }

    /// Removed control operations fail before allocating or dispatching a plan.
    #[test]
    fn removed_control_is_rejected_without_dispatch() {
        let mut p = parser();
        let error = p
            .push(r#"{"rtdl":{"op":"stop_plan_at","plan_id":"p","target_op_id":"n","when":"on_complete"}}"#)
            .unwrap_err();
        assert!(error.to_string().contains("unknown RTDL operator"));
        assert!(p.plan.nodes.is_empty());
        assert!(p.plan.plan_id.is_empty());
        assert!(p.take_messages().is_empty());
    }

    /// The last delta emits remaining closures; finalization preserves nodes and emits nothing.
    #[test]
    fn finish_keeps_submitted_node_identities() {
        let mut p = parser();
        p.push(r#"{"content":"work","rtdl_description":"root","message_updates":[],"rtdl_parent_task_id":"T1","task_updates":[],"rtdl":{"op":"sequence","description":"root","children":[{"op":"do","cap":"run","args":{}}"#).unwrap();
        let prefix = p.plan.clone();
        assert_eq!(p.take_messages().len(), 3);
        p.push("]}}").unwrap();
        assert_eq!(
            p.take_messages()
                .iter()
                .map(|m| m.message_kind)
                .collect::<Vec<_>>(),
            [EXECUTE_STREAM_CLOSE]
        );
        p.finish().unwrap();
        assert_eq!(p.plan, prefix);
        assert!(p.take_messages().is_empty());
        assert!(p.finish().is_err());
    }

    /// EOF after a late label must not release a call whose submission was waiting for it.
    #[test]
    fn finish_does_not_release_calls_from_a_truncated_response() {
        let mut p = parser();
        p.push(r#"{"content":"work","message_updates":[],"rtdl_parent_task_id":"T1","task_updates":[],"rtdl":{"op":"do","cap":"run","args":{}},"rtdl_description":"late""#).unwrap();
        let before = p.plan.clone();
        assert!(before.nodes.is_empty());
        assert!(p.take_messages().is_empty());
        assert!(p.finish().unwrap_err().to_string().contains("truncated"));
        assert_eq!(p.plan, before);
        assert_eq!(p.sequence.load(Ordering::Relaxed), 0);
        assert!(p.take_messages().is_empty());
    }

    #[test]
    fn node_validation_accepts_valid_calls_and_rejects_invalid_fields() {
        for (rtdl, valid) in [
            (
                serde_json::json!({"op": "do", "cap": "run", "args": {"v": [null, true, 1]}, "description": " trimmed "}),
                true,
            ),
            (
                serde_json::json!({"op": "sequence", "children": [{"op": "do", "cap": "run", "args": {}}]}),
                true,
            ),
            (
                serde_json::json!({"op": "do", "cap": "run", "args": {}, "description": null}),
                true,
            ),
            (serde_json::json!({"op": "do", "cap": "run"}), true),
            (serde_json::json!({"op": "do", "args": {}}), false),
            (
                serde_json::json!({"op": "do", "cap": "missing", "args": {}}),
                false,
            ),
            (
                serde_json::json!({"op": "do", "cap": "run", "args": []}),
                false,
            ),
            (
                serde_json::json!({"op": "do", "cap": "run", "args": {}, "out": "x"}),
                false,
            ),
            (serde_json::json!({"op": "sequence", "children": {}}), false),
            (serde_json::json!({"op": "parallel"}), false),
            (
                serde_json::json!({"op": "sequence", "children": [{"op": "cancel_all"}]}),
                false,
            ),
            (serde_json::json!({"op": 0}), false),
            (serde_json::json!({"op": "unknown"}), false),
        ] {
            let mut p = parser();
            let raw = serde_json::json!({"content": "", "rtdl_description": "root", "message_updates": [], "rtdl_parent_task_id": "T1", "task_updates": [], "rtdl": rtdl}).to_string();
            let actual = p.push(&raw).and_then(|()| p.finish());
            assert_eq!(actual.is_ok(), valid, "{raw}: {actual:?}");
            if !valid {
                continue;
            }
            if rtdl["op"] == "sequence" {
                assert_nodes(
                    &p,
                    &[(RTDL_SEQUENCE, &[1], "root"), (RTDL_DO, &[], "call run")],
                );
            } else {
                let description = if rtdl["description"].is_string() {
                    "trimmed"
                } else {
                    "root"
                };
                assert_nodes(&p, &[(RTDL_DO, &[], description)]);
                let args: Value =
                    serde_json::from_str(&p.plan.nodes[0].call.as_ref().unwrap().args_json)
                        .unwrap();
                assert_eq!(
                    args,
                    rtdl.get("args")
                        .cloned()
                        .unwrap_or_else(|| serde_json::json!({}))
                );
            }
        }
    }

    fn assert_nodes(parser: &RtdlIncrementalParser, expected: &[(u32, &[u32], &str)]) {
        let plan = &parser.plan;
        assert_eq!(plan.plan_id, format!("{}-1", *planner::RUN_TAG));
        assert_eq!(plan.session_id, "session");
        assert_eq!(plan.round, 7);
        assert_eq!(plan.root_index, 0);
        assert_eq!(plan.nodes.len(), expected.len());
        let ids: std::collections::HashSet<_> =
            plan.nodes.iter().map(|node| node.op_id.as_str()).collect();
        assert!(!ids.contains(""));
        assert_eq!(ids.len(), plan.nodes.len());
        let mut call_index = 0;
        for (node, (kind, children, description)) in plan.nodes.iter().zip(expected) {
            assert_eq!(node.node_kind, *kind);
            assert_eq!(node.children, *children);
            assert_eq!(node.description, *description);
            if *kind == RTDL_DO {
                let call = node.call.as_ref().unwrap();
                assert_eq!(call.call_id, format!("{}:{call_index}", plan.plan_id));
                assert_eq!(call.provider_id, "executor");
                assert_eq!(call.contract_id, "test/run");
                call_index += 1;
            } else {
                assert!(node.call.is_none());
            }
        }
    }

    /// Exercise every UTF-8 split and character deltas with nested args and escaped structural text.
    #[test]
    fn arbitrary_splits_preserve_unicode_and_tree_shape() {
        let raw = r#"{"content":"你好","rtdl_description":"root","message_updates":[],"rtdl_parent_task_id":"T1","task_updates":[],"rtdl":{"description":"root","op":"sequence","children":[{"op":"do","cap":"run","args":{"x":[{"v":"引号\"\\{},[]\u4e2d"}]}},{"op":"parallel","description":"p","children":[{"args":{},"cap":"run","op":"do"}]}]}}"#; // i18n-ok: multibyte UTF-8 test fixture
        let check = |p: &RtdlIncrementalParser| {
            assert_nodes(
                p,
                &[
                    (RTDL_SEQUENCE, &[1, 2], "root"),
                    (RTDL_DO, &[], "call run"),
                    (RTDL_PARALLEL, &[3], "p"),
                    (RTDL_DO, &[], "call run"),
                ],
            );
            let args: Value =
                serde_json::from_str(&p.plan.nodes[1].call.as_ref().unwrap().args_json).unwrap();
            assert_eq!(
                args,
                serde_json::from_str::<Value>(raw).unwrap()["rtdl"]["children"][0]["args"]
            );
        };
        for split in raw.char_indices().map(|(i, _)| i).chain([raw.len()]) {
            let mut p = parser();
            p.push(&raw[..split]).unwrap();
            p.push(&raw[split..]).unwrap();
            p.finish().unwrap();
            check(&p);
        }
        let mut p = parser();
        for ch in raw.chars() {
            p.push(&ch.to_string()).unwrap();
        }
        p.finish().unwrap();
        check(&p);
    }

    /// The first call and its described ancestors can leave before later siblings exist.
    #[test]
    fn emits_prefix_before_response_completion() {
        let mut p = parser();
        p.push(r#"{"message_updates":[],"rtdl_parent_task_id":"T1","rtdl":{"op":"sequence","description":"root","children":[{"op":"do","cap":"run","args":{}}"#).unwrap();
        assert_eq!(p.plan.nodes.len(), 2);
        assert_eq!(
            p.take_messages()
                .iter()
                .map(|m| m.message_kind)
                .collect::<Vec<_>>(),
            [
                EXECUTE_STREAM_START,
                EXECUTE_STREAM_NODE,
                EXECUTE_STREAM_NODE,
            ]
        );
        assert!(p.finish().is_err());
        assert_eq!(p.plan.nodes.len(), 2);
    }

    #[test]
    fn waits_for_task_ownership_before_releasing_a_tree() {
        let mut p = parser();
        p.push(r#"{"content":"work","rtdl_description":"root","rtdl":{"op":"do","cap":"run","args":{}},"message_updates":[{"message_id":"m2","parent_task_id":"new"}]"#).unwrap();
        assert!(p.take_messages().is_empty());
        p.push(r#", "rtdl_parent_task_id":"new","task_updates":["#)
            .unwrap();
        assert_eq!(p.take_messages().len(), 2);
        assert_eq!(p.task_fields.task, Some(tasks::TaskRef::New));
        assert_eq!(
            p.task_fields.messages,
            vec![("m2".into(), tasks::TaskRef::New)]
        );
        assert!(p.task_fields.updates.is_empty());
        let prefix = p.plan.clone();
        p.push(r#"{"task_id":"new","status":"done"}]}"#).unwrap();
        let (envelope, _) = p.finish().unwrap();
        assert_eq!(envelope.task_fields.updates[0].task, tasks::TaskRef::New);
        assert_eq!(p.plan, prefix);
        assert!(p.take_messages().is_empty());
    }

    /// Closed descendants wait until later ancestor metadata and root fallback labels are known.
    #[test]
    fn late_ancestor_fields_and_root_label_are_cached() {
        let raw = r#"{"rtdl":{"children":[{"args":{},"cap":"run","op":"do"}],"op":"sequence"},"rtdl_description":"late label","content":"","message_updates":[],"rtdl_parent_task_id":"T1","task_updates":[]}"#;
        let split = raw.find(",\"rtdl_description\"").unwrap();
        let mut p = parser();
        p.push(&raw[..split]).unwrap();
        assert!(!p.committed());
        p.push(&raw[split..]).unwrap();
        p.finish().unwrap();
        assert_nodes(
            &p,
            &[
                (RTDL_SEQUENCE, &[1], "late label"),
                (RTDL_DO, &[], "call run"),
            ],
        );
    }

    /// Duplicate keys, unknown fields, and syntax errors cannot erase or resend a valid prefix.
    #[test]
    fn rejects_bad_tails_and_never_submits_unclosed_do() {
        let prefix = r#"{"message_updates":[],"rtdl_parent_task_id":"T1","rtdl":{"op":"sequence","description":"root","children":[{"op":"do","cap":"run","args":{}}"#;
        for tail in [",]}}", "]},\"rtdl\":{}", "]},\"unknown\":1}"] {
            let mut p = parser();
            assert!(p.push(&format!("{prefix}{tail}")).is_err());
            assert_eq!(p.plan.nodes.len(), 2);
        }
        for node in [
            r#"{"op":"do","op":"do","cap":"run","args":{}}"#,
            r#"{"op":"do","cap":"run","args":{},"other":1}"#,
            r#"{"op":"do","cap":"run","args":{"a":1,"a":2}}"#,
        ] {
            let mut p = parser();
            assert!(p.push(&format!(r#"{{"rtdl":{node}}}"#)).is_err());
            assert!(!p.committed());
        }
        let mut p = parser();
        p.push(r#"{"rtdl":{"op":"do","description":"d","cap":"run","args":{}"#)
            .unwrap();
        assert!(!p.committed());
        assert!(p.finish().is_err());
    }

    /// Reject malformed completed fields before any capability can be admitted.
    #[test]
    fn invalid_envelope_and_call_fields_never_commit() {
        for raw in [
            r#"{"content":7,"rtdl":{"op":"do","description":"d","cap":"run","args":{}}}"#,
            r#"{"task_update":[0,1],"rtdl":{}}"#,
            r#"{"task_update":{"goal":[0,1]}}"#,
            r#"{"rtdl":{"op":"do","description":"d","cap":"run","args":[]}}"#,
            r#"{"rtdl":{"op":"do","description":"d","cap":"missing","args":{}}}"#,
            r#"{"rtdl":{"op":"do","description":"d","cap":3,"args":{}}}"#,
        ] {
            let mut p = parser();
            assert!(p.push(raw).is_err(), "{raw}");
            assert!(!p.committed());
        }
    }

    /// Escaped field names and deeply nested args obey the same validation and buffering rules.
    #[test]
    fn escaped_args_key_preserves_deep_args() {
        let mut args = serde_json::json!({"text": "你好"}); // i18n-ok: multibyte UTF-8 test fixture
        for _ in 0..48 {
            args = serde_json::json!({"nested": args});
        }
        let raw = format!(
            r#"{{"content":"","rtdl_description":"d","message_updates":[],"rtdl_parent_task_id":"T1","task_updates":[],"rtdl":{{"op":"do","cap":"run","a\u0072gs":{args}}}}}"#
        );
        let mut p = parser();
        for ch in raw.chars() {
            p.push(&ch.to_string()).unwrap();
        }
        p.finish().unwrap();
        assert_nodes(&p, &[(RTDL_DO, &[], "d")]);
        let actual: Value =
            serde_json::from_str(&p.plan.nodes[0].call.as_ref().unwrap().args_json).unwrap();
        assert_eq!(actual, args);
    }

    #[test]
    fn container_metadata_uses_root_description_fallback() {
        let array = serde_json::json!(vec![0; 4096]);
        let raw = format!(
            r#"{{"content":"","rtdl_description":"label","message_updates":[],"rtdl_parent_task_id":"T1","task_updates":[],"rtdl":{{"op":"do","op_id":{array},"description":{array},"cap":"run","args":{{}}}}}}"#
        );
        let mut p = parser();
        p.push(&raw).unwrap();
        p.finish().unwrap();
        assert_nodes(&p, &[(RTDL_DO, &[], "label")]);
        assert_eq!(p.plan.nodes[0].call.as_ref().unwrap().args_json, "{}");
    }

    /// Permute all envelope fields, including a root label arriving after RTDL.
    #[test]
    fn every_envelope_field_order_preserves_tree_shape() {
        let fields = [
            r#""content":"hi""#,
            r#""rtdl_description":"label""#,
            r#""message_updates":[],"rtdl_parent_task_id":"T1","task_updates":[]"#,
            r#""rtdl":{"children":[{"args":{},"cap":"run","op":"do"}],"op":"sequence"}"#,
        ];
        for a in 0..4 {
            for b in 0..4 {
                for c in 0..4 {
                    for d in 0..4 {
                        let indices = [a, b, c, d];
                        if indices
                            .iter()
                            .copied()
                            .collect::<std::collections::HashSet<_>>()
                            .len()
                            != 4
                        {
                            continue;
                        }
                        let raw = format!(
                            "{{{}}}",
                            indices
                                .iter()
                                .map(|i| fields[*i])
                                .collect::<Vec<_>>()
                                .join(",")
                        );
                        let mut p = parser();
                        for ch in raw.chars() {
                            p.push(&ch.to_string()).unwrap();
                        }
                        p.finish().unwrap();
                        assert_nodes(
                            &p,
                            &[(RTDL_SEQUENCE, &[1], "label"), (RTDL_DO, &[], "call run")],
                        );
                    }
                }
            }
        }
    }

    /// Enforce byte and node bounds before an oversized response reaches execution.
    #[test]
    fn resource_limits() {
        let mut p = parser();
        assert!(
            p.push(&format!("{{\"content\":\"{}", "x".repeat(MAX_BYTES)))
                .is_err()
        );
        let nodes = std::iter::repeat_n(r#"{"op":"do","cap":"run","args":{}}"#, 256)
            .collect::<Vec<_>>()
            .join(",");
        let mut p = parser();
        assert!(
            p.push(&format!(
                r#"{{"message_updates":[],"rtdl_parent_task_id":"T1","rtdl":{{"op":"sequence","description":"root","children":[{nodes}]}}}}"#
            ))
            .is_err()
        );
        assert_eq!(p.plan.nodes.len(), 256);
        let mut node = r#"{"op":"do","cap":"run","args":{}}"#.to_string();
        for _ in 0..32 {
            node = format!(r#"{{"op":"sequence","description":"nested","children":[{node}]}}"#);
        }
        assert!(parser().push(&format!(r#"{{"rtdl":{node}}}"#)).is_err());
    }
}
