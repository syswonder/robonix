// SPDX-License-Identifier: MulanPSL-2.0
//! Bounded JSON-prefix parsing and incremental RTDL arena construction.
use crate::pb::executor::ExecuteStreamInput;
use crate::pb::pilot::{CapabilityCall, Plan, RtdlNode};
use crate::planner::{
    self, CapabilityTargetMap, MetaPlanOp, RTDL_DO, RTDL_PARALLEL, RTDL_SEQUENCE, RtdlEnvelope,
};
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
    text: &'a str,
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

struct Candidate {
    node: RtdlNode,
    parent: Option<usize>,
    closed: bool,
}

/// One model response, with a single immutable-identity arena and pending wire messages.
pub(crate) struct RtdlIncrementalParser {
    text: String,
    targets: CapabilityTargetMap,
    sequence: Arc<AtomicU64>,
    pub(crate) plan: Plan,
    closed: Vec<bool>,
    pending: Vec<ExecuteStreamInput>,
    in_string: bool,
    escaped: bool,
    complete: bool,
    envelope: Option<RtdlEnvelope>,
    failed: bool,
    scan_depth: usize,
    args_depth: Option<usize>,
    string_start: usize,
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
                    // These values never contain RTDL children. Inspect their containers
                    // once at closure, including legacy metadata ignored by full expansion.
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
                ["content", "rtdl", "rtdl_description", "task_update"].contains(&key.as_str()),
                "unknown response field `{key}`"
            );
        }
        for name in ["content", "rtdl_description"] {
            if let Some(value) = fields.get(name) {
                ensure!(value.string().is_some(), "{name} must be a string");
            }
        }
        let mut task_update = None;
        if let Some(update) = fields.get("task_update") {
            match update {
                Partial::Scalar(Value::Null) => {}
                Partial::Object(members, _) => {
                    for (key, value) in members {
                        ensure!(
                            ["goal", "success_criterion", "status"].contains(&key.as_str()),
                            "unknown task_update field `{key}`"
                        );
                        ensure!(
                            value.string().is_some(),
                            "task_update.{key} must be a string"
                        );
                    }
                    if let Some(value) = update.value() {
                        task_update = Some(planner::parse_task_update(&value)?);
                    }
                }
                _ => bail!("task_update must be null or an object"),
            }
        }
        if complete {
            ensure!(fields.len() == 4, "response requires four fields");
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
                task_update,
            });
            self.complete = true;
        }
        let submit = candidates.iter().any(|c| c.node.node_kind == RTDL_DO);
        if candidates.is_empty() || (!submit && !complete) {
            return Ok(());
        }
        if self.plan.plan_id.is_empty() {
            self.plan.plan_id = (self.sequence.fetch_add(1, Ordering::Relaxed) + 1).to_string();
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
                        ["cap", "args"].contains(&key.as_str())
                    } else {
                        key == "children"
                    },
                "unexpected RTDL field `{key}`"
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
            .and_then(Partial::value)
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

/// Allocate the next global op_id (1, 2, 3, …) as a decimal string.
pub(crate) fn next_op_id() -> String {
    (OP_ID_SEQ.fetch_add(1, Ordering::Relaxed) + 1).to_string()
}

#[cfg(test)]
#[path = "reference.rs"]
pub(crate) mod reference;

#[cfg(test)]
mod tests {
    use super::reference::expand_rtdl_to_plan;
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
            "content": "done", "rtdl_description": "", "task_update": null,
            "rtdl": {"op": "sequence", "children": []}
        });
        let mut invalid = Vec::new();
        for key in ["content", "rtdl_description", "rtdl", "task_update"] {
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
            ("task_update", serde_json::json!([])),
            (
                "task_update",
                serde_json::json!({"goal": "g", "status": "done"}),
            ),
            (
                "task_update",
                serde_json::json!({"goal": 1, "success_criterion": "c", "status": "done"}),
            ),
            (
                "task_update",
                serde_json::json!({"goal": "g", "success_criterion": "c", "status": "bad"}),
            ),
            (
                "task_update",
                serde_json::json!({"goal": "g", "success_criterion": "c", "status": "done", "extra": 1}),
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
        assert!(envelope.task_update.is_none());
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
                "task_update": {"goal": "test", "success_criterion": "checked", "status": "done"}
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
                envelope.task_update,
                reference::parse_rtdl_assistant_response(&raw)
                    .unwrap()
                    .task_update
            );
            assert!(meta_op.is_none());
            assert_eq!(p.plan.plan_id, "1");
            assert!(p.take_messages().is_empty());
            assert_same(&p, &raw);
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
                    "content": "", "rtdl_description": "control", "task_update": null, "rtdl": rtdl
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
        p.push(r#"{"content":"work","rtdl_description":"root","task_update":null,"rtdl":{"op":"sequence","description":"root","children":[{"op":"do","cap":"run","args":{}}"#).unwrap();
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
        p.push(r#"{"content":"work","task_update":null,"rtdl":{"op":"do","cap":"run","args":{}},"rtdl_description":"late""#).unwrap();
        let before = p.plan.clone();
        assert!(before.nodes.is_empty());
        assert!(p.take_messages().is_empty());
        assert!(p.finish().unwrap_err().to_string().contains("truncated"));
        assert_eq!(p.plan, before);
        assert_eq!(p.sequence.load(Ordering::Relaxed), 0);
        assert!(p.take_messages().is_empty());
    }

    /// The production node decoder retains the full expander's acceptance and fallback behavior.
    #[test]
    fn node_validation_matches_independent_expansion() {
        for rtdl in [
            serde_json::json!({"op": "do", "cap": "run", "args": {"v": [null, true, 1]}, "description": " trimmed "}),
            serde_json::json!({"op": "sequence", "children": [{"op": "do", "cap": "run", "args": {}}]}),
            serde_json::json!({"op": "do", "cap": "run", "args": {}, "description": null}),
            serde_json::json!({"op": "do", "cap": "run"}),
            serde_json::json!({"op": "do", "args": {}}),
            serde_json::json!({"op": "do", "cap": "missing", "args": {}}),
            serde_json::json!({"op": "do", "cap": "run", "args": []}),
            serde_json::json!({"op": "do", "cap": "run", "args": {}, "out": "x"}),
            serde_json::json!({"op": "sequence", "children": {}}),
            serde_json::json!({"op": "parallel"}),
            serde_json::json!({"op": "sequence", "children": [{"op": "cancel_all"}]}),
            serde_json::json!({"op": 0}),
            serde_json::json!({"op": "unknown"}),
        ] {
            let mut p = parser();
            let expected =
                expand_rtdl_to_plan(&rtdl, &p.targets, "1".into(), "session".into(), 7, "root");
            let raw = serde_json::json!({"content": "", "rtdl_description": "root", "task_update": null, "rtdl": rtdl}).to_string();
            let actual = p.push(&raw).and_then(|()| p.finish());
            assert_eq!(actual.is_ok(), expected.is_ok(), "{raw}: {actual:?}");
            if actual.is_ok() {
                assert_same(&p, &raw);
            }
        }
    }

    /// Compare all fields, reusing the incremental identity allocation in the independent expansion.
    fn assert_same(parser: &RtdlIncrementalParser, raw: &str) {
        let ids: std::collections::HashSet<_> = parser
            .plan
            .nodes
            .iter()
            .map(|node| node.op_id.as_str())
            .collect();
        assert!(!ids.contains(""));
        assert_eq!(ids.len(), parser.plan.nodes.len());
        let v: Value = serde_json::from_str(raw).unwrap();
        let mut expected = expand_rtdl_to_plan(
            &v["rtdl"],
            &parser.targets,
            parser.plan.plan_id.clone(),
            "session".into(),
            7,
            v["rtdl_description"].as_str().unwrap(),
        )
        .unwrap();
        for (a, b) in expected.nodes.iter_mut().zip(&parser.plan.nodes) {
            a.op_id = b.op_id.clone();
        }
        assert_eq!(expected, parser.plan);
    }

    /// Exercise every UTF-8 split and character deltas with nested args and escaped structural text.
    #[test]
    fn arbitrary_splits_and_unicode_match_complete_expansion() {
        let raw = r#"{"content":"你好","rtdl_description":"root","task_update":null,"rtdl":{"description":"root","op":"sequence","children":[{"op":"do","cap":"run","args":{"x":[{"v":"引号\"\\{},[]\u4e2d"}]}},{"op":"parallel","description":"p","children":[{"args":{},"cap":"run","op":"do"}]}]}}"#;
        for split in raw.char_indices().map(|(i, _)| i).chain([raw.len()]) {
            let mut p = parser();
            p.push(&raw[..split]).unwrap();
            p.push(&raw[split..]).unwrap();
            p.finish().unwrap();
            assert_same(&p, raw);
        }
        let mut p = parser();
        for ch in raw.chars() {
            p.push(&ch.to_string()).unwrap();
        }
        p.finish().unwrap();
        assert_same(&p, raw);
    }

    /// The first call and its described ancestors can leave before later siblings exist.
    #[test]
    fn emits_prefix_before_response_completion() {
        let mut p = parser();
        p.push(r#"{"rtdl":{"op":"sequence","description":"root","children":[{"op":"do","cap":"run","args":{}}"#).unwrap();
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

    /// Closed descendants wait until later ancestor metadata and root fallback labels are known.
    #[test]
    fn late_ancestor_fields_and_root_label_are_cached() {
        let raw = r#"{"rtdl":{"children":[{"args":{},"cap":"run","op":"do"}],"op":"sequence"},"rtdl_description":"late label","content":"","task_update":null}"#;
        let split = raw.find(",\"rtdl_description\"").unwrap();
        let mut p = parser();
        p.push(&raw[..split]).unwrap();
        assert!(!p.committed());
        p.push(&raw[split..]).unwrap();
        p.finish().unwrap();
        assert_same(&p, raw);
    }

    /// Duplicate keys, unknown fields, and syntax errors cannot erase or resend a valid prefix.
    #[test]
    fn rejects_bad_tails_and_never_submits_unclosed_do() {
        let prefix = r#"{"rtdl":{"op":"sequence","description":"root","children":[{"op":"do","cap":"run","args":{}}"#;
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
    fn escaped_args_key_and_deep_args_match_full_expansion() {
        let mut args = serde_json::json!({"text": "你好"});
        for _ in 0..48 {
            args = serde_json::json!({"nested": args});
        }
        let raw = format!(
            r#"{{"content":"","rtdl_description":"d","task_update":null,"rtdl":{{"op":"do","cap":"run","a\u0072gs":{args}}}}}"#
        );
        let mut p = parser();
        for ch in raw.chars() {
            p.push(&ch.to_string()).unwrap();
        }
        p.finish().unwrap();
        assert_same(&p, &raw);
    }

    /// Legacy opaque metadata containers must remain bounded without changing full-parse fallbacks.
    #[test]
    fn container_metadata_preserves_complete_expansion() {
        let array = serde_json::json!(vec![0; 4096]);
        let raw = format!(
            r#"{{"content":"","rtdl_description":"label","task_update":null,"rtdl":{{"op":"do","op_id":{array},"description":{array},"cap":"run","args":{{}}}}}}"#
        );
        let mut p = parser();
        p.push(&raw).unwrap();
        p.finish().unwrap();
        assert_same(&p, &raw);
    }

    /// Permute all envelope fields, including a root label arriving after RTDL.
    #[test]
    fn every_envelope_field_order_matches_full_expansion() {
        let fields = [
            r#""content":"hi""#,
            r#""rtdl_description":"label""#,
            r#""task_update":null"#,
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
                        assert_same(&p, &raw);
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
                r#"{{"rtdl":{{"op":"sequence","description":"root","children":[{nodes}]}}}}"#
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
