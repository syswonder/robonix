// SPDX-License-Identifier: MulanPSL-2.0
// Flat `TaskCallEvent` (`lib/executor/msg/TaskCallEvent.msg`).

use crate::pb::executor::{BatchComplete, TaskCallEvent, TaskCallStarted};
use crate::pb::pilot::TaskCallResult;

pub const EVT_STARTED: u32 = 0;
pub const EVT_RESULT: u32 = 1;
pub const EVT_COMPLETE: u32 = 2;

pub fn started(call_id: String, tool_name: String) -> TaskCallEvent {
    TaskCallEvent {
        event_kind: EVT_STARTED,
        started: Some(TaskCallStarted { call_id, tool_name }),
        ..Default::default()
    }
}

pub fn result(r: TaskCallResult) -> TaskCallEvent {
    TaskCallEvent {
        event_kind: EVT_RESULT,
        result: Some(r),
        ..Default::default()
    }
}

pub fn complete(graph_id: String, any_failed: bool) -> TaskCallEvent {
    TaskCallEvent {
        event_kind: EVT_COMPLETE,
        complete: Some(BatchComplete {
            graph_id,
            any_failed,
        }),
        ..Default::default()
    }
}
