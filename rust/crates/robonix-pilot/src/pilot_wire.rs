// SPDX-License-Identifier: MulanPSL-2.0
// Flat `PilotEvent` wire encoding (`lib/pilot/msg/PilotEvent.msg`).

use crate::pilot::{BatchResult, PilotEvent, SessionStatusEvent, TaskGraph};

pub const EVT_TEXT_CHUNK: u32 = 0;
/// Payload field `task_graph` (`PilotEvent.task_graph`).
pub const EVT_TASK_GRAPH: u32 = 1;
pub const EVT_BATCH_RESULT: u32 = 2;
pub const EVT_STATUS: u32 = 3;
pub const EVT_FINAL_TEXT: u32 = 4;

pub enum PilotStreamBody {
    TextChunk(String),
    FinalText(String),
    TaskGraph(TaskGraph),
    BatchResult(BatchResult),
    Status(SessionStatusEvent),
}

pub fn pack(session_id: &str, body: PilotStreamBody) -> PilotEvent {
    let mut e = PilotEvent {
        session_id: session_id.to_string(),
        ..Default::default()
    };
    match body {
        PilotStreamBody::TextChunk(s) => {
            e.event_kind = EVT_TEXT_CHUNK;
            e.text_chunk = s;
        }
        PilotStreamBody::TaskGraph(g) => {
            e.event_kind = EVT_TASK_GRAPH;
            e.task_graph = Some(g);
        }
        PilotStreamBody::BatchResult(b) => {
            e.event_kind = EVT_BATCH_RESULT;
            e.batch_result = Some(b);
        }
        PilotStreamBody::Status(s) => {
            e.event_kind = EVT_STATUS;
            e.status = Some(s);
        }
        PilotStreamBody::FinalText(s) => {
            e.event_kind = EVT_FINAL_TEXT;
            e.final_text = s;
        }
    }
    e
}
