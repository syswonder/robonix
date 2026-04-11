// SPDX-License-Identifier: MulanPSL-2.0
// session.rs — Session and SessionManager
//
// A Session tracks one user conversation (potentially many VLM rounds and many
// user turns).  Pilot creates a Session on the first Task, then continues the
// same Session for subsequent turns with the same session_id.

use crate::session_state::SessionState;
use crate::vlm::Message;
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::Mutex;

/// One ongoing user session.
#[allow(dead_code)]
pub struct Session {
    pub session_id: String,
    /// Full VLM conversation history (system prompt excluded — added per turn).
    pub history: Vec<Message>,
    /// Current turn count (incremented on each SubmitTask call).
    pub turn_count: u32,
    #[allow(dead_code)]
    pub state: SessionState,
    #[allow(dead_code)]
    pub created_at_ms: u64,
}

impl Session {
    pub fn new(session_id: impl Into<String>) -> Self {
        let ts = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_millis() as u64;
        Self {
            session_id: session_id.into(),
            history: Vec::new(),
            turn_count: 0,
            state: SessionState::Active,
            created_at_ms: ts,
        }
    }
}

/// Thread-safe map of active sessions.
#[derive(Default, Clone)]
pub struct SessionManager {
    sessions: Arc<Mutex<HashMap<String, Arc<Mutex<Session>>>>>,
}

#[allow(dead_code)]
impl SessionManager {
    /// Get or create a session by ID.
    pub async fn get_or_create(&self, session_id: &str) -> Arc<Mutex<Session>> {
        let mut map = self.sessions.lock().await;
        map.entry(session_id.to_string())
            .or_insert_with(|| Arc::new(Mutex::new(Session::new(session_id))))
            .clone()
    }

    #[allow(dead_code)]
    pub async fn get(&self, session_id: &str) -> Option<Arc<Mutex<Session>>> {
        self.sessions.lock().await.get(session_id).cloned()
    }

    #[allow(dead_code)]
    pub async fn list(&self) -> Vec<(String, SessionState, u64, u32)> {
        let map = self.sessions.lock().await;
        let mut out = Vec::new();
        for (id, arc) in map.iter() {
            let s = arc.lock().await;
            out.push((id.clone(), s.state, s.created_at_ms, s.turn_count));
        }
        out
    }
}
