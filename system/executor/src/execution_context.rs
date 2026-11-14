// SPDX-License-Identifier: MulanPSL-2.0
use serde_json::Value;
use std::collections::HashMap;
use std::sync::Arc;
use tokio::sync::Mutex;

#[derive(Clone, Default)]
pub struct ExecutionContext {
    vars: Arc<Mutex<HashMap<String, Value>>>,
}

impl ExecutionContext {
    pub async fn snapshot(&self) -> HashMap<String, Value> {
        self.vars.lock().await.clone()
    }

    pub async fn store(&self, name: &str, output: &str) {
        let value = serde_json::from_str(output).unwrap_or_else(|_| Value::String(output.into()));
        self.vars.lock().await.insert(name.into(), value);
    }
}
