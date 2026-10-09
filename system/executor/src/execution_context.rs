use std::collections::HashMap;
use std::sync::Arc;

use serde_json::Value;
use tokio::sync::Mutex;

#[derive(Clone, Default)]
pub struct ExecutionContext{
    vars: Arc<Mutex<HashMap<String,Value>>>,
}