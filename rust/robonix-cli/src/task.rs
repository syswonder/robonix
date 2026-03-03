// SPDX-License-Identifier: MulanPSL-2.0
// Task Module
//
// Task management functionality for robonix-cli

use crate::config::Config;
use anyhow::Result;
use robonix_core::ros_idl::task::{
    CancelTaskRequest, CancelTaskResponse, SubmitTaskRequest, SubmitTaskResponse, TaskDataRequest,
    TaskDataResponse,
};

pub struct TaskClient {
    _config: Config,
}

impl TaskClient {
    pub fn new(config: Config) -> Result<Self> {
        Ok(Self { _config: config })
    }

    pub async fn submit(
        &self,
        description: String,
        params: serde_json::Value,
    ) -> Result<SubmitTaskResponse> {
        use crate::daemon_client::{DaemonClient, DaemonCommand, DaemonResponse};

        let daemon_client = DaemonClient::new()?;
        daemon_client.ensure_daemon_running().await?;

        // Serialize params to JSON string
        let params_str = serde_json::to_string(&params).unwrap_or_else(|_| "{}".to_string());
        let request = SubmitTaskRequest {
            description,
            params: params_str,
        };
        let request_json = serde_json::to_string(&request)?;
        let response = daemon_client
            .send_command(DaemonCommand::CallSubmitTask {
                request: request_json,
            })
            .await?;

        match response {
            DaemonResponse::SubmitTaskResponse { response } => {
                let resp: SubmitTaskResponse = serde_json::from_str(&response)?;
                Ok(resp)
            }
            DaemonResponse::Error(e) => anyhow::bail!("Daemon error: {}", e),
            _ => anyhow::bail!("Unexpected response type"),
        }
    }

    pub async fn data(&self, task_id: String) -> Result<TaskDataResponse> {
        use crate::daemon_client::{DaemonClient, DaemonCommand, DaemonResponse};

        let daemon_client = DaemonClient::new()?;
        daemon_client.ensure_daemon_running().await?;

        let request = TaskDataRequest { task_id };
        let request_json = serde_json::to_string(&request)?;
        let response = daemon_client
            .send_command(DaemonCommand::CallTaskData {
                request: request_json,
            })
            .await?;

        match response {
            DaemonResponse::TaskDataResponse { response } => {
                let resp: TaskDataResponse = serde_json::from_str(&response)?;
                Ok(resp)
            }
            DaemonResponse::Error(e) => anyhow::bail!("Daemon error: {}", e),
            _ => anyhow::bail!("Unexpected response type"),
        }
    }

    pub async fn cancel(&self, task_id: String) -> Result<CancelTaskResponse> {
        use crate::daemon_client::{DaemonClient, DaemonCommand, DaemonResponse};

        let daemon_client = DaemonClient::new()?;
        daemon_client.ensure_daemon_running().await?;

        let request = CancelTaskRequest { task_id };
        let request_json = serde_json::to_string(&request)?;
        let response = daemon_client
            .send_command(DaemonCommand::CallCancelTask {
                request: request_json,
            })
            .await?;

        match response {
            DaemonResponse::CancelTaskResponse { response } => {
                let resp: CancelTaskResponse = serde_json::from_str(&response)?;
                Ok(resp)
            }
            DaemonResponse::Error(e) => anyhow::bail!("Daemon error: {}", e),
            _ => anyhow::bail!("Unexpected response type"),
        }
    }
}
