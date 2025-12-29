// SPDX-License-Identifier: MulanPSL-2.0
// Restart Command Module
//
// Restart command implementation for robonix-cli

use super::start;
use super::stop;
use crate::output;
use crate::Config;
use anyhow::Result;
use tokio::time::{sleep, Duration};

pub async fn execute(config: Config, target: String) -> Result<()> {
    output::action("Restarting", &format!("item(s) matching: {}", target));

    // Step 1: Stop running processes
    if let Err(e) = stop::execute(config.clone(), target.clone()).await {
        output::warning(&format!("Some processes failed to stop: {}", e));
        // Continue anyway to try starting
    }

    // Wait a bit for processes to fully stop
    output::sub_step("Waiting for processes to stop...");
    sleep(Duration::from_millis(200)).await;

    // Step 2: Start processes
    start::execute(config, target).await
}
