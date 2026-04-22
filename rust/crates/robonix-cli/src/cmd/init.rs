// SPDX-License-Identifier: MulanPSL-2.0
// Init command: create a new robonix project skeleton

use anyhow::Result;
use robonix_cli::output;
use std::path::Path;

pub async fn execute(name: &str, path: Option<&Path>) -> Result<()> {
    let target = path
        .map(|p| p.to_path_buf())
        .unwrap_or_else(|| std::path::PathBuf::from(name));
    output::action("Init", &format!("project '{}' at {}", name, target.display()));
    output::warning("init not yet implemented (WIP)");
    Ok(())
}
