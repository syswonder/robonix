// SPDX-License-Identifier: MulanPSL-2.0
// Deploy command: start the full stack from robonix_manifest.yaml

use anyhow::Result;
use robonix_cli::output;
use std::path::Path;

pub async fn execute(config_path: &Path) -> Result<()> {
    output::action("Deploy", &format!("from {}", config_path.display()));
    output::warning("deploy not yet implemented (WIP)");
    Ok(())
}
