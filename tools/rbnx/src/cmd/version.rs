// SPDX-License-Identifier: MulanPSL-2.0
// `rbnx version [--json]` — the build facts behind `rbnx --version`, in a
// form a script can read.

use anyhow::Result;
use robonix_cli::{Config, SourcePathKey, build_info::BuildInfo};

pub fn execute(config: &Config, json: bool) -> Result<()> {
    let info = BuildInfo::get();
    if !json {
        println!("rbnx {}", info.summary());
        return Ok(());
    }
    let mut report = serde_json::to_value(&info)?;
    report["source_root"] = serde_json::json!(config.resolve_source_path(SourcePathKey::Root).ok());
    println!("{report}");
    Ok(())
}
