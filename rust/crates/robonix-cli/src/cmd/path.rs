// SPDX-License-Identifier: MulanPSL-2.0
// `rbnx path <key>` — print a well-known path rooted in the robonix source tree.
// Used by package build.sh scripts in place of fragile relative-dir traversal:
//   CAPABILITIES_DIR="$(rbnx path capabilities)"
//   INTERFACES_LIB="$(rbnx path interfaces-lib)"

use anyhow::Result;
use robonix_cli::{Config, config::SourcePathKey};
use std::str::FromStr;

pub async fn execute(config: Config, key: String) -> Result<()> {
    let parsed = SourcePathKey::from_str(&key).map_err(|e| anyhow::anyhow!(e))?;
    let p = config.resolve_source_path(parsed)?;
    // Print exactly one line with the absolute path — safe for `$(...)` capture.
    println!("{}", p.display());
    Ok(())
}
