// SPDX-License-Identifier: MulanPSL-2.0
// PackageNew command: create a new package skeleton
//
// Generated structure:
//   <role_dir>/<name>/
//   ├── package_manifest.yaml
//   ├── scripts/
//   │   ├── build.sh
//   │   └── start.sh
//   ├── src/
//   └── capabilities/

use anyhow::{Context, Result};
use robonix_cli::output;
use std::fs;
use std::path::Path;

const MANIFEST_TEMPLATE: &str = r#"manifestVersion: 1

build:
  script: scripts/build.sh

start: bash scripts/start.sh

package:
  name: com.vendor.{name}
  version: 0.0.1
  vendor: vendor
  description: TODO — describe your package
  license: Apache-2.0

nodes:
  - id: com.vendor.{name}.main
    type: python
    start: bash scripts/start.sh

capabilities: []

depends: []
"#;

const BUILD_SH: &str = r#"#!/usr/bin/env bash
# Build script for {name}
# Called by: rbnx build
# Working directory: package root
# Environment:
#   RBNX_PACKAGE_ROOT — absolute path to this package
#   RBNX_BUILD_CLEAN  — set to "1" when --clean is passed

set -euo pipefail

echo "Building {name} ..."

if [ "${RBNX_BUILD_CLEAN:-}" = "1" ]; then
    echo "Clean build requested — removing rbnx-build/"
    rm -rf rbnx-build
fi

mkdir -p rbnx-build

# TODO: Add your build steps here.

echo "Build complete."
"#;

const START_SH: &str = r#"#!/usr/bin/env bash
# Start script for {name}
# Called by: rbnx deploy / rbnx start
# Working directory: package root

set -euo pipefail

echo "Starting {name} ..."

# TODO: Add your start command here.
# Examples:
#   python3 -m {name}.main
#   exec ./rbnx-build/{name}
"#;

pub async fn execute(name: &str, pkg_type: &str, path: Option<&Path>) -> Result<()> {
    let role_dir = match pkg_type {
        "primitive" => "primitives",
        "service" => "services",
        "skill" => "skills",
        other => anyhow::bail!(
            "unknown package type '{}': expected 'primitive', 'service', or 'skill'",
            other
        ),
    };

    let target = path
        .map(|p| p.to_path_buf())
        .unwrap_or_else(|| std::path::PathBuf::from(role_dir).join(name));

    if target.exists() {
        anyhow::bail!("directory '{}' already exists", target.display());
    }

    output::action(
        "PackageNew",
        &format!("creating {} '{}' at {}", pkg_type, name, target.display()),
    );

    // Create directory structure.
    fs::create_dir_all(target.join("scripts"))
        .with_context(|| format!("failed to create {}/scripts", target.display()))?;
    fs::create_dir_all(target.join("src"))?;
    fs::create_dir_all(target.join("capabilities"))?;

    // Write package_manifest.yaml.
    let manifest = MANIFEST_TEMPLATE.replace("{name}", name);
    fs::write(target.join("package_manifest.yaml"), manifest)?;
    output::check("package_manifest.yaml");

    // Write scripts/build.sh.
    let build_sh = BUILD_SH.replace("{name}", name);
    let build_path = target.join("scripts").join("build.sh");
    fs::write(&build_path, build_sh)?;
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        fs::set_permissions(&build_path, fs::Permissions::from_mode(0o755))?;
    }
    output::check("scripts/build.sh");

    // Write scripts/start.sh.
    let start_sh = START_SH.replace("{name}", name);
    let start_path = target.join("scripts").join("start.sh");
    fs::write(&start_path, start_sh)?;
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        fs::set_permissions(&start_path, fs::Permissions::from_mode(0o755))?;
    }
    output::check("scripts/start.sh");

    output::check("src/");
    output::check("capabilities/");

    output::success(&format!(
        "Package '{}' created at {}",
        name,
        target.display()
    ));
    output::info("Next steps:");
    output::info(&format!(
        "  Edit {}/package_manifest.yaml to set package name, capabilities, and depends",
        target.display()
    ));
    output::info(&format!(
        "  Edit {}/scripts/build.sh to add build steps",
        target.display()
    ));
    output::info(&format!(
        "  Edit {}/scripts/start.sh to add start command",
        target.display()
    ));

    Ok(())
}
