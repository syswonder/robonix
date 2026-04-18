// SPDX-License-Identifier: MulanPSL-2.0
// Package-new command: create a new package under packages/ with scaffolding.
//
// Usage:  rbnx package new <name> [--path <dir>]
// Creates:
//   packages/<name>/
//   ├── robonix_manifest.yaml
//   ├── scripts/
//   │   ├── build.sh
//   │   └── start.sh
//   └── src/
//       └── .gitkeep

use anyhow::{Context, Result};
use robonix_cli::output;
use std::fs;
use std::path::Path;

const MANIFEST_TEMPLATE: &str = r#"manifestVersion: 1

build:
  script: scripts/build.sh

package:
  id: com.vendor.{name}
  name: {name}
  version: 0.0.1
  vendor: vendor
  description: TODO — describe your package
  license: Apache-2.0

nodes:
  - id: com.vendor.{name}.main
    type: python
    start: >
      bash scripts/start.sh

interfaces:
  provides: []
    # - id: robonix/prm/camera/rgb
  consumes: []
    # - id: robonix/sys/perception/yolo/gpu_tensor

# Package-level dependencies (other package names this package depends on)
depend: []
  # - com.robonix.pkg.memory
"#;

const BUILD_SH_TEMPLATE: &str = r#"#!/usr/bin/env bash
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
# Examples:
#   colcon build --packages-select {name}
#   pip install -e .
#   cargo build --release

echo "Build complete."
"#;

const START_SH_TEMPLATE: &str = r#"#!/usr/bin/env bash
# Start script for {name}
# Called by: rbnx start -p <package> -n <node>
# Working directory: package root

set -euo pipefail

echo "Starting {name} ..."

# TODO: Add your start command here.
# Examples:
#   source /opt/ros/humble/setup.bash && ros2 launch {name} launch.py
#   python3 -m {name}.main
#   exec ./rbnx-build/{name}
"#;

pub async fn execute(name: &str, path: Option<&Path>) -> Result<()> {
    output::action("Package", &format!("creating new package '{}'", name));

    // Validate package name: must be non-empty, alphanumeric + underscores.
    if name.is_empty() {
        anyhow::bail!("package name must not be empty");
    }
    if !name.chars().all(|c| c.is_alphanumeric() || c == '_' || c == '-') {
        anyhow::bail!(
            "invalid package name '{}': only alphanumeric characters, hyphens and underscores are allowed",
            name
        );
    }

    // Determine where to create:
    // 1. If --path is given, create under that directory.
    // 2. Else if `packages/` exists (inside a workspace), use it.
    // 3. Otherwise, create in the current directory.
    let pkg_root = if let Some(base) = path {
        // Ensure the parent directory exists.
        if !base.exists() {
            fs::create_dir_all(base)
                .with_context(|| format!("failed to create directory '{}'", base.display()))?;
        }
        base.join(name)
    } else {
        let packages_dir = Path::new("packages");
        if packages_dir.is_dir() {
            packages_dir.join(name)
        } else {
            output::warning("'packages/' directory not found — creating package in current directory");
            Path::new(name).to_path_buf()
        }
    };

    if pkg_root.exists() {
        anyhow::bail!(
            "directory '{}' already exists — choose a different name or remove it first",
            pkg_root.display()
        );
    }

    // Create directory structure.
    output::step("Creating", &format!("{}", pkg_root.display()));

    fs::create_dir_all(pkg_root.join("scripts"))
        .with_context(|| format!("failed to create {}/scripts", pkg_root.display()))?;
    fs::create_dir_all(pkg_root.join("src"))
        .with_context(|| format!("failed to create {}/src", pkg_root.display()))?;

    // Write robonix_manifest.yaml.
    let manifest_content = MANIFEST_TEMPLATE.replace("{name}", name);
    let manifest_path = pkg_root.join("robonix_manifest.yaml");
    fs::write(&manifest_path, &manifest_content)
        .with_context(|| format!("failed to write {}", manifest_path.display()))?;
    output::check("robonix_manifest.yaml");

    // Write scripts/build.sh.
    let build_sh = BUILD_SH_TEMPLATE.replace("{name}", name);
    let build_sh_path = pkg_root.join("scripts").join("build.sh");
    fs::write(&build_sh_path, &build_sh)
        .with_context(|| format!("failed to write {}", build_sh_path.display()))?;
    make_executable(&build_sh_path)?;
    output::check("scripts/build.sh");

    // Write scripts/start.sh.
    let start_sh = START_SH_TEMPLATE.replace("{name}", name);
    let start_sh_path = pkg_root.join("scripts").join("start.sh");
    fs::write(&start_sh_path, &start_sh)
        .with_context(|| format!("failed to write {}", start_sh_path.display()))?;
    make_executable(&start_sh_path)?;
    output::check("scripts/start.sh");

    // Write src/.gitkeep (placeholder so git tracks the directory).
    fs::write(pkg_root.join("src").join(".gitkeep"), "")
        .with_context(|| "failed to write src/.gitkeep")?;
    output::check("src/");

    // Summary.
    let pkg_abs = pkg_root.canonicalize().unwrap_or_else(|_| pkg_root.clone());
    output::success(&format!(
        "Package '{}' created at {}",
        name,
        pkg_abs.display()
    ));
    output::info("");
    output::info("Next steps:");
    output::info("  1. Edit robonix_manifest.yaml to describe your package");
    output::info("  2. Edit scripts/build.sh with your build commands");
    output::info("  3. Edit scripts/start.sh with your start commands");
    output::info("  4. Add your package to robonix_workspace.yaml under 'packages:'");
    output::info(&format!("     e.g.  - name: com.vendor.{}", name));
    output::info(&format!("            path: ./packages/{}", name));
    output::info("  5. Add your package:node to deploy/<target>.yaml under 'packages_run:'");
    output::info(&format!("     e.g.  - name: com.vendor.{}:all", name));

    Ok(())
}

/// Make a file executable (chmod +x) on Unix systems.
/// Only adds the execute bit for user/group/other — does not alter read/write bits.
#[cfg(unix)]
fn make_executable(path: &Path) -> Result<()> {
    use std::os::unix::fs::PermissionsExt;
    let metadata = fs::metadata(path)?;
    let mut perms = metadata.permissions();
    perms.set_mode(perms.mode() | 0o111);
    fs::set_permissions(path, perms)?;
    Ok(())
}

#[cfg(not(unix))]
fn make_executable(_path: &Path) -> Result<()> {
    // No-op on non-Unix platforms.
    Ok(())
}
