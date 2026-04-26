// SPDX-License-Identifier: MulanPSL-2.0
// `rbnx package-new <name>` — scaffold a new package.
//
// Two modes:
//   1. `rbnx package-new my_cam --path ./primitives/my_cam`
//       → creates directly at the given path (--type is ignored).
//   2. `rbnx package-new my_cam -t primitive`
//       → creates at `<cwd>/<role_dir>/my_cam` where role_dir is
//         derived from --type (primitives/ services/ skills/).

use anyhow::{Context, Result};
use robonix_cli::output;
use std::path::{Path, PathBuf};

pub async fn execute(name: &str, pkg_type: &str, path: Option<&Path>) -> Result<()> {
    let pkg_dir: PathBuf = if let Some(p) = path {
        // --path given: use it directly, no type inference needed.
        if p.is_absolute() {
            p.to_path_buf()
        } else {
            std::env::current_dir()?.join(p)
        }
    } else {
        // No --path: derive from --type.
        let role_dir = match pkg_type {
            "primitive" => "primitives",
            "service" => "services",
            "skill" => "skills",
            other => anyhow::bail!(
                "unknown package type '{other}'; expected: primitive, service, skill"
            ),
        };
        std::env::current_dir()?.join(role_dir).join(name)
    };

    if pkg_dir.exists() {
        anyhow::bail!("directory '{}' already exists", pkg_dir.display());
    }

    output::action("PackageNew", &format!("creating package '{name}'"));

    // Create directory structure; put .gitkeep in empty dirs.
    for sub in ["scripts", "capabilities"] {
        let dir = pkg_dir.join(sub);
        std::fs::create_dir_all(&dir)
            .with_context(|| format!("failed to create {sub}/ directory"))?;
        // .gitkeep so git tracks the empty directory.
        std::fs::write(dir.join(".gitkeep"), "")
            .with_context(|| format!("failed to write {sub}/.gitkeep"))?;
    }

    // package_manifest.yaml — capabilities default to empty.
    let manifest = format!(
        r#"manifestVersion: 1

build: bash scripts/build.sh

start: bash scripts/start.sh

package:
  name: com.vendor.{name}
  version: 0.0.1
  vendor: vendor
  description: TODO
  license: Apache-2.0

capabilities: []

depends: []
"#
    );
    std::fs::write(pkg_dir.join("package_manifest.yaml"), manifest)
        .context("failed to write package_manifest.yaml")?;

    // scripts/build.sh
    let build_sh = format!(
        r#"#!/usr/bin/env bash
set -euo pipefail
echo "[{name}] building..."
mkdir -p rbnx-build
echo "[{name}] build done"
"#
    );
    std::fs::write(pkg_dir.join("scripts/build.sh"), build_sh)
        .context("failed to write scripts/build.sh")?;
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        std::fs::set_permissions(
            pkg_dir.join("scripts/build.sh"),
            std::fs::Permissions::from_mode(0o755),
        )?;
    }

    // scripts/start.sh
    let start_sh = format!(
        r#"#!/usr/bin/env bash
set -euo pipefail
echo "[{name}] starting..."
# TODO: replace with actual start command
sleep infinity
"#
    );
    std::fs::write(pkg_dir.join("scripts/start.sh"), start_sh)
        .context("failed to write scripts/start.sh")?;
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        std::fs::set_permissions(
            pkg_dir.join("scripts/start.sh"),
            std::fs::Permissions::from_mode(0o755),
        )?;
    }

    // Remove .gitkeep from scripts/ since it now has real files.
    let _ = std::fs::remove_file(pkg_dir.join("scripts/.gitkeep"));

    // .gitignore
    std::fs::write(
        pkg_dir.join(".gitignore"),
        "rbnx-build/\n__pycache__/\n*.pyc\n.venv/\n",
    )
    .context("failed to write .gitignore")?;

    output::success(&format!(
        "Package '{name}' created at {}",
        pkg_dir.display()
    ));
    output::sub_step("package_manifest.yaml");
    output::sub_step("scripts/build.sh");
    output::sub_step("scripts/start.sh");
    output::sub_step("capabilities/  (.gitkeep)");
    output::sub_step(".gitignore");

    Ok(())
}
