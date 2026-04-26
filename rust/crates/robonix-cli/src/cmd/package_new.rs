// SPDX-License-Identifier: MulanPSL-2.0
// `rbnx package-new <name> --type <primitive|service|skill>` — scaffold a new package.

use anyhow::{Context, Result};
use robonix_cli::output;
use std::path::Path;

pub async fn execute(name: &str, pkg_type: &str, path: Option<&Path>) -> Result<()> {
    let role_dir = match pkg_type {
        "primitive" => "primitives",
        "service" => "services",
        "skill" => "skills",
        other => anyhow::bail!(
            "unknown package type '{other}'; expected: primitive, service, skill"
        ),
    };

    let base = match path {
        Some(p) => p.to_path_buf(),
        None => std::env::current_dir()?,
    };
    let pkg_dir = base.join(role_dir).join(name);

    if pkg_dir.exists() {
        anyhow::bail!("directory '{}' already exists", pkg_dir.display());
    }

    output::action(
        "PackageNew",
        &format!("creating {pkg_type} package '{name}'"),
    );

    // Create directory structure.
    for sub in ["scripts", "capabilities"] {
        std::fs::create_dir_all(pkg_dir.join(sub))
            .with_context(|| format!("failed to create {sub}/ directory"))?;
    }

    // package_manifest.yaml
    let cap_name = match pkg_type {
        "primitive" => format!("robonix/primitive/{name}/driver"),
        "service" => format!("robonix/srv/{name}"),
        "skill" => format!("robonix/skill/{name}"),
        _ => unreachable!(),
    };
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

capabilities:
  - name: {cap_name}

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

    // .gitignore
    std::fs::write(
        pkg_dir.join(".gitignore"),
        "rbnx-build/\n__pycache__/\n*.pyc\n.venv/\n",
    )
    .context("failed to write .gitignore")?;

    output::success(&format!(
        "Package '{name}' ({pkg_type}) created at {}",
        pkg_dir.display()
    ));
    output::sub_step("package_manifest.yaml");
    output::sub_step("scripts/build.sh");
    output::sub_step("scripts/start.sh");
    output::sub_step("capabilities/");
    output::sub_step(".gitignore");

    Ok(())
}
