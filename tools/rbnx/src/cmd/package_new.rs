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

/// Reject names containing path separators or traversal components.
fn validate_name(name: &str) -> Result<()> {
    if name.is_empty() {
        anyhow::bail!("name must not be empty");
    }
    if name.contains('/') || name.contains('\\') || name == ".." || name.starts_with("../") {
        anyhow::bail!("invalid name '{name}': must not contain path separators or '..' components");
    }
    Ok(())
}

fn package_name(name: &str, ns_kind: &str) -> String {
    let normalized = name.replace('-', "_");
    format!("robonix.{ns_kind}.{normalized}")
}

fn python_module_name(name: &str) -> String {
    name.replace('-', "_")
}

fn package_manifest(package_name: &str, ns_kind: &str) -> String {
    format!(
        r#"manifestVersion: 1

build: bash scripts/build.sh

start: bash scripts/start.sh

package:
  name: "{package_name}"
  version: 0.0.1
  description: "TODO: describe what this {ns_kind} package provides."
  tags:
    - {ns_kind}
    - robonix
  maintainers:
    - Your Name <you@example.com>
  license: Apache-2.0

capabilities: []

depends: []
"#
    )
}

pub async fn execute(name: &str, pkg_type: &str, path: Option<&Path>) -> Result<()> {
    validate_name(name)?;

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
            other => {
                anyhow::bail!("unknown package type '{other}'; expected: primitive, service, skill")
            }
        };
        std::env::current_dir()?.join(role_dir).join(name)
    };

    if pkg_dir.exists() {
        anyhow::bail!("directory '{}' already exists", pkg_dir.display());
    }

    output::action("PackageNew", &format!("creating package '{name}'"));

    // Provider class + namespace segment for the generated skeleton.
    // `--path` mode leaves pkg_type at its clap default ("service").
    let (provider_class, ns_kind) = match pkg_type {
        "service" => ("Service", "service"),
        "skill" => ("Skill", "skill"),
        _ => ("Primitive", "primitive"),
    };
    let package_name = package_name(name, ns_kind);
    let module_name = python_module_name(name);

    // Create directory structure; put .gitkeep in empty dirs.
    for sub in ["scripts", "capabilities"] {
        let dir = pkg_dir.join(sub);
        std::fs::create_dir_all(&dir)
            .with_context(|| format!("failed to create {sub}/ directory"))?;
        // .gitkeep so git tracks the empty directory.
        std::fs::write(dir.join(".gitkeep"), "")
            .with_context(|| format!("failed to write {sub}/.gitkeep"))?;
    }

    // package_manifest.yaml — Driver omission canonically selects the shared
    // lifecycle contract, so authors only list domain capabilities here.
    let manifest = package_manifest(&package_name, ns_kind);
    std::fs::write(pkg_dir.join("package_manifest.yaml"), manifest)
        .context("failed to write package_manifest.yaml")?;

    // scripts/build.sh — generate the gRPC / MCP stubs for the contracts
    // this package declares. `robonix-api` auto-discovers the output under
    // <pkg>/rbnx-build/codegen/, so nothing else is needed for a pure
    // Python package. Append your own steps (cargo, pip install -e, docker
    // build, ...) after this line if your package needs them.
    let build_sh = format!(
        r#"#!/usr/bin/env bash
set -euo pipefail
PKG="${{RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}}"

rbnx codegen -p "$PKG"
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

    // scripts/start.sh — launch the provider process. `rbnx path
    // robonix-api` prints the path to the in-tree client library
    // (`<robonix>/pylib/robonix-api`), which start.sh puts on PYTHONPATH so
    // `from robonix_api import ...` resolves. Edit the final line if your
    // entrypoint module differs, or replace it entirely (docker run, ssh,
    // a compiled binary, ...).
    let start_sh = format!(
        r#"#!/usr/bin/env bash
set -eo pipefail
PKG_ROOT="${{RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}}"
cd "$PKG_ROOT"

export PYTHONPATH="$(rbnx path robonix-api):$PKG_ROOT:${{PYTHONPATH:-}}"

exec python3 -m {module_name}.main
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

    // Minimal Python provider skeleton: <pkg>/<name>/{__init__.py, main.py}
    // so `python3 -m {name}.main` (from start.sh) runs out of the box. The
    // author fills in lifecycle handlers + capability declarations.
    let module_dir = pkg_dir.join(&module_name);
    std::fs::create_dir_all(&module_dir).context("failed to create python module directory")?;
    std::fs::write(module_dir.join("__init__.py"), "").context("failed to write __init__.py")?;
    let main_py = format!(
        r#"#!/usr/bin/env python3
"""{name} — Robonix {provider_class_lower} provider."""
from robonix_api import {provider_class}, Ok

# `id` must equal this entry's `name:` in the deploy robonix_manifest.yaml.
# `namespace` groups the capabilities this provider declares.
provider = {provider_class}(id="{name}", namespace="robonix/{ns_kind}/{name}")


@provider.on_init
def init(cfg: dict):
    # TODO: initialise hardware / resources; declare capabilities to atlas.
    return Ok()


if __name__ == "__main__":
    provider.run()
"#,
        provider_class_lower = provider_class.to_lowercase(),
    );
    std::fs::write(module_dir.join("main.py"), main_py).context("failed to write main.py")?;

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
    output::sub_step(&format!("{module_name}/main.py  (provider skeleton)"));
    output::sub_step("capabilities/  (.gitkeep)");
    output::sub_step(".gitignore");

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn generated_manifest_has_complete_package_metadata() {
        let root: serde_yaml::Value =
            serde_yaml::from_str(&package_manifest("robonix.service.example", "service")).unwrap();
        let package = root
            .get("package")
            .and_then(|value| value.as_mapping())
            .unwrap();

        assert_eq!(
            root.get("manifestVersion").and_then(|value| value.as_u64()),
            Some(1)
        );
        for key in [
            "name",
            "version",
            "description",
            "license",
            "tags",
            "maintainers",
        ] {
            assert!(
                package.contains_key(serde_yaml::Value::String(key.into())),
                "missing package.{key}"
            );
        }
        assert!(root.get("start").and_then(|value| value.as_str()).is_some());
        assert!(
            root.get("capabilities")
                .and_then(|value| value.as_sequence())
                .is_some()
        );
        let capabilities = root
            .get("capabilities")
            .and_then(|value| value.as_sequence())
            .unwrap();
        assert!(capabilities.is_empty());
        assert!(
            root.get("depends")
                .and_then(|value| value.as_sequence())
                .is_some()
        );
    }
}
