// SPDX-License-Identifier: MulanPSL-2.0
// `rbnx package-new <name>` — scaffold a new package.
//
// Two modes:
//   1. `rbnx package-new my_cam --path ./primitives`
//       → creates at `./primitives/my_cam` and infers primitive/service/skill
//         from the parent path when possible.
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

fn infer_pkg_type(pkg_type: &str, path: Option<&Path>) -> Result<&'static str> {
    if let Some(path) = path {
        for component in path.components() {
            if let Some(part) = component.as_os_str().to_str() {
                match part {
                    "primitives" => return Ok("primitive"),
                    "services" => return Ok("service"),
                    "skills" => return Ok("skill"),
                    _ => {}
                }
            }
        }
    }
    match pkg_type {
        "primitive" => Ok("primitive"),
        "service" => Ok("service"),
        "skill" => Ok("skill"),
        other => {
            anyhow::bail!("unknown package type '{other}'; expected: primitive, service, skill")
        }
    }
}

fn role_dir(pkg_type: &str) -> &'static str {
    match pkg_type {
        "primitive" => "primitives",
        "service" => "services",
        "skill" => "skills",
        _ => unreachable!("pkg_type is validated by infer_pkg_type"),
    }
}

fn provider_class(pkg_type: &str) -> &'static str {
    match pkg_type {
        "primitive" => "Primitive",
        "service" => "Service",
        "skill" => "Skill",
        _ => unreachable!("pkg_type is validated by infer_pkg_type"),
    }
}

fn config_spec(pkg_type: &str) -> String {
    format!(
        r#"# Documentation-only template for the `config:` block in robonix_manifest.yaml.
# `rbnx` does not parse this file; your provider's `on_init(cfg)` does.
fields:
  example_field:
    type: string
    required: false
    default: ""
    description: "TODO: describe the config accepted by this {pkg_type} package."
"#
    )
}

fn capability_md_template(name: &str) -> String {
    format!(
        r#"---
description: "TODO: describe when and why the agent should use the `{name}` skill."
---

# Overview

TODO: describe this skill's purpose, expected inputs, side effects, and operator notes.
"#
    )
}

fn driver_contract(ns_kind: &str, name: &str) -> String {
    format!(
        r#"[contract]
id      = "robonix/{ns_kind}/{name}/driver"
version = "1"
kind    = "{ns_kind}"
idl     = "lifecycle/srv/Driver.srv"

[mode]
type = "rpc"
"#
    )
}

fn skill_run_contract(name: &str, idl_lib_name: &str) -> String {
    format!(
        r#"[contract]
id      = "robonix/skill/{name}/run"
version = "1"
kind    = "skill"
idl     = "{idl_lib_name}/srv/Run.srv"

[mode]
type = "rpc"

[semantics]
user_invocable = true
"#
    )
}

fn skill_run_srv() -> &'static str {
    r#"string instruction
---
bool accepted
string message
"#
}

fn package_manifest(name: &str, ns_kind: &str, package_name: &str, skill_idl_lib: &str) -> String {
    let capabilities = if ns_kind == "skill" {
        format!(
            "capabilities:\n  - name: robonix/{ns_kind}/{name}/driver\n    path: capabilities/driver.v1.toml\n  - name: robonix/{ns_kind}/{name}/run\n    path: capabilities/run.v1.toml"
        )
    } else {
        format!(
            "capabilities:\n  - name: robonix/{ns_kind}/{name}/driver\n    path: capabilities/driver.v1.toml"
        )
    };

    let _ = skill_idl_lib;
    format!(
        r#"manifestVersion: 1

build: bash scripts/build.sh

start: bash scripts/start.sh

package:
  name: "{package_name}"
  version: 0.1.0
  description: "TODO: describe what this {ns_kind} package provides."
  tags:
    - {ns_kind}
    - example
    - robonix
  maintainers:
    - Your Name <you@example.com>
  license: Apache-2.0

{capabilities}

depends: []
"#
    )
}

fn build_sh(name: &str) -> String {
    format!(
        r#"#!/usr/bin/env bash
set -euo pipefail
PKG="${{RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}}"

rbnx codegen --mcp -p "$PKG"
echo "[{name}] build done"
"#
    )
}

fn start_sh(source_module: &str) -> String {
    format!(
        r#"#!/usr/bin/env bash
set -eo pipefail
PKG_ROOT="${{RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}}"
cd "$PKG_ROOT"

export PYTHONPATH="$(rbnx path robonix-api):$PKG_ROOT:${{PYTHONPATH:-}}"

exec python3 -m {source_module}.main
"#
    )
}

fn provider_main(name: &str, ns_kind: &str, provider_class: &str) -> String {
    format!(
        r#"#!/usr/bin/env python3
"""{name} — Robonix {provider_class_lower} provider."""
from robonix_api import {provider_class}, Ok

# `id` must equal this entry's `name:` in the deploy robonix_manifest.yaml.
# Adjust `namespace` if you later switch to a shared contract family.
provider = {provider_class}(id="{name}", namespace="robonix/{ns_kind}/{name}")


@provider.on_init
def init(cfg: dict):
    # TODO: validate config and perform light dependency checks here.
    return Ok()


@provider.on_activate
def activate():
    # TODO: acquire hot resources and declare business capabilities here.
    return Ok()


@provider.on_deactivate
def deactivate():
    # TODO: release any resources acquired in on_activate.
    return Ok()


if __name__ == "__main__":
    provider.run()
"#,
        provider_class_lower = provider_class.to_lowercase(),
    )
}

fn skill_main(name: &str, idl_lib_name: &str) -> String {
    format!(
        r#"#!/usr/bin/env python3
"""{name} — Robonix skill provider."""
from robonix_api import Skill, Ok
from {idl_lib_name}_mcp import Run_Request, Run_Response

# `id` must equal this entry's `name:` in the deploy robonix_manifest.yaml.
provider = Skill(id="{name}", namespace="robonix/skill/{name}")


@provider.on_init
def init(cfg: dict):
    # TODO: parse config and validate light dependencies here.
    return Ok()


@provider.on_activate
def activate():
    # Skills must allocate heavy resources lazily here.
    return Ok()


@provider.on_deactivate
def deactivate():
    # Skills must release the resources acquired in on_activate here.
    return Ok()


@provider.mcp("robonix/skill/{name}/run")
def run(req: Run_Request) -> Run_Response:
    """Execute this skill's main task."""
    instruction = (req.instruction or "").strip()
    if not instruction:
        return Run_Response(accepted=False, message="instruction is empty")
    return Run_Response(
        accepted=True,
        message=f"TODO: implement skill logic for {{instruction}}",
    )


if __name__ == "__main__":
    provider.run()
"#
    )
}

pub async fn execute(name: &str, pkg_type: &str, path: Option<&Path>) -> Result<()> {
    validate_name(name)?;
    let pkg_type = infer_pkg_type(pkg_type, path)?;

    let base_dir: PathBuf = if let Some(p) = path {
        if p.is_absolute() {
            p.to_path_buf()
        } else {
            std::env::current_dir()?.join(p)
        }
    } else {
        std::env::current_dir()?.join(role_dir(pkg_type))
    };
    let pkg_dir = base_dir.join(name);

    if pkg_dir.exists() {
        anyhow::bail!("directory '{}' already exists", pkg_dir.display());
    }

    output::action("PackageNew", &format!("creating package '{name}'"));

    let ns_kind = pkg_type;
    let provider_class = provider_class(pkg_type);
    let package_name = package_name(name, ns_kind);
    let package_stem = python_module_name(name);
    let source_module = if pkg_type == "skill" {
        format!("{package_stem}_skill")
    } else {
        package_stem.clone()
    };

    std::fs::create_dir_all(&pkg_dir).context("failed to create package directory")?;
    std::fs::create_dir_all(pkg_dir.join("scripts"))
        .context("failed to create scripts/ directory")?;
    std::fs::create_dir_all(pkg_dir.join("capabilities"))
        .context("failed to create capabilities/ directory")?;

    let module_dir = pkg_dir.join(&source_module);
    std::fs::create_dir_all(&module_dir).context("failed to create python module directory")?;
    std::fs::write(module_dir.join("__init__.py"), "").context("failed to write __init__.py")?;

    std::fs::write(
        pkg_dir.join("package_manifest.yaml"),
        package_manifest(name, ns_kind, &package_name, &package_stem),
    )
    .context("failed to write package_manifest.yaml")?;
    std::fs::write(pkg_dir.join("config.spec"), config_spec(pkg_type))
        .context("failed to write config.spec")?;
    std::fs::write(pkg_dir.join("scripts/build.sh"), build_sh(name))
        .context("failed to write scripts/build.sh")?;
    std::fs::write(pkg_dir.join("scripts/start.sh"), start_sh(&source_module))
        .context("failed to write scripts/start.sh")?;
    std::fs::write(
        pkg_dir.join("capabilities/driver.v1.toml"),
        driver_contract(ns_kind, name),
    )
    .context("failed to write capabilities/driver.v1.toml")?;

    let main_py = if pkg_type == "skill" {
        skill_main(name, &package_stem)
    } else {
        provider_main(name, ns_kind, provider_class)
    };
    std::fs::write(module_dir.join("main.py"), main_py).context("failed to write main.py")?;

    if pkg_type == "skill" {
        let skill_srv_dir = pkg_dir
            .join("capabilities/lib")
            .join(&package_stem)
            .join("srv");
        std::fs::create_dir_all(&skill_srv_dir)
            .context("failed to create skill capabilities lib directory")?;
        std::fs::write(
            pkg_dir.join("capabilities/run.v1.toml"),
            skill_run_contract(name, &package_stem),
        )
        .context("failed to write capabilities/run.v1.toml")?;
        std::fs::write(skill_srv_dir.join("Run.srv"), skill_run_srv())
            .context("failed to write capabilities/lib/.../Run.srv")?;
        std::fs::write(pkg_dir.join("CAPABILITY.md"), capability_md_template(name))
            .context("failed to write CAPABILITY.md")?;
    }

    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        std::fs::set_permissions(
            pkg_dir.join("scripts/build.sh"),
            std::fs::Permissions::from_mode(0o755),
        )?;
        std::fs::set_permissions(
            pkg_dir.join("scripts/start.sh"),
            std::fs::Permissions::from_mode(0o755),
        )?;
    }

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
    output::sub_step("config.spec");
    output::sub_step("scripts/build.sh");
    output::sub_step("scripts/start.sh");
    output::sub_step(&format!("capabilities/driver.v1.toml"));
    if pkg_type == "skill" {
        output::sub_step("capabilities/run.v1.toml");
        output::sub_step(&format!("capabilities/lib/{package_stem}/srv/Run.srv"));
        output::sub_step("CAPABILITY.md");
    }
    output::sub_step(&format!("{source_module}/main.py  (provider skeleton)"));
    output::sub_step(".gitignore");

    Ok(())
}
