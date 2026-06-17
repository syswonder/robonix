// SPDX-License-Identifier: MulanPSL-2.0
// `rbnx init <name>` — scaffold a new robonix project.

use anyhow::{Context, Result};
use robonix_cli::output;
use std::path::Path;

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

pub async fn execute(name: &str, path: Option<&Path>) -> Result<()> {
    validate_name(name)?;

    let base = match path {
        Some(p) => p.to_path_buf(),
        None => std::env::current_dir()?,
    };
    let project_dir = base.join(name);

    if project_dir.exists() {
        anyhow::bail!("directory '{}' already exists", project_dir.display());
    }

    output::action("Init", &format!("creating project '{name}'"));

    // Create directory structure.
    for sub in ["primitives", "services", "skills"] {
        std::fs::create_dir_all(project_dir.join(sub))
            .with_context(|| format!("failed to create {sub}/ directory"))?;
    }

    // robonix_manifest.yaml (deployment manifest).
    let manifest = format!(
        r#"name: {name}

env:
  LOG: "INFO"

system:
  atlas:
    listen: 127.0.0.1:50051
    log: info
  executor:
    listen: 127.0.0.1:50061
    log: info
  pilot:
    listen: 127.0.0.1:50071
    log: info
    vlm:
      upstream: ${{VLM_BASE_URL}}
      api_key: ${{VLM_API_KEY}}
      model: ${{VLM_MODEL}}
      api_format: openai
  liaison:
    listen: 127.0.0.1:50081
    log: info
  memory: []

primitive: []

service: []

skill: []
"#
    );
    std::fs::write(project_dir.join("robonix_manifest.yaml"), manifest)
        .context("failed to write robonix_manifest.yaml")?;

    // .gitignore
    let gitignore = "\
rbnx-build/
rbnx-boot/
__pycache__/
*.pyc
.venv/
";
    std::fs::write(project_dir.join(".gitignore"), gitignore)
        .context("failed to write .gitignore")?;

    output::success(&format!(
        "Project '{name}' created at {}",
        project_dir.display()
    ));
    output::sub_step("robonix_manifest.yaml");
    output::sub_step("primitives/");
    output::sub_step("services/");
    output::sub_step("skills/");
    output::sub_step(".gitignore");

    Ok(())
}
