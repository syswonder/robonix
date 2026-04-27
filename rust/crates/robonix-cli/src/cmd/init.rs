// SPDX-License-Identifier: MulanPSL-2.0
// `rbnx init <name>` — scaffold a new robonix project.

use anyhow::{Context, Result};
use robonix_cli::output;
use std::path::Path;

pub async fn execute(name: &str, path: Option<&Path>) -> Result<()> {
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
      upstream: ""
      api_key: ""
      model: ""
      api_format: openai
  liaison:
    listen: 127.0.0.1:50081
    log: info
  nexus:
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
rbnx-deploy/
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
