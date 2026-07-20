// SPDX-License-Identifier: MulanPSL-2.0
// `rbnx init <name>` — scaffold a robot deployment directory.

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

fn robot_catalog_name(name: &str) -> String {
    let suffix = name.strip_prefix("robot-").unwrap_or(name);
    format!("robonix.robot.{}", suffix.replace('-', "."))
}

fn deployment_manifest(name: &str) -> String {
    let catalog_name = robot_catalog_name(name);
    format!(
        r#"manifestVersion: 1
catalog:
  name: {catalog_name}
  version: 0.1.0
  description: "TODO: describe this robot deployment."
  license: Apache-2.0
  tags:
    - robot
    - deploy
    - robonix
  maintainers:
    - Your Name <you@example.com>

name: {name}

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
    listen: 0.0.0.0:50081
    log: info

primitive: []

service: []

skill: []
"#
    )
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

    output::action("Init", &format!("creating robot deployment '{name}'"));

    std::fs::create_dir_all(&project_dir).context("failed to create deployment directory")?;

    // robonix_manifest.yaml (deployment manifest).
    let manifest = deployment_manifest(name);
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
        "Robot deployment '{name}' created at {}",
        project_dir.display()
    ));
    output::sub_step("robonix_manifest.yaml");
    output::sub_step(".gitignore");

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn generated_manifest_has_complete_robot_catalog_metadata() {
        let root: serde_yaml::Value =
            serde_yaml::from_str(&deployment_manifest("robot-example-mobile")).unwrap();
        let catalog = root
            .get("catalog")
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
                catalog.contains_key(serde_yaml::Value::String(key.into())),
                "missing catalog.{key}"
            );
        }
    }
}
