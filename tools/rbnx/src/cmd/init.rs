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

fn robot_slug(name: &str) -> String {
    name.replace('-', "_")
}

fn deployment_manifest(name: &str, catalog_name: &str) -> String {
    format!(
        r#"manifestVersion: 1
name: {name}

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

env: {{}}

system:
  atlas:
    listen: 127.0.0.1:50051
    log: info
  executor:
    listen: 127.0.0.1:50061
    log: info
  soma:
    robot_yaml: ./soma.yaml
  vitals:
    listen: 127.0.0.1:50093
  pilot:
    listen: 127.0.0.1:50071
    log: info
    vlm:
      upstream: ${{VLM_BASE_URL}}
      api_key: ${{VLM_API_KEY}}
      model: ${{VLM_MODEL}}
  liaison:
    listen: 127.0.0.1:50081
    log: info

primitive: []

service: []

skill: []
"#
    )
}

fn soma_manifest(name: &str, robot_file: &str, robot_id: &str) -> String {
    format!(
        r#"urdf:
  path: ./urdf/{robot_file}
  root_link: base_link
  model_name: {robot_id}

robot:
  id: {robot_id}
  display_name: "{name}"
  family: mobile_robot
  root_part: base
  dimensions: {{ length_m: 0.50, width_m: 0.50, height_m: 1.00 }}
  mass_kg: 20.0
  exports: []
  components:
    - id: base
      type: mobile_base
      urdf_link: base_link
      exports: []

description:
  summary: "TODO: describe this robot deployment."
  can_do: []
  cannot_do: []
  notes:
    - "Fill in robot.exports and robot.components as you add primitives, services, and skills."
    - "Keep provider_id values in soma.yaml aligned with the corresponding name entries in robonix_manifest.yaml."
"#
    )
}

fn minimal_urdf(robot_id: &str) -> String {
    format!(
        r#"<?xml version="1.0"?>
<robot name="{robot_id}">
  <link name="base_link"/>
</robot>
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
    for subdir in ["primitives", "services", "skills", "urdf"] {
        std::fs::create_dir_all(project_dir.join(subdir))
            .with_context(|| format!("failed to create {subdir}/ directory"))?;
    }
    for subdir in ["primitives", "services", "skills"] {
        std::fs::write(project_dir.join(subdir).join(".gitkeep"), "")
            .with_context(|| format!("failed to write {subdir}/.gitkeep"))?;
    }

    let catalog_name = robot_catalog_name(name);
    let robot_id = robot_slug(name);
    let robot_file = format!("{name}.urdf");
    std::fs::write(
        project_dir.join("robonix_manifest.yaml"),
        deployment_manifest(name, &catalog_name),
    )
    .context("failed to write robonix_manifest.yaml")?;
    std::fs::write(
        project_dir.join("soma.yaml"),
        soma_manifest(name, &robot_file, &robot_id),
    )
    .context("failed to write soma.yaml")?;
    std::fs::write(
        project_dir.join("urdf").join(&robot_file),
        minimal_urdf(&robot_id),
    )
    .context("failed to write URDF template")?;

    let gitignore = "\
rbnx-build/\nrbnx-boot/\nlogs/\n__pycache__/\n*.pyc\n.venv/\n";
    std::fs::write(project_dir.join(".gitignore"), gitignore)
        .context("failed to write .gitignore")?;

    output::success(&format!(
        "Robot deployment '{name}' created at {}",
        project_dir.display()
    ));
    output::sub_step("robonix_manifest.yaml");
    output::sub_step("soma.yaml");
    output::sub_step(&format!("urdf/{robot_file}"));
    output::sub_step("primitives/");
    output::sub_step("services/");
    output::sub_step("skills/");
    output::sub_step(".gitignore");

    Ok(())
}
