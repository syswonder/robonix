// SPDX-License-Identifier: MulanPSL-2.0
// Init command: generate a new robonix workspace skeleton.
//
// Usage:  rbnx init <name> [--path <dir>]
// Creates:
//   <name>/
//   ├── robonix_workspace.yaml   (upstream workspace config template)
//   ├── deploy/
//   │   └── config.yaml          (project-level deploy config)
//   ├── packages/                (empty directory for user packages)
//   └── .gitignore

use anyhow::{Context, Result};
use robonix_cli::output;
use std::fs;
use std::path::Path;

const WORKSPACE_YAML_TEMPLATE: &str = r#"workspace: {name}

# Global environment variables (injected into all child processes)
env:
  VLM_API_KEY: ""
  VLM_BASE_URL: ""
  VLM_MODEL: ""

# ── Robonix runtime core components (started in dependency order) ──────────
runtime:
  - name: robonix-atlas
    endpoint: "127.0.0.1:50051"
    description: Service registry and discovery

  - name: robonix-executor
    endpoint: "127.0.0.1:50061"
    depends_on:
      - robonix-atlas
    description: Task scheduling and execution engine

  - name: robonix-pilot
    endpoint: "127.0.0.1:50071"
    depends_on:
      - robonix-atlas
      - robonix-executor
    description: Autonomous decision making and planner

  - name: robonix-liaison
    endpoint: "127.0.0.1:50081"
    depends_on:
      - robonix-atlas
    description: External communication and bridge gateway

# ── Upstream system services (started after runtime is ready) ──────────────
system: []
  # Example:
  # - name: robonix.sys.model.vlm
  #   contract_id: robonix/sys/model/vlm/chat
  #   node_id: com.robonix.services.vlm
  #   depends_on:
  #     - robonix-atlas
  #     - robonix-executor
  #   description: VLM vision-language model
"#;

const CONFIG_YAML_TEMPLATE: &str = r#"upstream_config: ../robonix_workspace.yaml
target: default

# Downstream packages
packages: []
  # Example:
  # - name: my-package
  #   path: ./packages/my_package
  #   depends_on: []
"#;

const GITIGNORE_TEMPLATE: &str = r#"# Build artifacts
rbnx-build/
.rbnx-built

# OS files
.DS_Store
Thumbs.db

# IDE
.idea/
.vscode/
*.swp
*.swo

# ROS 2 build artifacts
build/
install/
log/
"#;

pub async fn execute(name: &str, path: Option<&Path>) -> Result<()> {
    output::action("Init", &format!("creating workspace '{}'", name));

    let base_dir = path.unwrap_or_else(|| Path::new("."));
    let workspace_dir = base_dir.join(name);

    // Check if directory already exists.
    if workspace_dir.exists() {
        anyhow::bail!(
            "directory '{}' already exists — choose a different name or remove it first",
            workspace_dir.display()
        );
    }

    // Ensure the parent directory exists.
    if !base_dir.exists() {
        fs::create_dir_all(base_dir)
            .with_context(|| format!("failed to create parent directory '{}'", base_dir.display()))?;
    }

    // Create directory structure.
    output::step("Creating", "workspace directory structure");

    fs::create_dir_all(workspace_dir.join("packages"))
        .with_context(|| format!("failed to create {}/packages", workspace_dir.display()))?;
    output::check("packages/");

    // Write robonix_workspace.yaml.
    let workspace_yaml = WORKSPACE_YAML_TEMPLATE.replace("{name}", name);
    let workspace_yaml_path = workspace_dir.join("robonix_workspace.yaml");
    fs::write(&workspace_yaml_path, &workspace_yaml).with_context(|| {
        format!(
            "failed to write {}",
            workspace_yaml_path.display()
        )
    })?;
    output::check("robonix_workspace.yaml");

    // Create deploy/ directory and write config.yaml.
    fs::create_dir_all(workspace_dir.join("deploy"))
        .with_context(|| format!("failed to create {}/deploy", workspace_dir.display()))?;
    let config_yaml_path = workspace_dir.join("deploy").join("config.yaml");
    fs::write(&config_yaml_path, CONFIG_YAML_TEMPLATE).with_context(|| {
        format!("failed to write {}", config_yaml_path.display())
    })?;
    output::check("deploy/config.yaml");

    // Write .gitignore.
    let gitignore_path = workspace_dir.join(".gitignore");
    fs::write(&gitignore_path, GITIGNORE_TEMPLATE)
        .with_context(|| format!("failed to write {}", gitignore_path.display()))?;
    output::check(".gitignore");

    // Summary.
    output::success(&format!("Workspace '{}' created at {}", name, workspace_dir.display()));
    output::info("");
    output::info("Next steps:");
    output::info(&format!(
        "  cd {}",
        workspace_dir.display()
    ));
    output::info("  rbnx package new my_package    # create a new package");
    output::info("  rbnx build --config deploy/config.yaml # build all packages");
    output::info("  rbnx deploy deploy/config.yaml         # deploy the full stack");

    Ok(())
}
