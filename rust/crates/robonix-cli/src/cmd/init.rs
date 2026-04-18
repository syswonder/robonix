// SPDX-License-Identifier: MulanPSL-2.0
// Init command: generate a new robonix workspace skeleton.
//
// Usage:  rbnx init <name> [--path <dir>]
// Creates:
//   <name>/
//   ├── robonix_workspace.yaml   (workspace config: env + packages list)
//   ├── deploy/
//   │   └── init.yaml            (deploy profile: target, env overrides, packages_run)
//   ├── packages/                (empty directory for user packages)
//   └── .gitignore

use anyhow::{Context, Result};
use robonix_cli::output;
use std::fs;
use std::path::Path;

const WORKSPACE_YAML_TEMPLATE: &str = r#"workspace: {name}

# Global environment variables (injected into all child processes)
env:
  ROBONIX_ATLAS: "127.0.0.1:50051"
  ROBONIX_META_GRPC_ENDPOINT: "127.0.0.1:50051"
  ROBONIX_ATLAS_ENDPOINT: "http://127.0.0.1:50051"
  ROBONIX_EXECUTOR_ENDPOINT: "http://127.0.0.1:50061"
  ROBONIX_PILOT_ENDPOINT: "http://127.0.0.1:50071"
  RUST_LOG: "robonix_atlas=info,robonix_pilot=info,robonix_executor=info,robonix_liaison=warn"

# Packages in this workspace (url or path, at least one required per entry)
packages: []
  # Examples:
  # - name: com.robonix.pkg.memory
  #   url: https://github.com/xxx
  #
  # - name: my.local.pkg
  #   path: ./packages/my_local_pkg
"#;

const CONFIG_YAML_TEMPLATE: &str = r#"upstream_config: ../robonix_workspace.yaml
target: init

# Environment variable overrides (merged with workspace env, local wins)
env: {}

# Packages and nodes to run (format: "package_name:node_id" or "package_name:all")
packages_run: []
  # Examples:
  # - name: com.robonix.pkg.mapping:all
  # - name: com.robonix.pkg.memory:node_xx
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

    // Validate workspace name to prevent path traversal and invalid names.
    if name.is_empty() {
        anyhow::bail!("workspace name must not be empty");
    }
    if name.contains('/') || name.contains('\\') || name == "." || name == ".." || name.starts_with('.') {
        anyhow::bail!(
            "invalid workspace name '{}': must not contain path separators or start with '.'",
            name
        );
    }
    if !name.chars().all(|c| c.is_alphanumeric() || c == '_' || c == '-') {
        anyhow::bail!(
            "invalid workspace name '{}': only alphanumeric characters, hyphens and underscores are allowed",
            name
        );
    }

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

    // Create deploy/ directory and write init.yaml.
    fs::create_dir_all(workspace_dir.join("deploy"))
        .with_context(|| format!("failed to create {}/deploy", workspace_dir.display()))?;
    let config_yaml_path = workspace_dir.join("deploy").join("init.yaml");
    fs::write(&config_yaml_path, CONFIG_YAML_TEMPLATE).with_context(|| {
        format!("failed to write {}", config_yaml_path.display())
    })?;
    output::check("deploy/init.yaml");

    // Write .gitignore.
    let gitignore_path = workspace_dir.join(".gitignore");
    fs::write(&gitignore_path, GITIGNORE_TEMPLATE)
        .with_context(|| format!("failed to write {}", gitignore_path.display()))?;
    output::check(".gitignore");

    // Summary.
    let ws_abs = workspace_dir.canonicalize().unwrap_or_else(|_| workspace_dir.clone());
    output::success(&format!("Workspace '{}' created at {}", name, ws_abs.display()));
    output::info("");
    output::info("Next steps:");
    output::info(&format!("  cd {}", ws_abs.display()));
    output::info("  rbnx package-new my_package        # create a new package");
    output::info("  rbnx build                          # build all workspace packages");
    output::info("  rbnx deploy                         # deploy using workspace config");
    output::info("  rbnx build init                     # build by deploy target 'init'");
    output::info("  rbnx deploy init                    # deploy by target 'init'");

    Ok(())
}
