// SPDX-License-Identifier: MulanPSL-2.0
// Init command: create a new robonix project skeleton
//
// Generated structure:
//   <name>/
//   ├── robonix_manifest.yaml
//   ├── primitives/
//   ├── services/
//   ├── skills/
//   └── .gitignore

use anyhow::{Context, Result};
use robonix_cli::output;
use std::fs;
use std::path::Path;

const MANIFEST_TEMPLATE: &str = r#"name: {name}

env:
  LOG: "INFO"

# System services (Robonix core components)
# Each component can optionally specify an endpoint (defaults are shown below).
system:
  nexus: {}
  atlas:
    endpoint: "127.0.0.1:50051"
  executor:
    endpoint: "127.0.0.1:50061"
  pilot:
    endpoint: "127.0.0.1:50071"
    vlm_base_url: ""
    vlm_api_key: ""
    vlm_api_format: openai
  liaison:
    endpoint: "127.0.0.1:50081"

# Primitives (hardware-bound driver modules)
primitives: []
  # - package: com.vendor.my_driver
  #   path: ./primitives/my_driver
  #   name: my_driver

# Services (hardware-independent persistent modules)
services: []
  # - package: com.vendor.my_service
  #   path: ./services/my_service
  #   name: my_service

# Skills (on-demand functional units)
skills: []
  # - package: com.vendor.my_skill
  #   path: ./skills/my_skill
  #   name: my_skill
"#;

const GITIGNORE_TEMPLATE: &str = r#"# Robonix build artifacts
_build/
**/rbnx-build/

# Python
__pycache__/
*.pyc
*.pyo
*.egg-info/

# Proto generated
**/proto_gen/

# OS
.DS_Store
"#;

pub async fn execute(name: &str, path: Option<&Path>) -> Result<()> {
    let target = path
        .map(|p| p.to_path_buf())
        .unwrap_or_else(|| std::path::PathBuf::from(name));

    if target.exists() {
        anyhow::bail!(
            "directory '{}' already exists",
            target.display()
        );
    }

    output::action("Init", &format!("creating project '{}' at {}", name, target.display()));

    // Create directory structure.
    fs::create_dir_all(target.join("primitives"))
        .with_context(|| format!("failed to create {}/primitives", target.display()))?;
    fs::create_dir_all(target.join("services"))?;
    fs::create_dir_all(target.join("skills"))?;

    // Write robonix_manifest.yaml.
    let manifest_content = MANIFEST_TEMPLATE.replace("{name}", name);
    fs::write(
        target.join("robonix_manifest.yaml"),
        manifest_content,
    )?;
    output::check("robonix_manifest.yaml");

    // Write .gitignore.
    fs::write(target.join(".gitignore"), GITIGNORE_TEMPLATE)?;
    output::check(".gitignore");

    output::check("primitives/");
    output::check("services/");
    output::check("skills/");

    output::success(&format!("Project '{}' initialized", name));
    output::info("Next steps:");
    output::info(&format!("  cd {}", target.display()));
    output::info("  rbnx package-new my_driver -t primitive    # create a primitive package");
    output::info("  rbnx package-new my_service -t service     # create a service package");
    output::info("  rbnx build --all                           # build all packages");
    output::info("  rbnx deploy                                # deploy the system");

    Ok(())
}
