use crate::{Config, PackageUnregistrar};
use anyhow::Result;
use std::path::PathBuf;

pub async fn execute(config: Config, target: String) -> Result<()> {
    let unregistrar = PackageUnregistrar::new(config)?;

    // Parse target format
    if target.ends_with(".yaml") || target.ends_with(".yml") {
        // Recipe file
        let recipe_path = PathBuf::from(&target);
        unregistrar.unregister_from_recipe(&recipe_path).await?;
    } else if target.contains('.') {
        // package.capability or package.skill format
        let parts: Vec<&str> = target.splitn(2, '.').collect();
        if parts.len() == 2 {
            let package_name = parts[0];
            let cap_or_skill = parts[1];

            if cap_or_skill.starts_with("cap::") {
                unregistrar
                    .unregister_capability(package_name, cap_or_skill)
                    .await?;
            } else if cap_or_skill.starts_with("skl::") {
                unregistrar
                    .unregister_skill(package_name, cap_or_skill)
                    .await?;
            } else {
                anyhow::bail!(
                    "Invalid format. Expected 'package.cap::name' or 'package.skl::name', got: {}",
                    target
                );
            }
        } else {
            anyhow::bail!(
                "Invalid format. Expected 'package.cap::name' or 'package.skl::name', got: {}",
                target
            );
        }
    } else {
        // Package name
        unregistrar.unregister_package(&target).await?;
    }

    Ok(())
}
