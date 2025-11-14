use crate::{Config, PackageQuery};
use anyhow::Result;

pub async fn execute_cap(config: Config, name: String) -> Result<()> {
    let query = PackageQuery::new(config);
    let packages = query.find_by_capability(&name)?;
    if packages.is_empty() {
        println!("No packages found with capability: {}", name);
    } else {
        println!("Packages with capability '{}':", name);
        for pkg in packages {
            println!("  - {}", pkg);
        }
    }
    Ok(())
}

pub async fn execute_skill(config: Config, name: String) -> Result<()> {
    let query = PackageQuery::new(config);
    let packages = query.find_by_skill(&name)?;
    if packages.is_empty() {
        println!("No packages found with skill: {}", name);
    } else {
        println!("Packages with skill '{}':", name);
        for pkg in packages {
            println!("  - {}", pkg);
        }
    }
    Ok(())
}
